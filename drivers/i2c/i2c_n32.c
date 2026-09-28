/*
 * Copyright (c) 2024 Nations Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/gpio.h>

#include "i2c_bitbang.h"

#include <zephyr/drivers/clock_control/n32_clock_control.h>

#include <n32g45x_i2c.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>
#include <zephyr/irq.h>

#include <zephyr/sys/sys_io.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/init.h>
#include <zephyr/drivers/clock_control.h>

#include <zephyr/linker/sections.h>

#define DT_DRV_COMPAT nsing_n32_i2c

/* Slack added on top of the computed wire time before giving up on a
 * transfer.  Transfers can legitimately take a long time - a 4 KiB read at
 * 100 kHz is over 350 ms - so the timeout has to follow the transfer rather
 * than sit at a fixed value, or the driver only works for small ones.
 */
#define I2C_TRANSFER_SLACK_MS 100

LOG_MODULE_REGISTER(i2c_n32, CONFIG_I2C_LOG_LEVEL);

struct i2c_n32_config {
	uint32_t reg_base;
	uint32_t apb_clk_en;
	uint32_t i2c_bitrate;
	const struct pinctrl_dev_config *pinctrl_config;
	void (*irq_config_func)(void);

#ifdef CONFIG_I2C_N32_BUS_RECOVERY
	/* The I2C lines, so bus recovery can drive them by hand. */
	struct gpio_dt_spec scl;
	struct gpio_dt_spec sda;
#endif
};

struct i2c_n32_data {
	struct k_sem bus_mutex;
	struct k_sem sync_sem;
	struct i2c_msg *msgs;
	uint16_t slave_addr;

	/* Messages still to be handled in the transfer in progress, and the
	 * transfer's completion semaphore: the ISRs drive the whole message
	 * chain themselves, so the caller waits once for the lot.
	 */
	uint32_t msgs_left;

	/* Set by the ISRs when a transfer is aborted (e.g. slave NACK).
	 * Cleared at the start of every transfer, reported to the caller.
	 */
	int err;
};

#define I2C_N32_FLAG_TIMEOUT ((uint32_t)0x1000)
#define I2C_N32_LONG_TIMEOUT ((uint32_t)(20 * I2C_N32_FLAG_TIMEOUT))

/* Bring the peripheral up from scratch.  Callers must hold bus_mutex, or be
 * setting the bus up for the first time.
 */
static void i2c_n32_hw_init(const struct device *dev, uint32_t bitrate)
{
	const struct i2c_n32_config *cfg = dev->config;

	I2C_DeInit((I2C_Module *)cfg->reg_base);

	I2C_InitType i2c_init = {.BusMode = I2C_BUSMODE_I2C,
				 .FmDutyCycle = I2C_FMDUTYCYCLE_2,
				 .AckEnable = I2C_ACKEN,
				 .AddrMode = I2C_ADDR_MODE_7BIT};

	/* clock-frequency from devicetree is already the bitrate in Hz */
	i2c_init.ClkSpeed = bitrate;

	I2C_Init((I2C_Module *)cfg->reg_base, &i2c_init);
	I2C_Enable((I2C_Module *)cfg->reg_base, ENABLE);
}

static int i2c_n32_configure(const struct device *dev, uint32_t bitrate)
{
	struct i2c_n32_data *data = dev->data;

	k_sem_take(&data->bus_mutex, K_FOREVER);

	i2c_n32_hw_init(dev, bitrate);

	/* release the mutex */
	k_sem_give(&data->bus_mutex);

	return 0;
}

/* The i2c_bitbang library drives the lines through these. */
#ifdef CONFIG_I2C_N32_BUS_RECOVERY
static void i2c_n32_bitbang_set_scl(void *io_context, int state)
{
	const struct i2c_n32_config *config = io_context;

	gpio_pin_set_dt(&config->scl, state);
}

static void i2c_n32_bitbang_set_sda(void *io_context, int state)
{
	const struct i2c_n32_config *config = io_context;

	gpio_pin_set_dt(&config->sda, state);
}

static int i2c_n32_bitbang_get_sda(void *io_context)
{
	const struct i2c_n32_config *config = io_context;

	return gpio_pin_get_dt(&config->sda) == 0 ? 0 : 1;
}
#endif /* CONFIG_I2C_N32_BUS_RECOVERY */

/**
 * @brief Put the peripheral back the way i2c_n32_init() left it.
 *
 * Shared by bus recovery and the transfer timeout path: both leave the
 * peripheral in a state it cannot be used from, and both have to hand back
 * working pins and registers.  Callers must hold bus_mutex.
 */
static void i2c_n32_restore(const struct device *dev)
{
	const struct i2c_n32_config *config = dev->config;

	(void)pinctrl_apply_state(config->pinctrl_config, PINCTRL_STATE_DEFAULT);
	i2c_n32_hw_init(dev, config->i2c_bitrate);
}

/**
 * @brief Free a bus that a slave is holding low.
 *
 * A slave that was interrupted mid-byte (because the MCU was reset during a
 * read) keeps driving SDA low, waiting for clocks it will never get.  While
 * SDA is low the peripheral reports BUSY forever and refuses to issue a
 * START, so every transfer times out.  Toggling the peripheral on and off
 * does not help - the pin has to be clocked free.
 *
 * The standard cure, which i2c_bitbang_recover_bus() implements, is to drive
 * SCL by hand for up to nine cycles, letting the slave shift out the rest of
 * its byte, and then issue the STOP that returns it to idle.  That needs the
 * lines as GPIOs, which is what scl-gpios/sda-gpios in devicetree provide.
 *
 * Callers must hold bus_mutex.  Returns -ENOSYS when recovery is not
 * possible, having reset the peripheral as a best effort.
 */
static int i2c_n32_bus_recover_locked(const struct device *dev)
{
	LOG_WRN("bus busy with no transfer in flight, clocking it free");

#ifdef CONFIG_I2C_N32_BUS_RECOVERY
	const struct i2c_n32_config *config = dev->config;
	I2C_Module *i2c_port = (I2C_Module *)config->reg_base;
	struct i2c_bitbang bitbang_ctx;
	struct i2c_bitbang_io bitbang_io = {
		.set_scl = i2c_n32_bitbang_set_scl,
		.set_sda = i2c_n32_bitbang_set_sda,
		.get_sda = i2c_n32_bitbang_get_sda,
	};
	int err;

	if ((config->scl.port == NULL) || (config->sda.port == NULL)) {
		LOG_ERR("scl-gpios/sda-gpios missing, cannot clock the bus free");
		i2c_n32_restore(dev);
		return -ENOSYS;
	}

	if (!gpio_is_ready_dt(&config->scl) || !gpio_is_ready_dt(&config->sda)) {
		LOG_ERR("bus recovery GPIOs not ready");
		return -EIO;
	}

	/* Park the peripheral before driving its pins by hand. */
	I2C_Enable(i2c_port, DISABLE);

	err = gpio_pin_configure_dt(&config->scl, GPIO_OUTPUT_HIGH);
	if (err != 0) {
		LOG_ERR("failed to configure SCL GPIO (err %d)", err);
		goto restore;
	}

	err = gpio_pin_configure_dt(&config->sda, GPIO_OUTPUT_HIGH);
	if (err != 0) {
		LOG_ERR("failed to configure SDA GPIO (err %d)", err);
		goto restore;
	}

	/* Only nine clocks and a STOP are needed, so the exact speed is
	 * irrelevant; the library's default (standard speed) is slow enough
	 * for any slave to follow.
	 */
	i2c_bitbang_init(&bitbang_ctx, &bitbang_io, (void *)config);

	err = i2c_bitbang_recover_bus(&bitbang_ctx);
	if (err != 0) {
		LOG_ERR("failed to clock the bus free (err %d)", err);
	}

restore:
	i2c_n32_restore(dev);
	return err;
#else
	/* No GPIOs to drive, so all that is left is to start the peripheral
	 * from a clean state and let the caller see the failure.
	 */
	i2c_n32_restore(dev);
	return -ENOSYS;
#endif /* CONFIG_I2C_N32_BUS_RECOVERY */
}

#ifdef CONFIG_I2C_N32_BUS_RECOVERY
/**
 * @brief i2c_driver_api::recover_bus implementation.
 *
 * This is what i2c_recover_bus() calls, so the mutex is taken here rather
 * than assumed.  The transfer path already holds it and calls the _locked
 * version directly.
 */
static int i2c_n32_recover_bus(const struct device *dev)
{
	struct i2c_n32_data *data = dev->data;
	int err;

	k_sem_take(&data->bus_mutex, K_FOREVER);
	err = i2c_n32_bus_recover_locked(dev);
	k_sem_give(&data->bus_mutex);

	return err;
}
#endif /* CONFIG_I2C_N32_BUS_RECOVERY */

/**
 * @brief Move past the message that has just been handled.
 *
 * When more messages belong to the transfer, the next one is started with a
 * repeated START.  Issuing that START here - rather than waiting for the
 * next message to do it - is what clears the BTF (EV8_2) flag: without it the
 * byte-transfer-finished condition stays asserted and the event interrupt
 * re-enters forever.  This mirrors the vendor EEPROM_Int example, which
 * generates the repeated START from the data-sending event.
 *
 * @param issue_stop true for the write path, where the STOP has not been
 *                   programmed yet; false for reads, which set it themselves
 *                   before the final byte.
 */
static void i2c_n32_advance(const struct device *dev, bool issue_stop)
{
	struct i2c_n32_data *data = dev->data;
	const struct i2c_n32_config *config = dev->config;
	I2C_Module *i2c_port = (I2C_Module *)config->reg_base;

	if (data->msgs_left > 1) {
		data->msgs++;
		data->msgs_left--;
		I2C_GenerateStart(i2c_port, ENABLE);
		return;
	}

	data->msgs_left = 0;

	if (issue_stop && (data->msgs->flags & I2C_MSG_STOP)) {
		I2C_GenerateStop(i2c_port, ENABLE);
	}

	/* Idle again.  Mask the sources here rather than when the waiting
	 * thread wakes up: the bus is still busy for a few cycles after the
	 * STOP, and an event arriving in that window would be served against
	 * no message at all.
	 */
	data->msgs = NULL;
	I2C_ConfigInt(i2c_port, I2C_INT_EVENT | I2C_INT_BUF | I2C_INT_ERR, DISABLE);
	k_sem_give(&data->sync_sem);
}

/**
 * @brief Give up on the transfer in progress and release the caller.
 *
 * Whatever triggered this, the ISR must both stop the peripheral from
 * re-asserting the same interrupt and hand the waiting thread back its
 * error - otherwise the thread blocks on sync_sem forever while the ISR
 * spins.  Clearing msgs before masking matters: the interrupt sources are
 * still live for a few cycles, and those events would otherwise be served
 * against a message that is being abandoned.
 */
static void i2c_n32_abort(const struct device *dev, int err)
{
	struct i2c_n32_data *data = dev->data;
	const struct i2c_n32_config *config = dev->config;
	I2C_Module *i2c_port = (I2C_Module *)config->reg_base;

	data->msgs = NULL;
	data->msgs_left = 0;
	data->err = err;
	I2C_ConfigInt(i2c_port, I2C_INT_EVENT | I2C_INT_BUF | I2C_INT_ERR, DISABLE);
	k_sem_give(&data->sync_sem);
}

static void i2c_n32_event_isr(const struct device *dev)
{
	struct i2c_n32_data *data = dev->data;
	const struct i2c_n32_config *config = dev->config;
	I2C_Module *i2c_port = (I2C_Module *)config->reg_base;

	unsigned int last_event = I2C_GetLastEvent(i2c_port);

	/* An event interrupt with no STS1 bit behind it carries no information.
	 * The peripheral raises one when the event sources are unmasked while
	 * the previous transfer is still winding down; BUSY is in STS2, so it
	 * does not show up here.
	 */
	if ((last_event & 0xFFFFU) == 0U) {
		return;
	}

	/* Nothing of ours is in flight, so whatever the peripheral is still
	 * asserting is left over from the transfer that just finished.  A byte
	 * sitting in DAT keeps RXDATNE set, which re-enters this ISR forever
	 * and starves the thread waiting to start the next transfer.  Collect
	 * whatever is pending and mask the sources; the next transfer turns
	 * them back on.
	 */
	if (data->msgs == NULL) {
		if (last_event & I2C_STS1_RXDATNE) {
			(void)I2C_RecvData(i2c_port);
		}
		I2C_ConfigInt(i2c_port, I2C_INT_EVENT | I2C_INT_BUF | I2C_INT_ERR, DISABLE);
		return;
	}

	/* The peripheral is no longer master, so the transfer cannot be
	 * carried on.  This shows up as a start-bit event with MSMODE clear:
	 * the START was never accepted, or arbitration was lost.  It has to be
	 * cleared and the transfer abandoned - returning here without doing
	 * either leaves the event asserted and the ISR re-enters forever,
	 * with no output to say why.
	 */
	if ((last_event & I2C_ROLE_MASTER) != I2C_ROLE_MASTER) {
		if (last_event & I2C_STS1_RXDATNE) {
			(void)I2C_RecvData(i2c_port);
		}
		if (last_event & I2C_STS1_STARTBF) {
			I2C_SendData(i2c_port, 0); /* writing DAT clears SB */
		}
		i2c_n32_abort(dev, -EIO);
		return;
	}

	/* Master mode */
	if ((last_event & I2C_ROLE_MASTER) == I2C_ROLE_MASTER) {
		switch (last_event) {
		case I2C_EVT_MASTER_MODE_FLAG: /* 0x00030001, EV5: START sent */
			/* The peripheral puts the slave address in DAT[7:1] with
			 * DAT[0] as the R/W bit, so the 7-bit address has to be
			 * shifted up by one first - exactly what the vendor demo
			 * does with its EEPROM_ADDRESS 0xA0 for a chip at 0x50.
			 */
			if (data->msgs->flags & I2C_MSG_READ) { /* read */
				I2C_SendAddr7bit(i2c_port, data->slave_addr << 1,
						 I2C_DIRECTION_RECV);
			} else { /* write */
				I2C_SendAddr7bit(i2c_port, data->slave_addr << 1,
						 I2C_DIRECTION_SEND);
			}
			break;
		case I2C_EVT_MASTER_TXMODE_FLAG: /* 0x00070082, EV8 just after EV6 */
			I2C_SendData(i2c_port, *data->msgs->buf);
			data->msgs->buf++;
			data->msgs->len--;
			break;
		/* Both of these say the same thing - TXE is set, so DAT can
		 * take the next byte.  EV8 is "the byte in DAT is shifting
		 * out", EV8_2 adds BTF, "and the one before it is fully on
		 * the bus".  Which one arrives depends only on how quickly
		 * the ISR got here, so both have to feed the next byte: a
		 * handler that ignores EV8_2 stops dead after the first byte
		 * of every write.  Writing DAT is also what clears BTF.
		 */
		case I2C_EVT_MASTER_DATA_SENDING: /* 0x00070080, EV8 */
		case I2C_EVT_MASTER_DATA_SENDED:  /* 0x00070084, EV8_2 */
			/* Only a write message ever has a byte to feed.  A read
			 * message still sees these: the write phase that preceded
			 * it leaves TXE set, and those events arrive after
			 * i2c_n32_advance() has already moved on to the read.
			 * Writing DAT then would push a byte of the receive
			 * buffer onto the bus in the middle of the repeated
			 * START and wedge it.
			 */
			if (data->msgs->flags & I2C_MSG_READ) {
				/* Mask the source too: TXE cannot be cleared without
				 * writing DAT, so leaving it enabled spins here until
				 * the read address phase ends.  EV6_RX brings it back,
				 * and nothing can be missed before then.
				 */
				I2C_ConfigInt(i2c_port, I2C_INT_BUF, DISABLE);
				break;
			}

			if (data->msgs->len > 0) {
				I2C_SendData(i2c_port, *data->msgs->buf);
				data->msgs->buf++;
				data->msgs->len--;
			} else {
				/* Nothing left to send: either a repeated START for
				 * the rest of the transfer or the STOP that ends it.
				 */
				i2c_n32_advance(dev, true);
			}
			break;

		case I2C_EVT_MASTER_RXMODE_FLAG: /* 0x00030002, EV6 */
			/* The receiver is live now, so the buffer interrupt is
			 * wanted again (see the EV8 case).
			 */
			I2C_ConfigInt(i2c_port, I2C_INT_BUF, ENABLE);

			if (data->msgs->len == 1) {
				I2C_ConfigAck(i2c_port, DISABLE);

				/* A single-byte read needs the STOP programmed in the
				 * address phase.  When another message follows, the
				 * repeated START ends this one instead.
				 */
				if (data->msgs_left == 1) {
					I2C_GenerateStop(i2c_port, ENABLE);
				}

			} else if (data->msgs->len == 2) {
				/* NACK the byte after the next one, i.e. end on the last. */
				i2c_port->CTRL1 |= I2C_NACK_POS_NEXT;
				I2C_ConfigAck(i2c_port, DISABLE);
			}
			break;
		/* One byte received: BUSY, MSMODE (master) and RXDATNE set. */
		case I2C_EVT_MASTER_DATA_RECVD_FLAG:
		/* Byte transfer finished while receiving (EV8_2 on ST parts):
		 * BUSY, MSMODE (master), RXDATNE and BSF set.
		 */
		case I2C_EVT_MASTER_SFT_DATA_RECVD_FLAG:

			*data->msgs->buf = I2C_RecvData(i2c_port);
			data->msgs->buf++;
			data->msgs->len--;

			if (data->msgs->len == 0) {
				/* Message complete.  Reads - including the one-byte
				 * case - have already programmed their own STOP, so
				 * only move on to the next message / finish here.
				 */
				i2c_n32_advance(dev, false);
			} else if (data->msgs->len == 1) {
				/* NACK the byte that follows, it is the last. */
				I2C_ConfigAck(i2c_port, DISABLE);

				if (data->msgs_left == 1) {
					/* Send the STOP condition. */
					I2C_GenerateStop(i2c_port, ENABLE);
				}
			}
			break;
		/* Arbitration lost. */
		case 0x00030201:
		/* Acknowledge failure. */
		case 0x00030401:
		/* Acknowledge failure and bus error. */
		case 0x00030501:
			/* Mark the transfer finished before resetting the
			 * peripheral: the reset itself raises events that would
			 * otherwise be served against the message we are
			 * abandoning.
			 */
			i2c_n32_abort(dev, -EIO);
			I2C_Enable(i2c_port, DISABLE);
			I2C_Enable(i2c_port, ENABLE);
			break;
		default:
			/* An event this driver does not know how to clear.  It
			 * would be asserted again the moment the ISR returns, so
			 * abandoning the transfer is the only way out - the caller
			 * gets -EIO rather than a system wedged in the ISR.  Unlike
			 * the error events above there is no known bit to write
			 * back, so reset the peripheral to drop whatever the flag
			 * is; leaving it set would fail the next transfer too.
			 */
			i2c_n32_abort(dev, -EIO);
			I2C_Enable(i2c_port, DISABLE);
			I2C_Enable(i2c_port, ENABLE);
			break;
		}
	}
}

static void i2c_n32_error_isr(const struct device *dev)
{
	struct i2c_n32_data *data = dev->data;
	const struct i2c_n32_config *config = dev->config;
	I2C_Module *i2c_port = (I2C_Module *)config->reg_base;

	if (I2C_GetFlag(i2c_port, I2C_FLAG_ACKFAIL)) {
		I2C_ClrFlag(i2c_port, I2C_FLAG_ACKFAIL);

		/* The slave did not acknowledge; record it so the transfer reports
		 * -EIO instead of "success".  This fires on every poll while an
		 * EEPROM write cycle runs - a NACK there is normal and the caller
		 * retries - so it is reported through data->err, not logged here.
		 * Console output from an ISR would also take longer than a byte on
		 * the wire and destroy the ACK/STOP timing.
		 */
		data->err = -EIO;
		data->msgs = NULL;
		data->msgs_left = 0;

		/* Send the STOP condition. */
		I2C_GenerateStop(i2c_port, ENABLE);
		k_sem_give(&data->sync_sem);
	}
}

static int i2c_n32_transfer_end(const struct device *dev)
{
	struct i2c_n32_data *data = dev->data;
	const struct i2c_n32_config *cfg = dev->config;
	I2C_Module *i2c_port = (I2C_Module *)cfg->reg_base;

	uint32_t timeout = I2C_N32_LONG_TIMEOUT;

	I2C_ConfigInt(i2c_port, I2C_INT_EVENT | I2C_INT_BUF | I2C_INT_ERR, DISABLE);

	/* Every transfer is terminated with a STOP, so wait for the bus to be
	 * released before the next transfer starts.
	 */
	while (I2C_GetFlag(i2c_port, I2C_FLAG_BUSY)) {
		if (timeout-- == 0U) {
			data->err = -ETIMEDOUT;
			break;
		}
	}

	return 0;
}

static int i2c_n32_transfer(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
			    uint16_t slave_addr)
{
	struct i2c_n32_data *data = dev->data;
	const struct i2c_n32_config *config = dev->config;
	I2C_Module *i2c_port = (I2C_Module *)config->reg_base;
	struct i2c_msg *current, *next;
	uint32_t bits, timeout_ms;
	int ret = 0;

	current = msgs;

	/* First message flags set I2C_MSG_RESTART flag. */
	current->flags |= I2C_MSG_RESTART;

	for (uint8_t i = 1; i <= num_msgs; i++) {

		if (i < num_msgs) {
			next = current + 1;

			/*
			 * If there have a R/W transfer state change between messages,
			 * An explicit I2C_MSG_RESTART flag is needed for the second message.
			 */
			if ((current->flags & I2C_MSG_RW_MASK) != (next->flags & I2C_MSG_RW_MASK)) {
				if ((next->flags & I2C_MSG_RESTART) == 0U) {
					return -EINVAL;
				}
			}

			/* Only the last message need I2C_MSG_STOP flag to free the Bus. */
			if (current->flags & I2C_MSG_STOP) {
				return -EINVAL;
			}
		} else {
			/* Last message flags contain I2C_MSG_STOP flag. */
			current->flags |= I2C_MSG_STOP;
		}

		if ((current->buf == NULL) || (current->len == 0U)) {
			return -EINVAL;
		}

		current++;
	}

	k_sem_take(&data->bus_mutex, K_FOREVER);

	/* Fresh transfer: no error recorded yet. */
	data->err = 0;

	data->msgs = msgs;
	data->msgs_left = num_msgs;
	data->slave_addr = slave_addr;

	/* We hold the bus mutex, so nothing of ours is in flight: a busy bus
	 * here means a slave is still holding SDA down from an earlier run and
	 * the START we are about to ask for would never be issued.
	 */
	if (I2C_GetFlag(i2c_port, I2C_FLAG_BUSY)) {
		(void)i2c_n32_bus_recover_locked(dev);
	}

	k_sem_reset(&data->sync_sem);

	/* The tail of a read turns ACK off to NACK its final byte, and it is
	 * only put back by a full I2C_Init.  Without re-arming it here the
	 * master NACKs the very next address it sends, so the second and every
	 * later transfer on the bus fails.
	 */
	I2C_ConfigAck(i2c_port, ENABLE);
	i2c_port->CTRL1 &= ~I2C_NACK_POS_NEXT;

	I2C_ConfigInt(i2c_port, I2C_INT_EVENT | I2C_INT_BUF | I2C_INT_ERR, ENABLE);

	/* How long this transfer should take on the wire, so the timeout scales
	 * with the work asked for instead of being a fixed guess: nine bits per
	 * byte (eight plus the acknowledge), plus an address phase per message.
	 * A slave may stretch the clock, hence the slack on top.
	 */
	bits = 9U * (2U * num_msgs);
	for (uint8_t i = 0; i < num_msgs; i++) {
		bits += 9U * (msgs[i].len + 1U);
	}

	timeout_ms = (uint32_t)DIV_ROUND_UP((uint64_t)bits * 1000U, config->i2c_bitrate);
	timeout_ms += I2C_TRANSFER_SLACK_MS;

	/* Kick the transfer off.  Everything after this point - the data
	 * bytes, the repeated STARTs between messages and the final STOP -
	 * is driven by the event ISR, so the caller waits once for the whole
	 * chain rather than once per message.
	 */
	I2C_GenerateStart(i2c_port, ENABLE);

	if (k_sem_take(&data->sync_sem, K_MSEC(timeout_ms)) != 0) {
		data->err = -ETIMEDOUT;
	}

	if (data->err == -ETIMEDOUT) {
		/* The semaphore expired with the message chain unfinished, so the
		 * ISR is not going to complete it.  Leave the peripheral and its
		 * pins usable for the next transfer, and report the error.
		 */
		LOG_ERR("transfer timed out after %u ms at message %u/%u", timeout_ms,
			num_msgs - data->msgs_left + 1, num_msgs);
		i2c_n32_abort(dev, -ETIMEDOUT);
		i2c_n32_restore(dev);
		ret = data->err;
		goto out;
	}

	(void)i2c_n32_transfer_end(dev);

	ret = data->err;

out:
	/* No transfer in progress anymore. */
	data->msgs = NULL;
	data->msgs_left = 0;

	/* release the mutex */
	k_sem_give(&data->bus_mutex);

	return ret;
}

/* API implementation: init */
static int i2c_n32_init(const struct device *dev)
{
	const struct i2c_n32_config *config = dev->config;
	struct i2c_n32_data *data = dev->data;
	int ret;

	/* Protect the I2C API in a multi-threaded environment. */
	k_sem_init(&data->bus_mutex, 1, 1);

	/* Synchronise the ISRs and the transfer API. */
	k_sem_init(&data->sync_sem, 0, K_SEM_MAX_LIMIT);

	ret = pinctrl_apply_state(config->pinctrl_config, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		LOG_ERR("failed to configure I2C pinctrl (err %d)", ret);
		return ret;
	}

	ret = clock_control_on(DEVICE_DT_GET(DT_NODELABEL(rcc)),
			       (clock_control_subsys_t *)&config->apb_clk_en);
	if (ret < 0) {
		LOG_ERR("failed to enable the I2C clock (err %d)", ret);
		return -EIO;
	}

	ret = i2c_n32_configure(dev, config->i2c_bitrate);
	if (ret != 0) {
		LOG_ERR("failed to configure I2C on init (err %d)", ret);
		return ret;
	}

	config->irq_config_func();

	return 0;
}

static DEVICE_API(i2c, i2c_n32_api) = {
	.configure = i2c_n32_configure,
	.transfer = i2c_n32_transfer,
#ifdef CONFIG_I2C_N32_BUS_RECOVERY
	.recover_bus = i2c_n32_recover_bus,
#endif
};

#ifdef CONFIG_I2C_N32_BUS_RECOVERY
/* The recovery pins are optional in devicetree, so this expands to an empty
 * initializer when the bus cannot be recovered by hand.
 */
#define I2C_N32_RECOVERY_CFG(n)                                                                    \
	.scl = GPIO_DT_SPEC_INST_GET_OR(n, scl_gpios, {0}),                                        \
	.sda = GPIO_DT_SPEC_INST_GET_OR(n, sda_gpios, {0}),
#else
#define I2C_N32_RECOVERY_CFG(n)
#endif

#define I2C_N32_INIT(n)                                                                            \
                                                                                                   \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static void i2c_n32_irq_config_##n(void)                                                   \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, event, irq),                                    \
			    DT_INST_IRQ_BY_NAME(n, event, priority), i2c_n32_event_isr,            \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQ_BY_NAME(n, event, irq));                                    \
                                                                                                   \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, error, irq),                                    \
			    DT_INST_IRQ_BY_NAME(n, error, priority), i2c_n32_error_isr,            \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQ_BY_NAME(n, error, irq));                                    \
	};                                                                                         \
                                                                                                   \
	static struct i2c_n32_data i2c_n32_dev_data_##n;                                           \
	static const struct i2c_n32_config i2c_n32_dev_cfg_##n = {                                 \
		.reg_base = DT_INST_REG_ADDR(n),                                                   \
		.apb_clk_en = DT_INST_CLOCKS_CELL(n, bits),                                        \
		.i2c_bitrate = DT_INST_PROP(n, clock_frequency),                                   \
		.pinctrl_config = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                               \
		.irq_config_func = i2c_n32_irq_config_##n,                                         \
		I2C_N32_RECOVERY_CFG(n)};                                                          \
                                                                                                   \
	I2C_DEVICE_DT_INST_DEFINE(n, i2c_n32_init, NULL, &i2c_n32_dev_data_##n,                    \
				  &i2c_n32_dev_cfg_##n, POST_KERNEL, CONFIG_I2C_INIT_PRIORITY,     \
				  &i2c_n32_api);

DT_INST_FOREACH_STATUS_OKAY(I2C_N32_INIT);
