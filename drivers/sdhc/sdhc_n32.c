// SPDX-License-Identifier: Apache-2.0
/* Copyright (c) 2026 */

#define DT_DRV_COMPAT nsing_n32_sdio

#include <errno.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/n32_clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/sdhc.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sd/sd_spec.h>
#include <zephyr/sys/util.h>
#include <n32g45x.h>

LOG_MODULE_REGISTER(sdhc_n32, CONFIG_SDHC_LOG_LEVEL);

#define CLK_DIV_LO      GENMASK(7, 0)
#define CLK_EN          BIT(8)
#define CLK_WIDE        BIT(11)
#define CLK_DIV_HI      BIT(15)
#define CMD_SHORT       BIT(6)
#define CMD_LONG        (BIT(6) | BIT(7))
#define CMD_EN          BIT(10)
#define DCTRL_EN        BIT(0)
#define DCTRL_READ      BIT(1)
#define STA_CCRC        BIT(0)
#define STA_DCRC        BIT(1)
#define STA_CTIMEOUT    BIT(2)
#define STA_DTIMEOUT    BIT(3)
#define STA_TXUND       BIT(4)
#define STA_RXOVER      BIT(5)
#define STA_CMDREND     BIT(6)
#define STA_CMDSENT     BIT(7)
#define STA_DATAEND     BIT(8)
#define STA_STARTERR    BIT(9)
#define STA_CMDACT      BIT(11)
#define STA_TXACT       BIT(12)
#define STA_RXACT       BIT(13)
#define STA_TXHALF      BIT(14)
#define STA_RXHALF      BIT(15)
#define STA_RXDAVL      BIT(21)
#define CLEAR_FLAGS     GENMASK(10, 0)
#define CMD_CLEAR_FLAGS (STA_CCRC | STA_CTIMEOUT | STA_CMDREND | STA_CMDSENT)
#define DATA_ERRORS     (STA_DCRC | STA_DTIMEOUT | STA_TXUND | STA_RXOVER | STA_STARTERR)

struct n32_sdhc_config {
	SDIO_Module *regs;
	const struct device *clock;
	struct n32_pclken clock_id;
	const struct pinctrl_dev_config *pcfg;
	uint32_t f_min;
	uint32_t f_max;
	uint8_t width;
};

struct n32_sdhc_data {
	struct k_mutex lock;
	uint32_t source_hz;
};

static bool timed_out(int64_t deadline)
{
	return k_uptime_get() >= deadline;
}

static int n32_command(SDIO_Module *regs, struct sdhc_command *cmd)
{
	uint32_t type = cmd->response_type & SDHC_NATIVE_RESPONSE_MASK;
	uint32_t control = cmd->opcode & GENMASK(5, 0);
	int64_t deadline = k_uptime_get() + (cmd->timeout_ms > 0 ? cmd->timeout_ms : 100);
	uint32_t status;

	regs->INTCLR = CMD_CLEAR_FLAGS;
	regs->CMDARG = cmd->arg;
	control |= type == SD_RSP_TYPE_R2 ? CMD_LONG : (type == SD_RSP_TYPE_NONE ? 0U : CMD_SHORT);
	regs->CMDCTRL = control | CMD_EN;
	do {
		status = regs->STS;
		if ((status & STA_CTIMEOUT) != 0U) {
			regs->INTCLR = CMD_CLEAR_FLAGS;
			return -ETIMEDOUT;
		}
		if ((status & STA_CCRC) != 0U && type != SD_RSP_TYPE_R3 && type != SD_RSP_TYPE_R4) {
			regs->INTCLR = CMD_CLEAR_FLAGS;
			return -EILSEQ;
		}
		if ((status & STA_CCRC) != 0U &&
		    (type == SD_RSP_TYPE_R3 || type == SD_RSP_TYPE_R4)) {
			break;
		}
		if ((type == SD_RSP_TYPE_NONE && (status & STA_CMDSENT) != 0U) ||
		    (type != SD_RSP_TYPE_NONE && (status & STA_CMDREND) != 0U)) {
			break;
		}
		k_yield();
	} while (!timed_out(deadline));
	if (timed_out(deadline)) {
		regs->INTCLR = CMD_CLEAR_FLAGS;
		return -ETIMEDOUT;
	}
	if (type == SD_RSP_TYPE_R2) {
		cmd->response[0] = regs->RESPONSE4;
		cmd->response[1] = regs->RESPONSE3;
		cmd->response[2] = regs->RESPONSE2;
		cmd->response[3] = regs->RESPONSE1;
	} else if (type != SD_RSP_TYPE_NONE) {
		cmd->response[0] = regs->RESPONSE1;
	}
	regs->INTCLR = CMD_CLEAR_FLAGS;
	return 0;
}

static bool is_write_cmd(uint32_t opcode)
{
	return opcode == SD_WRITE_SINGLE_BLOCK || opcode == SD_WRITE_MULTIPLE_BLOCK;
}

static int n32_data_xfer(SDIO_Module *regs, struct sdhc_data *data, bool write)
{
	uint8_t *buf = data->data;
	uint32_t left = data->block_size * data->blocks;
	int64_t deadline = k_uptime_get() + (data->timeout_ms > 0 ? data->timeout_ms : 1000);

	while (left != 0U) {
		uint32_t status = regs->STS;
		uint32_t count;

		if ((status & DATA_ERRORS) != 0U) {
			regs->INTCLR = CLEAR_FLAGS;
			return (status & STA_DTIMEOUT) != 0U ? -ETIMEDOUT : -EIO;
		}
		if (write && (status & STA_TXHALF) != 0U) {
			count = MIN(left, 32U);
		} else if (!write && (status & STA_RXHALF) != 0U) {
			count = MIN(left, 32U);
		} else if (!write && (status & STA_RXDAVL) != 0U) {
			count = MIN(left, 4U);
		} else {
			if (timed_out(deadline)) {
				return -ETIMEDOUT;
			}
			k_yield();
			continue;
		}
		for (uint32_t pos = 0U; pos < count; pos += 4U) {
			uint32_t word = 0U;
			uint32_t chunk = MIN(4U, count - pos);

			if (write) {
				memcpy(&word, buf + pos, chunk);
				regs->DATFIFO = word;
			} else {
				word = regs->DATFIFO;
				memcpy(buf + pos, &word, chunk);
			}
		}
		buf += count;
		left -= count;
	}
	while ((regs->STS & (STA_DATAEND | DATA_ERRORS)) == 0U) {
		if (timed_out(deadline)) {
			return -ETIMEDOUT;
		}
		k_yield();
	}
	if ((regs->STS & DATA_ERRORS) != 0U) {
		regs->INTCLR = CLEAR_FLAGS;
		return -EIO;
	}
	data->bytes_xfered = data->block_size * data->blocks;
	regs->INTCLR = CLEAR_FLAGS;
	return 0;
}

static int n32_request(const struct device *dev, struct sdhc_command *cmd, struct sdhc_data *data)
{
	const struct n32_sdhc_config *cfg = dev->config;
	struct n32_sdhc_data *priv = dev->data;
	bool write = data != NULL && is_write_cmd(cmd->opcode);
	int ret;

	k_mutex_lock(&priv->lock, K_FOREVER);
	if (data != NULL) {
		uint32_t length = data->block_size * data->blocks;

		if (data->data == NULL || data->blocks == 0U ||
		    !is_power_of_two(data->block_size) || data->block_size > 16384U ||
		    length > GENMASK(24, 0)) {
			ret = -EINVAL;
			goto done;
		}
		cfg->regs->DATCTRL = 0U;
		cfg->regs->DTIMER = priv->source_hz;
		cfg->regs->DATLEN = length;
		cfg->regs->DATCTRL =
			(LOG2(data->block_size) << 4) | (write ? 0U : DCTRL_READ) | DCTRL_EN;
	}
	ret = n32_command(cfg->regs, cmd);
	if (ret == 0 && data != NULL) {
		ret = n32_data_xfer(cfg->regs, data, write);
		if (ret == 0 && data->blocks > 1U) {
			struct sdhc_command stop = {
				.opcode = SD_STOP_TRANSMISSION,
				.response_type = SD_RSP_TYPE_R1b,
				.timeout_ms = cmd->timeout_ms,
			};

			cfg->regs->DATCTRL = 0U;
			ret = n32_command(cfg->regs, &stop);
		}
	}
	if (ret != 0) {
		LOG_ERR("CMD%u failed: %d, status 0x%08x", cmd->opcode, ret, cfg->regs->STS);
	}
done:
	cfg->regs->DATCTRL = 0U;
	k_mutex_unlock(&priv->lock);
	return ret;
}

static int n32_set_clock(const struct device *dev, uint32_t hz)
{
	const struct n32_sdhc_config *cfg = dev->config;
	struct n32_sdhc_data *priv = dev->data;
	uint32_t div;
	uint32_t reg;

	if (hz < cfg->f_min || hz > cfg->f_max) {
		return -EINVAL;
	}
	div = DIV_ROUND_UP(priv->source_hz, hz);
	div = div > 2U ? div - 2U : 0U;
	if (div > 0x1ffU) {
		return -EINVAL;
	}
	reg = cfg->regs->CLKCTRL & ~(CLK_DIV_LO | CLK_DIV_HI);
	reg |= div & CLK_DIV_LO;
	reg |= (div & BIT(8)) != 0U ? CLK_DIV_HI : 0U;
	cfg->regs->CLKCTRL = reg | CLK_EN;
	return 0;
}

static int n32_set_io(const struct device *dev, struct sdhc_io *io)
{
	const struct n32_sdhc_config *cfg = dev->config;
	int ret;

	if ((io->signal_voltage != 0U && io->signal_voltage != SD_VOL_3_3_V) ||
	    (io->timing != 0U && io->timing != SDHC_TIMING_LEGACY)) {
		return -ENOTSUP;
	}
	if (io->bus_width != 0U && io->bus_width != SDHC_BUS_WIDTH1BIT &&
	    (io->bus_width != SDHC_BUS_WIDTH4BIT || cfg->width < 4U)) {
		return -ENOTSUP;
	}
	if (io->power_mode == SDHC_POWER_OFF) {
		cfg->regs->CLKCTRL &= ~CLK_EN;
		cfg->regs->PWRCTRL = 0U;
	} else if (io->power_mode == SDHC_POWER_ON) {
		cfg->regs->PWRCTRL = 3U;
	}
	if (io->clock != 0U) {
		ret = n32_set_clock(dev, io->clock);
		if (ret != 0) {
			return ret;
		}
	}
	if (io->bus_width != 0U) {
		cfg->regs->CLKCTRL = (cfg->regs->CLKCTRL & ~CLK_WIDE) |
				     (io->bus_width == SDHC_BUS_WIDTH4BIT ? CLK_WIDE : 0U);
	}
	return 0;
}

static int n32_present(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 1;
}

static int n32_busy(const struct device *dev)
{
	const struct n32_sdhc_config *cfg = dev->config;

	return (cfg->regs->STS & (STA_CMDACT | STA_TXACT | STA_RXACT)) != 0U;
}

static int n32_props(const struct device *dev, struct sdhc_host_props *props)
{
	const struct n32_sdhc_config *cfg = dev->config;

	memset(props, 0, sizeof(*props));
	props->f_min = cfg->f_min;
	props->f_max = cfg->f_max;
	props->host_caps.max_blk_len = 3U;
	props->host_caps.vol_330_support = 1U;
	props->bus_4_bit_support = cfg->width >= 4U;
	return 0;
}

static int n32_reset(const struct device *dev)
{
	const struct n32_sdhc_config *cfg = dev->config;

	cfg->regs->CMDCTRL = 0U;
	cfg->regs->DATCTRL = 0U;
	cfg->regs->INTEN = 0U;
	cfg->regs->INTCLR = CLEAR_FLAGS;
	return 0;
}

static int n32_init(const struct device *dev)
{
	const struct n32_sdhc_config *cfg = dev->config;
	struct n32_sdhc_data *priv = dev->data;
	int ret;

	k_mutex_init(&priv->lock);
	ret = clock_control_on(cfg->clock, (clock_control_subsys_t)&cfg->clock_id);
	if (ret == 0) {
		ret = clock_control_get_rate(cfg->clock, (clock_control_subsys_t)&cfg->clock_id,
					     &priv->source_hz);
	}
	if (ret == 0) {
		ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	}
	if (ret != 0) {
		return ret;
	}
	cfg->regs->PWRCTRL = 3U;
	cfg->regs->CLKCTRL = 0U;
	n32_reset(dev);
	return n32_set_clock(dev, cfg->f_min);
}

static DEVICE_API(sdhc, n32_api) = {
	.reset = n32_reset,
	.request = n32_request,
	.set_io = n32_set_io,
	.get_card_present = n32_present,
	.card_busy = n32_busy,
	.get_host_props = n32_props,
};

#define N32_SDHC_INIT(inst)                                                                        \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
	static struct n32_sdhc_data n32_data_##inst;                                               \
	static const struct n32_sdhc_config n32_config_##inst = {                                  \
		.regs = (SDIO_Module *)DT_INST_REG_ADDR(inst),                                     \
		.clock = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),                                 \
		.clock_id = {.bus = DT_INST_CLOCKS_CELL(inst, bits)},                              \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                      \
		.f_min = DT_INST_PROP(inst, min_bus_freq),                                         \
		.f_max = DT_INST_PROP(inst, max_bus_freq),                                         \
		.width = DT_INST_PROP(inst, bus_width),                                            \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, n32_init, NULL, &n32_data_##inst, &n32_config_##inst,          \
			      POST_KERNEL, CONFIG_SDHC_INIT_PRIORITY, &n32_api);

DT_INST_FOREACH_STATUS_OKAY(N32_SDHC_INIT)
