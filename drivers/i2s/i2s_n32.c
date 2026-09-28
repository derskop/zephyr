/*
 * Copyright (c) 2024 Nations Technologies
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * I2S driver for the Nations N32G45x, using the DMA driver in dma_n32.c.
 *
 * The I2S block is the SPI block in I2S mode (I2SCFG.MODSEL), so this driver
 * shares a register file with spi_n32.c: a board enables either the SPI node
 * or the I2S node of a given instance, never both. The I2S half of the vendor
 * HAL lives in n32g45x_spi.c; there is no separate n32g45x_i2s.c.
 *
 * Four properties of the hardware shape everything below.
 *
 * 1. I2SCFG.MODCFG selects exactly one of slave-TX, slave-RX, master-TX and
 *    master-RX, so an instance cannot transmit and receive at the same time.
 *    I2S_DIR_BOTH is therefore answered with -ENOSYS, and a board that needs
 *    both directions at once enables two instances (i2s2 and i2s3 on this
 *    SoC). Both directions can still be *configured* on one instance,
 *    because configure() only validates and stores; the register file is
 *    programmed at START, when the direction being started is known.
 *
 * 2. There is no internal loopback in the block, so I2S_OPT_LOOPBACK is
 *    accepted and ignored, exactly as in i2s_stm32.c and i2s_mcux_sai.c. A
 *    board that runs the loopback tests wires the two instances together.
 *
 * 3. The frame clock prescaler is an integer divider off SYSCLK with no
 *    clock-source mux, and the vendor HAL silently clamps it when a request
 *    cannot be met. configure() computes and range-checks the divider
 *    itself; see i2s_n32_prescaler().
 *
 *    Because there is no mux, SYSCLK is whatever the SoC startup code chose,
 *    and the only number the driver has for it is the clock-frequency the
 *    devicetree declares. init() therefore checks that the PLL is actually
 *    the source before trusting that number: SetSysClock() falls back to the
 *    8 MHz HSI without reporting anything and leaves SystemCoreClock claiming
 *    the full rate, which would otherwise show up as a frame clock 18 times
 *    too slow rather than as an error.
 *
 * 4. OVR and UDR are informational: nothing in the API can be made to react
 *    to them that is not already visible at a DMA block boundary (a slab
 *    running dry, or a TX queue running empty). No interrupt is registered.
 *    The flags are cleared with the F1-style "read DAT, then read STS"
 *    sequence, and only while the block is disabled -- reading DAT on a
 *    running RX stream consumes a received word.
 *
 * 5. Nothing that can run while a stream is running logs above LOG_DBG. A
 *    rejected configure() or trigger() is reported through its return value,
 *    and an underrun or overrun through the ERROR state; neither is worth a
 *    console write. That is not politeness: ztest builds select
 *    LOG_DEFAULT_MINIMAL, which routes LOG_ERR straight to printk and thus to
 *    a blocking UART. Measured on this board at 115200 baud, the rejected-
 *    configure message in configure() held the CPU for 8.3 ms, while one
 *    BLOCK_SIZE (128 byte) transfer at the 8000 Hz frame clock of the test
 *    suite lasts 3.9 ms. Logging there starves the stream and turns a
 *    recoverable state error into an underrun. init() does log at LOG_ERR,
 *    because no stream can be running yet.
 *
 * Buffer ownership, which is what most of the state machine is about:
 *
 *  - TX: a block written with i2s_write() belongs to the driver from the
 *    moment it is queued until it is handed back with k_mem_slab_free(),
 *    which happens exactly once, either when its DMA transfer completes, or
 *    when the queue is discarded by DROP/PREPARE. The driver never allocates
 *    a TX block.
 *  - RX: the driver allocates one block per armed DMA transfer. A block read
 *    with i2s_read() belongs to the application and is freed by the caller
 *    (i2s_buf_read() does it). All blocks the driver still holds are returned
 *    by DROP and PREPARE, so after PREPARE the slab is completely free.
 */

#define DT_DRV_COMPAT nsing_n32_i2s

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/n32_clock_control.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/clock/n32_clock.h>
#include <zephyr/dt-bindings/dma/n32_dma.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <n32g45x.h>
#include <n32g45x_spi.h>

LOG_MODULE_REGISTER(i2s_n32, CONFIG_I2S_LOG_LEVEL);

/*
 * AFIO->RMP_CFG.SWJ_CFG is bits 26:24. 010 releases the JTAG-dedicated pins
 * (PA15, PB3, PB4) while keeping the SW-DP, so flashing and debugging over
 * SWD keep working. Bit 27 is reserved and is left alone by the mask.
 */
#define I2S_N32_SWJ_CFG_MASK GENMASK(26, 24)
#define I2S_N32_SWJ_CFG_SWD_ONLY 0x02000000U

/* I2SDIV is 8 bits and must be >= 2; I2SODD is the low bit of the same
 * divider, so the divider itself spans 2 * I2SDIV + I2SODD == [4, 511].
 */
#define I2S_N32_DIV_MIN 4U
#define I2S_N32_DIV_MAX 511U

/* Legal minimum of I2SDIV, used for a target whose prescaler is unused. */
#define I2S_N32_DIV_DEFAULT 2U

/* The shortest frame that a byte can carry, used to pick I2S_DATA_FMT_*. */
#define I2S_N32_SHORT_WORD_BITS 16U

/*
 * RCC->CFG.SCLKSTS reports the SYSCLK source. The HAL keeps the mask private
 * to n32g45x_rcc.c, so only RCC_GetSysclkSrc() is usable; it returns the field
 * itself, and its documentation names 0x08 as the PLL.
 */
#define I2S_N32_SYSCLK_PLL 0x08U

struct i2s_n32_queue_item {
	void *mem_block;
	size_t size;
};

struct i2s_n32_stream {
	const struct device *dev_dma;
	SPI_Module *regs;
	struct k_msgq *msgq;
	uint32_t dma_channel;
	uint16_t dma_req; /* SPI_I2S_DMA_RX or SPI_I2S_DMA_TX */
	bool is_tx;

	/*
	 * The block the DMA is armed with, or NULL when no transfer is in
	 * flight. Non-NULL is this driver's definition of "busy", and it is
	 * what decide whether STOP/DRAIN can complete right away.
	 */
	void *mem_block;

	struct dma_config dma_cfg;
	struct i2s_config cfg;
	enum i2s_state state;

	/* Set by STOP, cleared by DRAIN: both end in STOPPING, but only
	 * DRAIN keeps sending what is still queued.
	 */
	bool stop_for_drain;

	/* Resolved by configure(), written to the block by START. */
	uint16_t i2s_mode;     /* I2S_MODE_* */
	uint16_t i2s_standard; /* I2S_STD_* */
	uint16_t i2s_data_fmt; /* I2S_DATA_FMT_* */
	uint16_t i2s_clkpol;   /* I2S_CLKPOL_* */
	uint16_t i2s_prescaler;
};

struct i2s_n32_data {
	struct i2s_n32_stream rx;
	struct i2s_n32_stream tx;
};

struct i2s_n32_config {
	SPI_Module *regs;
	uint32_t apb_clk_en;
	uint32_t src_clk_hz;
	const struct pinctrl_dev_config *pcfg;
	bool jtag_release;
};

static unsigned int div_round_closest(uint32_t dividend, uint32_t divisor)
{
	return (dividend + (divisor / 2U)) / divisor;
}

static bool queue_is_empty(struct k_msgq *q)
{
	return k_msgq_num_used_get(q) == 0U;
}

/*
 * Get data from the queue
 */
static int queue_get(struct k_msgq *q, void **mem_block, size_t *size, int32_t timeout)
{
	struct i2s_n32_queue_item item;
	int ret = k_msgq_get(q, &item, SYS_TIMEOUT_MS(timeout));

	if (ret == 0) {
		*mem_block = item.mem_block;
		*size = item.size;
	}

	return ret;
}

/*
 * Put data in the queue
 */
static int queue_put(struct k_msgq *q, void *mem_block, size_t size, int32_t timeout)
{
	struct i2s_n32_queue_item item = {.mem_block = mem_block, .size = size};

	return k_msgq_put(q, &item, SYS_TIMEOUT_MS(timeout));
}

/*
 * Give every queued block back to the slab. The caller must have already
 * stopped the stream, so that no block can be added while this runs.
 */
static void stream_queue_drop(struct i2s_n32_stream *stream)
{
	void *mem_block;
	size_t size;

	while (queue_get(stream->msgq, &mem_block, &size, 0) == 0) {
		LOG_DBG("Dropping item from queue");
		k_mem_slab_free(stream->cfg.mem_slab, mem_block);
	}
}

/*
 * Stop the DMA, and return the block it was armed with.
 *
 * Clearing I2SEN resets the block and stops the bit clock on a controller.
 * That is not what the API asks for by default: the default option,
 * I2S_OPT_BIT_CLK_CONT, keeps the clock running so that a target on the other
 * side of the bus can finish the frame it is in the middle of. Only
 * I2S_OPT_BIT_CLK_GATED asks for the clock to be cut, so only then is I2SEN
 * cleared. I2S_Init() clears it too, so a later START always re-programs the
 * block from scratch.
 */
static void stream_disable(struct i2s_n32_stream *stream)
{

	if ((stream->cfg.options & I2S_OPT_BIT_CLK_GATED) != 0U) {
		I2S_Enable(stream->regs, DISABLE);
	}

	SPI_I2S_EnableDma(stream->regs, stream->dma_req, DISABLE);
	dma_stop(stream->dev_dma, stream->dma_channel);

	if (stream->mem_block != NULL) {
		k_mem_slab_free(stream->cfg.mem_slab, stream->mem_block);
		stream->mem_block = NULL;
	}
}

/*
 * Frame clock prescaler.
 *
 *	Fs = SYSCLK / (32 * packet_length * (2 * I2SDIV + I2SODD))
 *
 * so the whole divider is 2 * I2SDIV + I2SODD. The vendor HAL derives it the
 * same way but then rewrites the register with I2SDIV = 2 whenever the result
 * is out of range (n32g45x_spi.c, the "Test if the divider is 1 or 0 or
 * greater than 0xFF" block), which turns an unsatisfiable request into a
 * wildly wrong clock instead of an error. For 16-bit data that is not a corner
 * case: packet_length is 1 for I2S_DATA_FMT_16BITS, which puts the lowest
 * reachable frame clock at 144 MHz / (32 * 511) = 8806 Hz, so a request for
 * the 8 kHz of the test suite would come out 140 times too fast. That is why
 * 16-bit data is mapped to I2S_DATA_FMT_16BITS_EXTENDED (16 bits of data in a
 * 32-bit slot, packet_length 2), and why the divider is computed and checked
 * here rather than left to the HAL.
 */
static int i2s_n32_prescaler(uint32_t src_clk_hz, uint32_t frame_clk_freq,
			     uint32_t packet_length, uint16_t *prescaler)
{
	uint32_t denom;
	uint32_t div;
	uint32_t max_freq = UINT32_MAX / (32U * packet_length);

	if (frame_clk_freq > max_freq) {
		LOG_DBG("frame clock %u Hz is out of range for a %u Hz SYSCLK",
			frame_clk_freq, src_clk_hz);
		return -EINVAL;
	}

	denom = 32U * packet_length * frame_clk_freq;
	div = div_round_closest(src_clk_hz, denom);

	if (div < I2S_N32_DIV_MIN || div > I2S_N32_DIV_MAX) {
		LOG_DBG("frame clock %u Hz cannot be generated from %u Hz SYSCLK "
			"(reachable: %u..%u Hz)",
			frame_clk_freq, src_clk_hz,
			src_clk_hz / (32U * packet_length * I2S_N32_DIV_MAX),
			src_clk_hz / (32U * packet_length * I2S_N32_DIV_MIN));
		return -EINVAL;
	}

	/* I2SDIV is the divider halved, I2SODD is its low bit. */
	*prescaler = (uint16_t)((div / 2U) | ((div & 1U) << 8));

	return 0;
}

/*
 * Write the configuration to the block. Called from START, not from
 * configure(), because MODCFG can only name one direction at a time and one
 * instance may hold an RX and a TX configuration at once.
 */
static void stream_hw_setup(struct i2s_n32_stream *stream)
{
	I2S_InitType init = {
		/*
		 * The vendor HAL spells the slave modes I2S_MODE_SlAVE_*, with
		 * a lowercase l in "SlAVE"; the names below are the real ones.
		 */
		.I2sMode = stream->i2s_mode,
		.Standard = stream->i2s_standard,
		.DataFormat = stream->i2s_data_fmt,
		/* There is no MCLK pin on either I2S instance of this SoC, and
		 * enabling the output would switch the prescaler to its
		 * SYSCLK / 256 form.
		 */
		.MCLKEnable = I2S_MCLK_DISABLE,
		/*
		 * I2S_AUDIO_FREQ_DEFAULT makes I2S_Init() skip its own
		 * frequency division (and its ClocksFreq read) and write
		 * I2SDIV = 2, which the next line replaces with the value
		 * configure() computed and range-checked.
		 */
		.AudioFrequency = I2S_AUDIO_FREQ_DEFAULT,
		.CLKPOL = stream->i2s_clkpol,
	};

	/*
	 * I2S_Init() clears I2SEN, MODCFG, STDSEL, CKPOL, TDATLEN and CHBITS
	 * and rewrites I2SCFG from those fields, so it is safe on an instance
	 * that has been used before, and it is the only way to change the
	 * direction.
	 */
	I2S_Init(stream->regs, &init);
	stream->regs->I2SPREDIV = stream->i2s_prescaler;

	/*
	 * Clear OVR/UDR. Must stay on a path that only runs while the block
	 * is disabled: a read of DAT on a running RX stream would consume a
	 * received word.
	 */
	(void)stream->regs->DAT;
	(void)stream->regs->STS;
}

/*
 * Program the channel for a transfer. Used for the first block of a stream;
 * later blocks go through i2s_n32_dma_reload().
 *
 * dma_n32_config() rejects block_count != 1, so a stream is always a single
 * block re-armed from the completion callback.
 */
static int i2s_n32_dma_config(struct i2s_n32_stream *stream, void *buf, size_t size)
{
	struct dma_block_config blk = {0};
	struct dma_config *dcfg = &stream->dma_cfg;
	int ret;

	blk.block_size = size;
	if (stream->is_tx) {
		blk.source_address = (uint32_t)buf;
		blk.dest_address = (uint32_t)&stream->regs->DAT;
		blk.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		blk.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	} else {
		blk.source_address = (uint32_t)&stream->regs->DAT;
		blk.dest_address = (uint32_t)buf;
		blk.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		blk.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	}

	dcfg->head_block = &blk;

	ret = dma_config(stream->dev_dma, stream->dma_channel, dcfg);
	if (ret < 0) {
		return ret;
	}

	return dma_start(stream->dev_dma, stream->dma_channel);
}

/*
 * Point the configured channel at the next block and start it. The size may
 * be shorter than cfg.block_size, which is how a last partial TX block is
 * sent.
 */
static int i2s_n32_dma_reload(struct i2s_n32_stream *stream, void *buf, size_t size)
{
	int ret;

	if (stream->is_tx) {
		ret = dma_reload(stream->dev_dma, stream->dma_channel,
				 (uint32_t)buf, (uint32_t)&stream->regs->DAT, size);
	} else {
		ret = dma_reload(stream->dev_dma, stream->dma_channel,
				 (uint32_t)&stream->regs->DAT, (uint32_t)buf, size);
	}

	if (ret < 0) {
		return ret;
	}

	return dma_start(stream->dev_dma, stream->dma_channel);
}

static int stream_start(struct i2s_n32_stream *stream)
{
	size_t size;
	int ret;

	__ASSERT_NO_MSG(stream->mem_block == NULL);

	if (stream->is_tx) {
		/* START with an empty TX queue is -ENOMEM. */
		ret = queue_get(stream->msgq, &stream->mem_block, &size, 0);
		if (ret < 0) {
			return -ENOMEM;
		}
	} else {
		ret = k_mem_slab_alloc(stream->cfg.mem_slab, &stream->mem_block, K_NO_WAIT);
		if (ret < 0) {
			return -ENOMEM;
		}
		size = stream->cfg.block_size;
	}

	stream_hw_setup(stream);

	ret = i2s_n32_dma_config(stream, stream->mem_block, size);
	if (ret < 0) {
		LOG_DBG("failed to start the %s DMA transfer: %d", stream->is_tx ? "TX" : "RX",
			ret);
		goto err_free;
	}

	/*
	 * Arm the request last. With I2SEN still clear the peripheral is not
	 * shifting, but TXE already reads as set, so the DMA loads DAT once
	 * and the first word is ready the moment the clock starts.
	 */
	SPI_I2S_EnableDma(stream->regs, stream->dma_req, ENABLE);
	I2S_Enable(stream->regs, ENABLE);

	return 0;

err_free:
	if (stream->mem_block != NULL) {
		k_mem_slab_free(stream->cfg.mem_slab, stream->mem_block);
		stream->mem_block = NULL;
	}

	return ret;
}

/*
 * Shared tail of STOP and DRAIN. The caller holds the interrupt lock.
 *
 * mem_block is non-NULL exactly between arming a transfer and the completion
 * callback taking the block back, so it says whether a block boundary is still
 * coming. The interrupt lock is what makes that test meaningful: without it a
 * completion could land between the test and the state change and leave the
 * stream in STOPPING with nothing left to complete it.
 */
static void stream_stop(struct i2s_n32_stream *stream, bool drain)
{
	if (stream->mem_block != NULL) {
		stream->state = I2S_STATE_STOPPING;
		stream->stop_for_drain = drain;
		return;
	}

	stream_disable(stream);
	stream->state = I2S_STATE_READY;
}

/*
 * The DMA driver reports a completed block as status 0 and a transfer error as
 * a negative errno; it does not use DMA_STATUS_COMPLETE.
 */
static void dma_rx_callback(const struct device *dma_dev, void *user_data, uint32_t channel,
			    int status)
{
	struct i2s_n32_stream *stream = user_data;
	void *filled;

	ARG_UNUSED(dma_dev);
	ARG_UNUSED(channel);

	if (status < 0) {
		LOG_DBG("RX DMA error: %d", status);
		stream->state = I2S_STATE_ERROR;
		goto rx_disable;
	}

	__ASSERT_NO_MSG(stream->mem_block != NULL);

	filled = stream->mem_block;

	/*
	 * Arm the next buffer before handing out the filled one, so the
	 * target never spends time without a block to receive into. No free
	 * block means the application has not read fast enough: that is the
	 * overrun the API asks to be reported as ERROR state.
	 */
	if (k_mem_slab_alloc(stream->cfg.mem_slab, &stream->mem_block, K_NO_WAIT) < 0) {
		/* The failed allocation has already cleared mem_block -- see
		 * k_mem_slab_alloc(), which writes NULL on the way out -- so
		 * the stream no longer names the block that just filled up.
		 * Nothing else holds it either: it never reached the queue, so
		 * stream_disable() below has nothing to give back and the slab
		 * would lose the block for good.
		 */
		LOG_DBG("RX overrun: no free block in the slab");
		k_mem_slab_free(stream->cfg.mem_slab, filled);
		stream->state = I2S_STATE_ERROR;
		goto rx_disable;
	}

	if (i2s_n32_dma_reload(stream, stream->mem_block, stream->cfg.block_size) < 0) {
		LOG_DBG("failed to arm the next RX block");
		/* mem_block now owns the fresh block, so the filled one has
		 * to be returned explicitly or it would be lost.
		 */
		k_mem_slab_free(stream->cfg.mem_slab, filled);
		stream->state = I2S_STATE_ERROR;
		goto rx_disable;
	}

	if (queue_put(stream->msgq, filled, stream->cfg.block_size, 0) < 0) {
		/* The driver's own queue is full, which is an overrun of the
		 * read side rather than of the slab. Same handling.
		 */
		LOG_DBG("RX overrun: no free slot in the queue");
		k_mem_slab_free(stream->cfg.mem_slab, filled);
		stream->state = I2S_STATE_ERROR;
		goto rx_disable;
	}

	/*
	 * STOP and DRAIN end reception at a block boundary. The block that
	 * just landed stays in the queue: only DROP discards queued blocks.
	 */
	if (stream->state == I2S_STATE_STOPPING) {
		stream->state = I2S_STATE_READY;
		goto rx_disable;
	}

	return;

rx_disable:
	stream_disable(stream);
}

static void dma_tx_callback(const struct device *dma_dev, void *user_data, uint32_t channel,
			    int status)
{
	struct i2s_n32_stream *stream = user_data;
	size_t size;
	int ret;

	ARG_UNUSED(dma_dev);
	ARG_UNUSED(channel);

	if (status < 0) {
		LOG_DBG("TX DMA error: %d", status);
		stream->state = I2S_STATE_ERROR;
		goto tx_disable;
	}

	__ASSERT_NO_MSG(stream->mem_block != NULL);

	/* The block the application handed to i2s_write() is done with. */
	k_mem_slab_free(stream->cfg.mem_slab, stream->mem_block);
	stream->mem_block = NULL;

	if (stream->state == I2S_STATE_STOPPING) {
		if (queue_is_empty(stream->msgq)) {
			/* DRAIN: everything that was written has been sent. */
			stream->state = I2S_STATE_READY;
			goto tx_disable;
		}

		if (!stream->stop_for_drain) {
			/* STOP: end at this block boundary and leave the rest
			 * of the queue alone, so that a later START resumes
			 * with the next block.
			 */
			stream->state = I2S_STATE_READY;
			goto tx_disable;
		}

		/* DRAIN with blocks still queued: keep going. */
	}

	ret = queue_get(stream->msgq, &stream->mem_block, &size, 0);
	if (ret < 0) {
		if (stream->state == I2S_STATE_STOPPING) {
			stream->state = I2S_STATE_READY;
		} else {
			/* Still RUNNING with nothing left to send: the
			 * controller has run out of data, which is the
			 * underrun the API reports as ERROR state.
			 */
			LOG_DBG("TX underrun: no block queued");
			stream->state = I2S_STATE_ERROR;
		}
		goto tx_disable;
	}

	if (i2s_n32_dma_reload(stream, stream->mem_block, size) < 0) {
		LOG_DBG("failed to arm the next TX block");
		stream->state = I2S_STATE_ERROR;
		goto tx_disable;
	}

	return;

tx_disable:
	/* stream_disable() frees the block this stream still holds, if
	 * any, and leaves the bit clock running unless it was asked to be
	 * gated.
	 */
	stream_disable(stream);
}

static int i2s_n32_configure(const struct device *dev, enum i2s_dir dir,
			     const struct i2s_config *cfg)
{
	const struct i2s_n32_config *const dev_cfg = dev->config;
	struct i2s_n32_data *const data = dev->data;
	struct i2s_n32_stream *stream;
	uint16_t standard;
	uint16_t data_fmt;
	uint16_t prescaler = I2S_N32_DIV_DEFAULT;
	uint32_t packet_length;
	uint32_t data_size;
	bool target;
	int ret;

	switch (dir) {
	case I2S_DIR_RX:
		stream = &data->rx;
		break;
	case I2S_DIR_TX:
		stream = &data->tx;
		break;
	case I2S_DIR_BOTH:
		/* MODCFG names one direction. i2s_configure(I2S_DIR_BOTH) is
		 * not reached through the API, which splits it into a TX and
		 * an RX call, but be explicit about the hardware limit.
		 */
		return -ENOSYS;
	default:
		LOG_DBG("either RX or TX direction must be selected");
		return -EINVAL;
	}

	if (stream->state != I2S_STATE_NOT_READY && stream->state != I2S_STATE_READY) {
		LOG_DBG("configuration is only possible in NOT_READY or READY state");
		return -EINVAL;
	}

	/*
	 * frame_clk_freq == 0 un-configures the stream. This has to be
	 * checked before anything else: it is the documented way to tear a
	 * stream down, and the caller is expected to pass nothing else of
	 * value, so every other field would fail validation.
	 */
	if (cfg->frame_clk_freq == 0U) {
		stream_disable(stream);
		stream_queue_drop(stream);
		memset(&stream->cfg, 0, sizeof(stream->cfg));
		stream->state = I2S_STATE_NOT_READY;
		return 0;
	}

	/*
	 * Everything below only reads cfg, so a rejected configuration leaves
	 * the stream exactly as it was.
	 */
	if (cfg->mem_slab == NULL) {
		LOG_DBG("mem_slab must be provided");
		return -EINVAL;
	}

	/*
	 * An I2S-format frame is always two words wide, so channels is
	 * ignored for it -- but the API still documents 1 and 2 as the only
	 * valid values. Above 2 it cannot be expressed by the block in any
	 * case: CHBITS distinguishes 16- from 32-bit slots and nothing else.
	 */
	if (cfg->channels < 1U || cfg->channels > 2U) {
		LOG_DBG("channels must be 1 or 2 (%u)", cfg->channels);
		return -EINVAL;
	}

	switch (cfg->word_size) {
	case I2S_N32_SHORT_WORD_BITS:
		/*
		 * I2S_DATA_FMT_16BITS is deliberately not used: it is the one
		 * format with packet_length 1, which floors the frame clock at
		 * 8806 Hz on a 144 MHz SYSCLK. The extended format carries
		 * the same 16 bits of data in a 32-bit slot and keeps the
		 * divider range usable down to 4403 Hz. This is also what the
		 * vendor examples configure.
		 */
		data_fmt = I2S_DATA_FMT_16BITS_EXTENDED;
		data_size = 2U;
		break;
	case 32U:
		data_fmt = I2S_DATA_FMT_32BITS;
		data_size = 4U;
		break;
	case 24U:
		/* 24 bits in a 32-bit slot needs a packing convention that
		 * nothing in the vendor library documents.
		 */
		LOG_DBG("24-bit words are not supported");
		return -ENOTSUP;
	default:
		LOG_DBG("word_size must be 16, 24 or 32 (%u)", cfg->word_size);
		return -EINVAL;
	}

	switch (cfg->format & I2S_FMT_DATA_FORMAT_MASK) {
	case I2S_FMT_DATA_FORMAT_I2S:
		standard = I2S_STD_PHILLIPS;
		break;
	case I2S_FMT_DATA_FORMAT_PCM_SHORT:
		standard = I2S_STD_PCM_SHORTFRAME;
		break;
	case I2S_FMT_DATA_FORMAT_PCM_LONG:
		standard = I2S_STD_PCM_LONGFRAME;
		break;
	case I2S_FMT_DATA_FORMAT_LEFT_JUSTIFIED:
		standard = I2S_STD_MSB_ALIGN;
		break;
	case I2S_FMT_DATA_FORMAT_RIGHT_JUSTIFIED:
		standard = I2S_STD_LSB_ALIGN;
		break;
	default:
		/* LEFT_JUSTIFIED | RIGHT_JUSTIFIED has no meaning as a
		 * bitmask and lands here by construction.
		 */
		LOG_DBG("unsupported data format 0x%x", cfg->format);
		return -EINVAL;
	}

	if ((cfg->format & I2S_FMT_DATA_ORDER_LSB) != 0U) {
		/* There is no bit for the word order within a slot. */
		LOG_DBG("LSB-first data order is not supported");
		return -EINVAL;
	}

	if ((cfg->format & I2S_FMT_FRAME_CLK_INV) != 0U) {
		/* CLKPOL inverts the bit clock only. In the Philips standard
		 * the frame clock is always low for the left channel.
		 */
		LOG_DBG("an inverted frame clock is not supported");
		return -EINVAL;
	}

	target = (cfg->options & I2S_OPT_BIT_CLK_TARGET) != 0U;
	if (target != ((cfg->options & I2S_OPT_FRAME_CLK_TARGET) != 0U)) {
		/* MODCFG carries a single role for the whole block, so the
		 * two clocks cannot disagree. i2s_stm32.c treats "either bit
		 * set" as target, which silently gives the caller a target
		 * when only one of the two was asked for; refuse instead.
		 */
		LOG_DBG("BIT_CLK and FRAME_CLK must have the same role");
		return -EINVAL;
	}

	if ((cfg->options & I2S_OPT_PINGPONG) != 0U) {
		LOG_DBG("ping-pong mode is not supported");
		return -ENOTSUP;
	}

	if ((cfg->options & I2S_OPT_LOOPBACK) != 0U) {
		LOG_DBG("I2S_OPT_LOOPBACK ignored: the block has no internal loopback");
	}

	if (cfg->block_size == 0U || cfg->block_size % data_size != 0U) {
		/* The DMA counts in elements, so a block that is not a whole
		 * number of samples would leave a tail untransferred.
		 */
		LOG_DBG("block_size must be a non-zero multiple of %u (%zu)", data_size,
			cfg->block_size);
		return -EINVAL;
	}

	if (target) {
		/*
		 * A target does not drive the frame clock, so the prescaler is
		 * unused and frame_clk_freq is informational. Do not
		 * range-check it: an application may legitimately state the
		 * rate it expects on the bus even when this SoC could not
		 * generate it as a controller. I2S_AUDIO_FREQ_DEFAULT leaves
		 * the same I2SDIV = 2 that I2S_Init() itself would write.
		 */
		prescaler = I2S_N32_DIV_DEFAULT;
	} else {
		packet_length = (data_fmt == I2S_DATA_FMT_16BITS) ? 1U : 2U;

		ret = i2s_n32_prescaler(dev_cfg->src_clk_hz, cfg->frame_clk_freq, packet_length,
					&prescaler);
		if (ret < 0) {
			return ret;
		}
	}

	stream->i2s_mode = target ? (dir == I2S_DIR_TX ? I2S_MODE_SlAVE_TX : I2S_MODE_SlAVE_RX)
				  : (dir == I2S_DIR_TX ? I2S_MODE_MASTER_TX : I2S_MODE_MASTER_RX);
	stream->i2s_standard = standard;
	stream->i2s_data_fmt = data_fmt;
	stream->i2s_clkpol =
		((cfg->format & I2S_FMT_BIT_CLK_INV) != 0U) ? I2S_CLKPOL_HIGH : I2S_CLKPOL_LOW;
	stream->i2s_prescaler = prescaler;

	stream->dma_cfg.source_data_size = data_size;
	stream->dma_cfg.dest_data_size = data_size;
	stream->dma_cfg.source_burst_length = data_size;
	stream->dma_cfg.dest_burst_length = data_size;

	memcpy(&stream->cfg, cfg, sizeof(stream->cfg));
	stream->state = I2S_STATE_READY;

	return 0;
}

static const struct i2s_config *i2s_n32_config_get(const struct device *dev, enum i2s_dir dir)
{
	struct i2s_n32_data *const data = dev->data;
	struct i2s_n32_stream *stream;

	if (dir == I2S_DIR_RX) {
		stream = &data->rx;
	} else if (dir == I2S_DIR_TX) {
		stream = &data->tx;
	} else {
		return NULL;
	}

	/*
	 * NULL means "not configured" and is turned into -EIO by
	 * i2s_common.c, which is how both i2s_buf_read() and i2s_buf_write()
	 * report access to a torn-down stream.
	 */
	if (stream->state == I2S_STATE_NOT_READY) {
		return NULL;
	}

	return &stream->cfg;
}

static int i2s_n32_trigger(const struct device *dev, enum i2s_dir dir,
			   enum i2s_trigger_cmd cmd)
{
	struct i2s_n32_data *const data = dev->data;
	struct i2s_n32_stream *stream;
	unsigned int key;
	int ret;

	switch (dir) {
	case I2S_DIR_RX:
		stream = &data->rx;
		break;
	case I2S_DIR_TX:
		stream = &data->tx;
		break;
	case I2S_DIR_BOTH:
		return -ENOSYS;
	default:
		LOG_DBG("either RX or TX direction must be selected");
		return -EINVAL;
	}

	switch (cmd) {
	case I2S_TRIGGER_START:
		if (stream->state != I2S_STATE_READY) {
			LOG_DBG("START trigger: invalid state %d", stream->state);
			return -EIO;
		}

		ret = stream_start(stream);
		if (ret < 0) {
			LOG_DBG("START trigger failed: %d", ret);
			return ret;
		}

		stream->state = I2S_STATE_RUNNING;
		stream->stop_for_drain = false;
		break;

	case I2S_TRIGGER_STOP:
		key = irq_lock();
		if (stream->state != I2S_STATE_RUNNING) {
			irq_unlock(key);
			LOG_DBG("STOP trigger: invalid state %d", stream->state);
			return -EIO;
		}
		stream_stop(stream, false);
		irq_unlock(key);
		break;

	case I2S_TRIGGER_DRAIN:
		key = irq_lock();
		if (stream->state != I2S_STATE_RUNNING) {
			irq_unlock(key);
			LOG_DBG("DRAIN trigger: invalid state %d", stream->state);
			return -EIO;
		}

		if (stream->is_tx) {
			/*
			 * "Send everything in the queue, then stop." There is
			 * still something to send if a transfer is in flight,
			 * or if the queue is not empty yet.
			 */
			stream_stop(stream, stream->mem_block != NULL ||
						      !queue_is_empty(stream->msgq));
		} else {
			/* For RX, DRAIN is defined to be the same as STOP. */
			stream_stop(stream, false);
		}
		irq_unlock(key);
		break;

	case I2S_TRIGGER_DROP:
		if (stream->state == I2S_STATE_NOT_READY) {
			LOG_DBG("DROP trigger: invalid state");
			return -EIO;
		}

		stream_disable(stream);
		stream_queue_drop(stream);
		stream->state = I2S_STATE_READY;
		break;

	case I2S_TRIGGER_PREPARE:
		if (stream->state != I2S_STATE_ERROR) {
			LOG_DBG("PREPARE trigger: invalid state %d", stream->state);
			return -EIO;
		}

		/*
		 * The error path already returned the block in flight and
		 * stopped the DMA, so all that is left is to give back the
		 * blocks that arrived before the error. After this the slab
		 * is completely free.
		 */
		stream_queue_drop(stream);
		stream->state = I2S_STATE_READY;
		break;

	default:
		LOG_DBG("unsupported trigger command %d", cmd);
		return -EINVAL;
	}

	return 0;
}

static int i2s_n32_read(const struct device *dev, void **mem_block, size_t *size)
{
	struct i2s_n32_data *const data = dev->data;
	struct i2s_n32_stream *stream = &data->rx;
	int ret;

	if (stream->state == I2S_STATE_NOT_READY) {
		LOG_DBG("RX stream is not configured");
		return -EIO;
	}

	/*
	 * In ERROR state the blocks that arrived before the error are still
	 * handed out -- the overrun happened after they were queued. Only
	 * once the queue is empty is the error reported, and it has to be
	 * reported as -EIO rather than left to the timeout: an -EAGAIN
	 * would be indistinguishable from the queue being momentarily
	 * empty, and this is how an application notices the overrun.
	 */
	if (stream->state == I2S_STATE_ERROR && queue_is_empty(stream->msgq)) {
		return -EIO;
	}

	/* Get data from the beginning of RX queue */
	ret = queue_get(stream->msgq, mem_block, size, stream->cfg.timeout);
	if (ret < 0) {
		LOG_DBG("queue_get() failed: ret=%d, used=%u/%u", ret,
			k_msgq_num_used_get(stream->msgq), CONFIG_I2S_N32_RX_BLOCK_COUNT);
	}

	return ret;
}

static int i2s_n32_write(const struct device *dev, void *mem_block, size_t size)
{
	struct i2s_n32_data *const data = dev->data;
	struct i2s_n32_stream *stream = &data->tx;

	/*
	 * A block may only be queued while the stream can consume it. In
	 * particular ERROR has to be refused: that error is exactly what the
	 * application is being told about by the failed write.
	 */
	if (stream->state != I2S_STATE_READY && stream->state != I2S_STATE_RUNNING) {
		LOG_DBG("TX stream is not accepting data (state %d)", stream->state);
		return -EIO;
	}

	/* Add data to the end of the TX queue */
	return queue_put(stream->msgq, mem_block, size, stream->cfg.timeout);
}

static DEVICE_API(i2s, i2s_n32_driver_api) = {
	.configure = i2s_n32_configure,
	.config_get = i2s_n32_config_get,
	.read = i2s_n32_read,
	.write = i2s_n32_write,
	.trigger = i2s_n32_trigger,
};

/*
 * The source clock is SYSCLK, which is fixed by the SoC startup code, and not
 * the PCLK1 that clock_control_get_rate() reports for an APB1 peripheral: the
 * I2S block has no clock-source mux and the vendor HAL divides SYSCLK.
 */
#define I2S_N32_SRC_CLK_HZ DT_PROP(DT_NODELABEL(rcc), clock_frequency)

BUILD_ASSERT(I2S_N32_SRC_CLK_HZ > 0,
	     "the rcc node needs a clock-frequency property: the I2S frame clock "
	     "prescaler is derived from SYSCLK");

/*
 * Release PA15, PB3 and PB4 from the JTAG debug port, as the vendor examples
 * do with GPIO_ConfigPinRemap(GPIO_RMP_SW_JTAG_SW_ENABLE, ENABLE).
 *
 * This is not part of the pinctrl state because SWJ_CFG is not a pin mux but a
 * debug-port control, and drivers/pinctrl/pinctrl_n32.c deliberately never
 * writes it -- it forces the field to 111 ("no effect") on every RMP_CFG
 * read-modify-write, which leaves a value written here untouched. A board that
 * does not ask for this keeps the reset default, so nothing changes for it.
 */
static int i2s_n32_jtag_port_release(void)
{
	uint32_t clkid = N32_CLOCK_AFIO;
	int ret;

	ret = clock_control_on(DEVICE_DT_GET(DT_NODELABEL(rcc)), (clock_control_subsys_t)&clkid);
	if (ret < 0) {
		LOG_ERR("failed to enable the AFIO clock: %d", ret);
		return ret;
	}

	AFIO->RMP_CFG = (AFIO->RMP_CFG & ~I2S_N32_SWJ_CFG_MASK) | I2S_N32_SWJ_CFG_SWD_ONLY;

	return 0;
}

static int i2s_n32_init(const struct device *dev)
{
	const struct i2s_n32_config *const cfg = dev->config;
	struct i2s_n32_data *const data = dev->data;
	int ret;

	/*
	 * The prescaler divides SYSCLK, and SYSCLK is whatever the SoC startup
	 * code settled on -- it is not reflected in the clock_control API, so
	 * the only number the driver has is the clock-frequency the devicetree
	 * declares. That number is right only if the PLL came up: SetSysClock()
	 * falls back to the 8 MHz HSI without reporting anything and leaves
	 * SystemCoreClock claiming the full rate, so a failed HSE would show up
	 * as a frame clock 18 times too slow rather than as an error. Check the
	 * source here, before anything is programmed, and refuse to init.
	 */
	if (RCC_GetSysclkSrc() != I2S_N32_SYSCLK_PLL) {
		LOG_ERR("SYSCLK is not the PLL (SCLKSTS = 0x%02x), so the %u Hz "
			"declared for the rcc node cannot be trusted",
			RCC_GetSysclkSrc(), (uint32_t)I2S_N32_SRC_CLK_HZ);
		return -EIO;
	}

	ret = clock_control_on(DEVICE_DT_GET(DT_NODELABEL(rcc)),
			       (clock_control_subsys_t)&cfg->apb_clk_en);
	if (ret < 0) {
		LOG_ERR("failed to enable the I2S clock: %d", ret);
		return ret;
	}

	/*
	 * Set here rather than in the static initializer: the callback needs a
	 * pointer to the stream it belongs to, which is a member of the very
	 * object being initialized.
	 */
	data->rx.dma_cfg.user_data = &data->rx;
	data->tx.dma_cfg.user_data = &data->tx;

	if (!device_is_ready(data->rx.dev_dma) || !device_is_ready(data->tx.dev_dma)) {
		LOG_ERR("DMA device not ready");
		return -ENODEV;
	}

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("I2S pinctrl setup failed: %d", ret);
		return ret;
	}

	if (cfg->jtag_release) {
		ret = i2s_n32_jtag_port_release();
		if (ret < 0) {
			return ret;
		}
	}

	LOG_DBG("%s inited", dev->name);

	return 0;
}

/*
 * Expands to the brace initializer only, so the caller supplies the ".rx ="
 * or ".tx =". The message queue name is passed in already formed rather than
 * built from @p dir, because a "##" paste does not expand its operands:
 * "&dir##_##index##_queue" would literally name a queue dir_<index>_queue. The
 * DT_INST_DMAS_* helpers are fine with a parameter standing in for the name,
 * since DT_PROP_HAS_NAME()'s two-level DT_CAT6() expands it first.
 *
 * block_count is 1 because dma_n32_config() rejects chained blocks, so a
 * stream is always a single block re-armed from its completion callback.
 *
 * dma_slot 0 keeps the channel on its hardwired request mapping, which is what
 * the vendor examples use: I2S2 on DMA1 channel 4/5 and I2S3 on DMA2 channel
 * 1/2. A non-zero dma_slot would turn on CHMAPEN for the whole controller and
 * change how every other channel of it is decoded. The data sizes are per
 * configuration and are set by configure(); user_data is set by i2s_n32_init().
 */
#define I2S_N32_STREAM_INIT(index, dir, q_name, req, dma_dir, tx, cb)                           \
	{                                                                                      \
		.dev_dma = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(index, dir)),               \
		.regs = (SPI_Module *)DT_INST_REG_ADDR(index),                                 \
		.msgq = &q_name,                                                               \
		.dma_channel = DT_INST_DMAS_CELL_BY_NAME(index, dir, channel),                 \
		.dma_req = req,                                                                \
		.is_tx = tx,                                                                   \
		.dma_cfg = {                                                                   \
			.block_count = 1,                                                      \
			.dma_slot = DT_INST_DMAS_CELL_BY_NAME(index, dir, request),            \
			.channel_direction = dma_dir,                                          \
			.dma_callback = cb,                                                    \
			.channel_priority = N32_DMA_CH_CFG_PRIORITY_GET(                      \
				DT_INST_DMAS_CELL_BY_NAME(index, dir, config)),                \
		},                                                                             \
	}

#define I2S_N32_INIT(index)                                                                    \
	K_MSGQ_DEFINE_STATIC_TYPE(rx_##index##_queue, struct i2s_n32_queue_item,               \
				  CONFIG_I2S_N32_RX_BLOCK_COUNT);                              \
	K_MSGQ_DEFINE_STATIC_TYPE(tx_##index##_queue, struct i2s_n32_queue_item,               \
				  CONFIG_I2S_N32_TX_BLOCK_COUNT);                              \
                                                                                               \
	PINCTRL_DT_INST_DEFINE(index);                                                         \
                                                                                               \
	static const struct i2s_n32_config i2s_n32_config_##index = {                          \
		.regs = (SPI_Module *)DT_INST_REG_ADDR(index),                                 \
		.apb_clk_en = DT_INST_CLOCKS_CELL(index, bits),                                \
		.src_clk_hz = I2S_N32_SRC_CLK_HZ,                                              \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index),                                 \
		.jtag_release = DT_INST_PROP(index, nsing_jtag_port_release),                  \
	};                                                                                     \
                                                                                               \
	static struct i2s_n32_data i2s_n32_data_##index = {                                    \
		.rx = I2S_N32_STREAM_INIT(index, rx, rx_##index##_queue, SPI_I2S_DMA_RX,       \
					  PERIPHERAL_TO_MEMORY, false, dma_rx_callback),       \
		.tx = I2S_N32_STREAM_INIT(index, tx, tx_##index##_queue, SPI_I2S_DMA_TX,       \
					  MEMORY_TO_PERIPHERAL, true, dma_tx_callback),        \
	};                                                                                     \
                                                                                               \
	DEVICE_DT_INST_DEFINE(index, i2s_n32_init, NULL, &i2s_n32_data_##index,                \
			      &i2s_n32_config_##index, POST_KERNEL, CONFIG_I2S_INIT_PRIORITY,  \
			      &i2s_n32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(I2S_N32_INIT)
