/*
 * Copyright (c) 2024 Nations Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * MSPI (QSPI) driver for the Nations N32G45x.
 *
 * The block is a Synopsys DesignWare SSI with an XIP extension bolted on. Its
 * register map lines up field for field with the DesignWare one, so the
 * transfer engine below follows the device-independent sequence rather than the
 * vendor HAL's. Three places where the two disagree, and why this driver
 * follows the DesignWare one:
 *
 * 1. CTRL1.NDF counts frames minus one; upstream's mspi_dw.c writes
 *    FIELD_PREP(CTRLR1_NDF_MASK, data_frames - 1). The vendor HAL writes the
 *    raw frame count, which works only because every one of its entry points
 *    opens with QSPI_DeInit() -- a peripheral reset used as a FIFO flush -- so
 *    the extra frame is thrown away before anyone can see it. This driver
 *    writes frames - 1 and never resets the block.
 *
 * 2. CE is not a pin state, it is SLAVE_EN written per transaction. The vendor
 *    QSPI_Cmd() sets SLAVE_EN and EN together once at init and never clears
 *    them, which is only correct on a bus with a single device on it. Here SER
 *    is written for every packet and cleared at the end, the way mspi_dw.c does
 *    it (mspi_dw.c:1441), so mspi_dev_id::dev_idx really does select a device.
 *
 * 3. In Standard SPI mode the core cannot send the instruction and address
 *    phases on its own -- there are no INST_L/ADDR_LEN fields for them -- so an
 *    RX packet runs in TX_AND_RX mode and pushes a frame for every frame it
 *    wants to receive. Upstream says the same at mspi_dw.c:1216. The vendor
 *    example reads the flash status register the same way: it sends {0x05,
 *    0x00} and takes the *second* word back.
 *
 * 4. An enhanced transmit runs without clock stretching; an enhanced receive
 *    runs with it. The vendor sets CLK_STRETCH_EN on both of its quad configs,
 *    one bit for the quad format as a whole, so it says nothing about a
 *    transmit. Here it was tried on one, and a 258 frame page program wrote
 *    nothing at all -- the page came back as the ff the erase left -- while a
 *    transfer short enough to come out of the FIFO in one go stayed byte for
 *    byte correct. Whatever the core does while it is stretched, it is not a
 *    transfer an SPI flash can follow: their page program is a different shape
 *    rather than a different block, with the device selected before the first
 *    word and every word after it paced by the same stretched clock. With the
 *    bit off the core retires an enhanced transmit when its FIFO drains, which
 *    is the completion this driver waits for, and is also why a transmit longer
 *    than the FIFO has to be handed to DMA. Receives keep it: that is what holds
 *    the bus over the wait cycles.
 *
 * Register access is restricted the way it is on any DesignWare SSI: CTRL0,
 * CTRL1, ENH_CTRL0 and BAUD are only writable while EN is 0, so every packet
 * disables the controller, programs it, and enables it again. EN and SLAVE_EN
 * are separate registers in this SoC's header but one feature -- EN gates the
 * block, SLAVE_EN drives CE.
 *
 * Register access is also slow, and that is what shapes the transmit side: a
 * push into the FIFO costs this block about 550 ns at the 144 MHz HCLK the
 * board runs, against the 333 ns the core takes to shift a frame in an 8-8-8
 * data phase and the 83 ns it takes in a four line one. The CPU therefore feeds
 * a packet only as long as the whole of it fits in the FIFO, and hands anything
 * longer to the controller's transmit DMA channel. See the feed decision in
 * n32_qspi_run_packet().
 *
 * A frame comes out of the data register only when a word is written to it: with
 * the block enabled, a 32 bit store moves the FIFO level up by one and byte and
 * halfword stores of the same value leave it alone. The channel therefore feeds
 * one frame per memory word, out of a buffer built for it, rather than reading
 * the caller's bytes directly -- see n32_qspi_dma_tx().
 *
 * What is deliberately missing, and why:
 *
 *  - XIP / memory map. The block has XIP_CTRL and XIP_MODE but no XIP write
 *    register file, so there is nothing to program for writes and no way to
 *    test reads from here. memmap_config is left out, so mspi_memmap_config()
 *    answers -ENOTSUP and flash_mspi_nor.c never touches the window.
 *  - Asynchronous transfers. CONFIG_MSPI_ASYNC is off by default and
 *    flash_mspi_nor.c never sets mspi_xfer::async, so a polled synchronous
 *    engine is all anything in tree asks for. The DMA above is driven from
 *    inside that engine -- configured, started, waited for -- rather than
 *    through the async side of the API.
 *  - Receive DMA. A receive is paced by the frames the CPU pushes to clock it,
 *    so the core cannot outrun the CPU there, and the receive FIFO is drained a
 *    frame per pass as it fills. It has never been the side that limits a
 *    transfer, and a second channel would only be a second thing to get wrong.
 *  - hold_ce. Keeping CE asserted between packets means leaving the block
 *    enabled with a device selected while control returns to the caller, which
 *    is a state this driver does not have a way to make safe. A transfer that
 *    asks for it is rejected instead of quietly dropping CE.
 */

#define DT_DRV_COMPAT nsing_n32_qspi

/*
 * Whether the controller has a transmit DMA channel to fall back on, which is
 * what makes a transmit longer than the FIFO possible at all. The whole driver
 * is written for the one QSPI controller the SoC has, so the instance index
 * here is not a shortcut -- there is nothing to loop over.
 */
#define N32_QSPI_HAS_TX_DMA	DT_INST_DMAS_HAS_NAME(0, tx)

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/n32_clock_control.h>
#include <zephyr/drivers/mspi.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/clock/n32_clock.h>
#include <zephyr/dt-bindings/dma/n32_dma.h>
#if N32_QSPI_HAS_TX_DMA
#include <zephyr/drivers/dma.h>
#endif
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/time_units.h>
#include <zephyr/sys/util.h>

#include <n32g45x.h>

LOG_MODULE_REGISTER(mspi_n32, CONFIG_MSPI_LOG_LEVEL);

/*
 * Every frame this driver puts on the bus is 8 bits wide, which is what the
 * vendor examples use and what SPI NOR needs. DFS counts bits minus one; CFS is
 * only consulted for the instruction and address phases in enhanced mode and is
 * set to the same width so the two agree.
 */
#define N32_QSPI_FRAME_BITS	8U

/*
 * Field helpers. The SoC header carries one macro per field value, which does
 * not compose, so the shifts are spelled out here on top of the field masks.
 */
#define N32_QSPI_CTRL0_DFS(bits)	FIELD_PREP(QSPI_CTRL0_DFS, (bits) - 1U)
#define N32_QSPI_CTRL0_CFS(bits)	FIELD_PREP(QSPI_CTRL0_CFS, (bits) - 1U)

/*
 * INST_L is a code, not a length: 0 means no instruction phase, 1 to 3 mean 4, 8
 * and 16 bits. Only the widths SPI NOR uses are mapped; a command length
 * outside them is rejected when the device is configured.
 */
#define N32_QSPI_ENH_INST_L(cmd_length)						\
	FIELD_PREP(QSPI_ENH_CTRL0_INST_L,					\
		   (cmd_length) == 0U ? 0U					\
		   : (cmd_length) == 1U ? 2U /* 8 bits */			\
		   : 3U /* 16 bits */)

/* ADDR_LEN counts quads of bits: a 24-bit address is programmed as 6. */
#define N32_QSPI_ENH_ADDR_LEN(addr_length)					\
	FIELD_PREP(QSPI_ENH_CTRL0_ADDR_LEN, (addr_length) * 2U)

#define N32_QSPI_ENH_WAIT(cycles)						\
	FIELD_PREP(QSPI_ENH_CTRL0_WAIT_CYCLES, cycles)

/* The wait field is five bits, so the longest wait is 31 clock cycles. */
#define N32_QSPI_WAIT_MAX	31U

/*
 * The baud divider must be even and at least 2, and it is a straight division:
 * f_sck = f_qspi / BAUD. Upstream divides the same way (mspi_dw.c:944), and the
 * vendor example's CLOCK_DIVIDER of 6 against a 144 MHz AHB clock gives the
 * 24 MHz it documents.
 */
#define N32_QSPI_BAUD_MIN	2U

/* Frame counts live in a 16-bit field and are written as count - 1. */
#define N32_QSPI_FRAMES_MAX	(UINT16_MAX + 1U)

/* One SER bit per device, so a device index has to fit in a word. */
#define N32_QSPI_DEV_IDX_MAX	31U

/*
 * How many frames the transmit FIFO holds. Not in the datasheet: a transfer
 * that stops part way through reports the level it stopped at, and both
 * directions have been measured at 32 on this part. It matters because it is
 * what decides whether a transmit can be fed from the CPU at all, and how much
 * of it the CPU can have in flight before the DMA has to take over -- see the
 * feed decision in n32_qspi_run_packet().
 */
#define N32_QSPI_FIFO_DEPTH	32U

/*
 * The level the transmit DMA request is asserted at, as a fraction of the FIFO
 * depth: the channel is asked for more once the level falls to this, and is left
 * alone above it. Half leaves the core several frames in hand while the channel
 * refills, which at 24 MHz SCK is over a microsecond of slack for a channel that
 * only has to keep up with one write per frame. The vendor programs 1, which
 * works but leaves the FIFO all but empty and leans on the channel's latency
 * being shorter than a single frame. A threshold above the depth is worse than
 * either: the request would never be taken down, and the channel would be
 * writing into a FIFO that is already full, where the writes are dropped rather
 * than queued and frames go missing.
 */
#define N32_QSPI_DMA_TX_LEVEL	(N32_QSPI_FIFO_DEPTH / 2U)

/*
 * How much memory one frame takes on the transmit channel: a word, because that
 * is the only width the data register accepts a frame from. See
 * n32_qspi_dma_tx().
 */
#define N32_QSPI_DMA_FRAME_BYTES	sizeof(uint32_t)

/*
 * How many payload frames one receive packet carries. A longer read is run as
 * several transactions, one after another, each re-sending the instruction and
 * address for the next stretch of it.
 *
 * A read longer than this cannot be done in one transaction at a clock the CPU
 * cannot match. Standard SPI clocks a frame out of the flash for every frame
 * pushed into the transmit FIFO, and this block ends a transfer whose FIFO has
 * run dry: the flash is left mid-read, the frames the driver goes on to take out
 * of the receive FIFO are the leftovers of a transaction the core has already
 * abandoned, and the caller gets a buffer with the FIFO's worth of true bytes
 * and zeroes behind them, with no count and no error flag to say so. That is
 * what was measured on this board -- a 256 byte read and a 128 byte read of the
 * same page stopped at the same byte, both while 16 byte reads of that byte were
 * correct, and the same two reads were correct at 6 MHz. An enhanced receive has
 * the same shape from the other side: the core clocks those frames in by itself,
 * so a packet longer than the FIFO overflows it before the driver can drain it,
 * and the frames in flight when the block's clock stretching takes hold are lost
 * for good.
 *
 * The CPU's own rate is what sets the size. One frame pushed costs this block's
 * registers about 550 ns at the 144 MHz HCLK this board runs, against 333 ns a
 * frame an 8-8-8 data phase shifts and 83 ns a four line one. A packet inside the
 * FIFO is therefore pushed in full before CE goes down and needs nothing pushed
 * while the core shifts, which is the one arrangement the CPU keeps up with
 * whatever the clock. The reserve is for the instruction and address phases,
 * which are frames too, and for the frames still in flight while the driver
 * drains.
 */
#define N32_QSPI_RX_CHUNK_FRAMES	(N32_QSPI_FIFO_DEPTH - 8U)

#define N32_QSPI_DEFAULT_TIMEOUT_MS	CONFIG_MSPI_COMPLETION_TIMEOUT_TOLERANCE

struct mspi_n32_dev_cfg {
	enum mspi_io_mode io_mode;
	enum mspi_cpp_mode cpp;
	uint32_t baud;
};

struct mspi_n32_config {
	QSPI_Module *regs;
	struct mspi_cfg mspicfg;
	/* Clock gate, in the encoding n32_clock.h uses: (offset << 6) | bit. */
	uint32_t clk_en;
	const struct pinctrl_dev_config *pcfg;
#if N32_QSPI_HAS_TX_DMA
	/*
	 * Transmit DMA only. The receive side has no channel: the CPU drains
	 * that FIFO as the controller fills it, one frame per pass, and it has
	 * never been the side that limits a transfer -- a receive is paced by
	 * the frames the CPU pushes to clock it, so the core cannot run ahead of
	 * either end of it.
	 */
	const struct device *dma;
	uint32_t dma_channel;
	uint32_t dma_slot;
	uint32_t dma_priority;
#endif
};

struct mspi_n32_data {
	/*
	 * Session state. mspi_dev_config() acquires the controller for a device
	 * and takes this lock; mspi_get_channel_status() releases both. See the
	 * notes on those entry points.
	 */
	const struct mspi_dev_id *dev_id;
	struct k_mutex lock;

	struct mspi_n32_dev_cfg dev_cfg;
	/* f_qspi, as reported by the clock control driver. */
	uint32_t clock_freq;
	/*
	 * Set while a packet is on the wire. The session lock is already held
	 * for the whole session, so this only rejects a second transfer
	 * started from another thread on the same session, which the API has
	 * no way to queue.
	 */
	volatile bool in_use;
#if N32_QSPI_HAS_TX_DMA
	/*
	 * What the transmit channel reads from: one frame per word, because the
	 * data register only takes a frame from a word wide write. Built by
	 * n32_qspi_dma_tx() out of the caller's buffer, which is one frame per
	 * byte. The size is what can be handed over in one block, and a packet
	 * that needs more is refused rather than fed in pieces.
	 */
	uint32_t tx_bounce[CONFIG_MSPI_N32_TX_DMA_FRAMES];
#endif
};

/*
 * One packet normalised into the frame counts the FIFO engine needs. Instruction
 * and address are counted as data frames in Standard SPI mode (one frame per
 * byte) and as single control words in enhanced mode, which is the difference
 * between the two engines.
 */
struct mspi_n32_plan {
	uint32_t ctrl0;
	uint32_t ctrl1;
	uint32_t enh_ctrl0;
	uint32_t cmd_frames;
	uint32_t addr_frames;
	uint32_t dummy_frames;
	uint32_t payload_frames;
	/* Frames to take out of the FIFO, and how many of those to drop first. */
	uint32_t rx_frames;
	uint32_t discard_frames;
	bool enhanced;
	bool payload_is_data;
};

static inline bool n32_qspi_elapsed(uint32_t start, uint32_t timeout_cyc)
{
	return (k_cycle_get_32() - start) >= timeout_cyc;
}

/*
 * RXFN is the RX FIFO level, which is what the vendor HAL polls for the same
 * purpose. The STS flag for it means the same thing here, but the level costs
 * nothing extra to read and cannot be misread.
 */
static inline bool n32_qspi_rx_ready(const struct mspi_n32_config *cfg)
{
	return cfg->regs->RXFN != 0U;
}

static inline bool n32_qspi_tx_ready(const struct mspi_n32_config *cfg)
{
	return (cfg->regs->STS & QSPI_STS_TXFNF) != 0U;
}

static inline bool n32_qspi_idle(const struct mspi_n32_config *cfg)
{
	return (cfg->regs->STS & QSPI_STS_BUSY) == 0U && cfg->regs->TXFN == 0U;
}

static inline void n32_qspi_push(const struct mspi_n32_config *cfg, uint32_t word)
{
	cfg->regs->DAT0 = word;
}

static inline uint32_t n32_qspi_pop(const struct mspi_n32_config *cfg)
{
	return cfg->regs->DAT0;
}

/* The word to push for one frame of a packet; defined with the packet runner. */
static uint32_t n32_qspi_tx_frame(const struct mspi_n32_plan *plan,
				  const struct mspi_xfer_packet *pkt, uint32_t index);

#if N32_QSPI_HAS_TX_DMA
/*
 * Build the words the transmit channel will hand over, and start it on them.
 *
 * One frame per memory word, not one frame per memory byte. This block takes a
 * frame from the data register only when the write to it is a word wide: with the
 * block enabled and no device selected, a 32 bit store moves TXFN up by one and
 * byte and halfword stores of the same value leave it where it was. That was
 * measured on the part rather than read anywhere, and it is the shape the vendor
 * HAL's transmit has as well -- QSPI_XIP's page program feeds its channel from a
 * uint32_t[258] holding one payload byte per word, and QSPI_DMA_Config() sets
 * both ends of the transfer to Word. A byte wide block, which is what an 8 bit
 * frame looks like in the caller's buffer, is a block that programs nothing while
 * the channel counts it down to zero and reports success.
 *
 * The frames are built with the same function the CPU feed uses, so a frame that
 * reaches the bus through the channel is the frame the FIFO would have carried,
 * instruction and address phases included.
 *
 * The whole handover is one block, deliberately. An enhanced transmit ends when
 * its FIFO runs dry, so a feed that paused between blocks would let the core
 * retire a packet with only part of its payload on the wire, and neither the
 * channel's count nor the retire would show it. A packet that needs more words
 * than the bounce buffer holds is refused here, before CE ever goes down.
 *
 * The channel is configured and started, not waited for: the caller has frames in
 * the FIFO already, and this is called before CE is asserted. It has also left one
 * slot free for the frame this channel hands over the moment it is enabled -- see
 * the feed decision in n32_qspi_run_packet().
 */
static int n32_qspi_dma_tx(const struct device *dev, const struct mspi_n32_plan *plan,
			   const struct mspi_xfer_packet *pkt, uint32_t first,
			   uint32_t frames)
{
	const struct mspi_n32_config *cfg = dev->config;
	struct mspi_n32_data *data = dev->data;
	struct dma_block_config blk = { 0 };
	struct dma_config dma_cfg = { 0 };
	uint32_t i;
	int ret;

	if (frames > ARRAY_SIZE(data->tx_bounce)) {
		LOG_ERR("a %u frame handover does not fit this driver's %u word"
			" bounce buffer; raise CONFIG_MSPI_N32_TX_DMA_FRAMES",
			frames, (uint32_t)ARRAY_SIZE(data->tx_bounce));
		return -ENOTSUP;
	}

	for (i = 0U; i < frames; i++) {
		data->tx_bounce[i] = n32_qspi_tx_frame(plan, pkt, first + i);
	}

	dma_cfg.channel_direction = MEMORY_TO_PERIPHERAL;
	dma_cfg.source_data_size = N32_QSPI_DMA_FRAME_BYTES;
	dma_cfg.dest_data_size = N32_QSPI_DMA_FRAME_BYTES;
	dma_cfg.dma_slot = cfg->dma_slot;
	dma_cfg.channel_priority = cfg->dma_priority;
	dma_cfg.block_count = 1U;
	dma_cfg.head_block = &blk;

	/*
	 * block_size is in bytes and the channel counts elements, so the two are
	 * a frame apart: one element is one frame, and the DMA driver divides by
	 * the data size to get the count it programs. Handing it a frame count
	 * there would ask for a block that is not a whole number of elements and
	 * it would be refused.
	 */
	blk.block_size = frames * N32_QSPI_DMA_FRAME_BYTES;
	blk.source_address = (uint32_t)(uintptr_t)data->tx_bounce;
	blk.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	blk.dest_address = (uint32_t)(uintptr_t)&cfg->regs->DAT0;
	blk.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;

	ret = dma_config(cfg->dma, cfg->dma_channel, &dma_cfg);
	if (ret < 0) {
		LOG_ERR("could not configure the transmit DMA channel: %d", ret);
		return ret;
	}

	ret = dma_start(cfg->dma, cfg->dma_channel);
	if (ret < 0) {
		LOG_ERR("could not start the transmit DMA channel: %d", ret);
	}

	return ret;
}

/*
 * Frames the channel has still to hand over; zero once it is done. The DMA API
 * counts what is left in bytes, and a frame is one word of the bounce buffer.
 */
static uint32_t n32_qspi_dma_left(const struct mspi_n32_config *cfg)
{
	struct dma_status stat;

	if (dma_get_status(cfg->dma, cfg->dma_channel, &stat) < 0) {
		return 0U;
	}

	return (uint32_t)stat.pending_length / (uint32_t)N32_QSPI_DMA_FRAME_BYTES;
}

/*
 * Stop the channel and clear both of its registers. This has to happen before
 * the block is disabled: a channel still running would go on writing frames
 * into a data register whose FIFO has just been flushed, and those frames would
 * then be at the head of the next packet.
 */
static void n32_qspi_dma_stop(const struct mspi_n32_config *cfg)
{
	(void)dma_stop(cfg->dma, cfg->dma_channel);

	cfg->regs->DMA_CTRL = 0U;
	cfg->regs->DMATDL_CTRL = 0U;
}
#endif /* N32_QSPI_HAS_TX_DMA */

/*
 * Map an MSPI I/O mode onto CTRL0.SPI_FRF and ENH_CTRL0.TRANS_TYPE. The postfix
 * names the number of lines used for the command, address and data phases, and
 * that is what TRANS_TYPE selects: 0 keeps instruction and address on one line,
 * 1 puts the address on the data lines too, 2 puts everything on them. A mode
 * with no postfix has the same width for all three phases, which is 2.
 *
 * Returns false for a mode the block cannot express.
 */
static bool n32_qspi_io_mode(enum mspi_io_mode io_mode, uint32_t *frf,
			     uint32_t *trans_type)
{
	switch (io_mode) {
	case MSPI_IO_MODE_SINGLE:
		*frf = QSPI_CTRL0_SPI_FRF_STANDARD_FORMAT;
		*trans_type = QSPI_ENH_CTRL0_TRANS_TYPE_STANDARD;
		return true;
	case MSPI_IO_MODE_DUAL:
		*frf = QSPI_CTRL0_SPI_FRF_DUAL_FORMAT;
		*trans_type = QSPI_ENH_CTRL0_TRANS_TYPE_ALL_BY_FRF;
		return true;
	case MSPI_IO_MODE_DUAL_1_1_2:
		*frf = QSPI_CTRL0_SPI_FRF_DUAL_FORMAT;
		*trans_type = QSPI_ENH_CTRL0_TRANS_TYPE_STANDARD;
		return true;
	case MSPI_IO_MODE_DUAL_1_2_2:
		*frf = QSPI_CTRL0_SPI_FRF_DUAL_FORMAT;
		*trans_type = QSPI_ENH_CTRL0_TRANS_TYPE_ADDRESS_BY_FRF;
		return true;
	case MSPI_IO_MODE_QUAD:
		*frf = QSPI_CTRL0_SPI_FRF_QUAD_FORMAT;
		*trans_type = QSPI_ENH_CTRL0_TRANS_TYPE_ALL_BY_FRF;
		return true;
	case MSPI_IO_MODE_QUAD_1_1_4:
		*frf = QSPI_CTRL0_SPI_FRF_QUAD_FORMAT;
		*trans_type = QSPI_ENH_CTRL0_TRANS_TYPE_STANDARD;
		return true;
	case MSPI_IO_MODE_QUAD_1_4_4:
		*frf = QSPI_CTRL0_SPI_FRF_QUAD_FORMAT;
		*trans_type = QSPI_ENH_CTRL0_TRANS_TYPE_ADDRESS_BY_FRF;
		return true;
	default:
		return false;
	}
}

/*
 * The baud divider for a requested clock: rounded down to an even number, as the
 * core requires, and floored at 2 so that no device can ask for more than half
 * of f_qspi.
 */
static uint32_t n32_qspi_baud(uint32_t clock_freq, uint32_t freq)
{
	uint32_t divider;

	if (freq == 0U || freq > clock_freq / N32_QSPI_BAUD_MIN) {
		divider = N32_QSPI_BAUD_MIN;
	} else {
		divider = clock_freq / freq;
	}

	return MAX(divider & ~1U, N32_QSPI_BAUD_MIN);
}

/*
 * Work out everything a packet needs programmed, without touching the block.
 */
static int n32_qspi_plan(const struct mspi_n32_dev_cfg *dev_cfg,
			 const struct mspi_xfer *xfer,
			 const struct mspi_xfer_packet *pkt,
			 struct mspi_n32_plan *plan)
{
	uint32_t frf = 0U;
	uint32_t trans_type = 0U;
	bool rx = (pkt->dir == MSPI_RX) && (pkt->num_bytes > 0U);

	if (xfer->cmd_length > 2U) {
		LOG_ERR("command length %u is not supported", xfer->cmd_length);
		return -ENOTSUP;
	}

	if (xfer->addr_length > 4U) {
		LOG_ERR("address length %u is not supported", xfer->addr_length);
		return -ENOTSUP;
	}

	if (xfer->rx_dummy > N32_QSPI_WAIT_MAX) {
		LOG_ERR("rx dummy cycles %u exceed the %u the block can wait",
			xfer->rx_dummy, N32_QSPI_WAIT_MAX);
		return -ENOTSUP;
	}

	if (pkt->num_bytes == 0U && xfer->cmd_length == 0U && xfer->addr_length == 0U) {
		/* Nothing to put on the bus, and NDF would underflow to 0xffff. */
		LOG_ERR("packet carries no frames");
		return -EINVAL;
	}

	if (!n32_qspi_io_mode(dev_cfg->io_mode, &frf, &trans_type)) {
		LOG_ERR("I/O mode %d is not supported", dev_cfg->io_mode);
		return -ENOTSUP;
	}

	*plan = (struct mspi_n32_plan){
		.enhanced = dev_cfg->io_mode != MSPI_IO_MODE_SINGLE,
		.rx_frames = rx ? pkt->num_bytes : 0U,
		.payload_is_data = !rx,
	};

	plan->ctrl0 = N32_QSPI_CTRL0_DFS(N32_QSPI_FRAME_BITS) |
		      N32_QSPI_CTRL0_CFS(N32_QSPI_FRAME_BITS) | frf;

	if ((dev_cfg->cpp & 0x2U) != 0U) {
		plan->ctrl0 |= QSPI_CTRL0_SCPOL_HIGH;
	}

	if ((dev_cfg->cpp & 0x1U) != 0U) {
		plan->ctrl0 |= QSPI_CTRL0_SCPH_SECOND_EDGE;
	}

	plan->enh_ctrl0 = trans_type |
			  N32_QSPI_ENH_ADDR_LEN(xfer->addr_length) |
			  N32_QSPI_ENH_INST_L(xfer->cmd_length) |
			  N32_QSPI_ENH_WAIT(MIN(xfer->rx_dummy, N32_QSPI_WAIT_MAX));

	if (plan->enhanced) {
		/*
		 * Instruction and address each occupy one FIFO entry, whatever
		 * their length, and the payload follows.
		 */
		plan->cmd_frames = xfer->cmd_length != 0U ? 1U : 0U;
		plan->addr_frames = xfer->addr_length != 0U ? 1U : 0U;
		plan->dummy_frames = 0U;

		if (rx) {
			/*
			 * Clock stretching is what holds the bus through the wait
			 * cycles between the address and the first data frame. The
			 * vendor sets it for its quad format as a whole, both
			 * directions, so the bit is not evidence that a transmit
			 * wants it.
			 *
			 * RX_ONLY: the core puts the control frames on the bus
			 * itself, waits, and clocks exactly NDF + 1 frames, so
			 * no payload goes into the FIFO.
			 */
			plan->enh_ctrl0 |= QSPI_ENH_CTRL0_CLK_STRETCH_EN;
			plan->ctrl0 |= QSPI_CTRL0_TMOD_RX_ONLY;
			plan->ctrl1 = pkt->num_bytes - 1U;
			plan->payload_frames = 0U;
			return 0;
		}

		/*
		 * TX_ONLY, and clock stretching stays off.
		 *
		 * It is tempting here, and it was tried: the vendor's quad page
		 * program has the bit set, and stretching is the block's own
		 * answer to a FIFO the CPU feeds more slowly than the core shifts
		 * it. What the core does while it is stretched is not a transfer
		 * an SPI flash can follow, though -- a 258 frame page program with
		 * the bit set wrote nothing at all, the page coming back as the ff
		 * the erase left, while at the same speed a transfer short enough
		 * to come out of the FIFO in one go stayed byte for byte correct.
		 * Their transmit is a different shape rather than a different
		 * block: the device is selected before the first word is pushed
		 * and every word after it is paced by the same stretched clock, so
		 * what their flash sees is a slower clock and not a stopped one.
		 *
		 * So the core ends the transfer on a FIFO that has run dry, and
		 * that is the completion signal the wait below looks for. It also
		 * sets the limit on what this path can send: the frames in the FIFO
		 * when CE goes down, plus what the feed can put behind them. See
		 * the feed decision in n32_qspi_run_packet() for what happens when
		 * that is not enough.
		 *
		 * CTRL1 is left at zero, as the vendor leaves it: NDF is a receive
		 * count on this block, and a transmit does not stop at it. A
		 * transmit ends when the core runs out of frames to shift, which is
		 * the same thing the wait below waits on, and why the feed has to
		 * stay ahead of the core from CE to the last frame.
		 */
		plan->ctrl0 |= QSPI_CTRL0_TMOD_TX_ONLY;
		plan->ctrl1 = 0U;
		plan->payload_frames = pkt->num_bytes;
		return 0;
	}

	/*
	 * Standard SPI mode. Instruction, address and any dummy cycles are
	 * ordinary data frames, so an RX packet runs in TX_AND_RX and pushes a
	 * frame for every frame it wants clocked back; CTRL1 is ignored there
	 * as well. The vendor example depends on this: its
	 * QspiSendAndGetWords() pushes both words of {0x05, 0x00} and then
	 * waits for the RX FIFO level to reach 2.
	 */
	plan->cmd_frames = xfer->cmd_length;
	plan->addr_frames = xfer->addr_length;
	plan->dummy_frames = xfer->rx_dummy / N32_QSPI_FRAME_BITS;
	plan->payload_frames = pkt->num_bytes;
	plan->discard_frames = plan->cmd_frames + plan->addr_frames + plan->dummy_frames;

	plan->ctrl0 |= rx ? QSPI_CTRL0_TMOD_TX_AND_RX : QSPI_CTRL0_TMOD_TX_ONLY;

	if (rx) {
		/*
		 * The payload frames are padding, not data, and the frames the
		 * instruction and address phases shift back arrive ahead of the
		 * payload, so both have to be taken out of the FIFO and thrown
		 * away. Unlike enhanced mode, the core has no way to skip them.
		 */
		plan->payload_is_data = false;
		plan->rx_frames += plan->discard_frames;
	}

	plan->ctrl1 = (plan->cmd_frames + plan->addr_frames + plan->dummy_frames +
		       plan->payload_frames) - 1U;

	return 0;
}

/*
 * The word to push for frame number `index` of a packet, instruction first.
 * Standard SPI sends the instruction and address most significant byte first,
 * one frame per byte, the way mspi_dw.c's tx_control_field() does; enhanced mode
 * sends each as a single control word that the core breaks up itself.
 */
static uint32_t n32_qspi_tx_frame(const struct mspi_n32_plan *plan,
				  const struct mspi_xfer_packet *pkt, uint32_t index)
{
	if (index < plan->cmd_frames) {
		if (plan->enhanced) {
			return pkt->cmd;
		}

		return (pkt->cmd >> (8U * (plan->cmd_frames - 1U - index))) & 0xFFU;
	}

	index -= plan->cmd_frames;

	if (index < plan->addr_frames) {
		if (plan->enhanced) {
			return pkt->address;
		}

		return (pkt->address >> (8U * (plan->addr_frames - 1U - index))) & 0xFFU;
	}

	index -= plan->addr_frames;

	if (index < plan->dummy_frames) {
		return 0U;
	}

	index -= plan->dummy_frames;

	if (!plan->payload_is_data || index >= pkt->num_bytes) {
		/*
		 * Either the padding an RX packet needs to clock data out of
		 * the flash, or a frame in a plan that has no payload at all.
		 */
		return 0U;
	}

	return pkt->data_buf[index];
}

/*
 * Run one packet. The controller is left disabled on the way out; CE is
 * asserted for exactly as long as the packet is on the wire.
 */
static int n32_qspi_run_packet(const struct device *dev, const struct mspi_xfer *xfer,
			       const struct mspi_xfer_packet *pkt)
{
	const struct mspi_n32_config *cfg = dev->config;
	const struct mspi_n32_data *data = dev->data;
	struct mspi_n32_plan plan;
	uint32_t timeout_cyc;
	uint32_t start;
	uint32_t tx_frames;
	/* How many frames of the packet the CPU feeds itself. */
	uint32_t pio_frames;
	uint32_t next_tx = 0U;
	uint32_t prefilled;
	uint32_t popped = 0U;
	uint32_t written = 0U;
#if N32_QSPI_HAS_TX_DMA
	bool tx_dma = false;
#endif
	int ret;

	ret = n32_qspi_plan(&data->dev_cfg, xfer, pkt, &plan);
	if (ret < 0) {
		return ret;
	}

	tx_frames = plan.cmd_frames + plan.addr_frames + plan.dummy_frames +
		    plan.payload_frames;

	/*
	 * A transmit is fed from one of two places, and which one is decided
	 * here rather than discovered part way through.
	 *
	 * A packet that only shifts frames out and is longer than the FIFO
	 * cannot be fed from the CPU. The core takes a frame every 333 ns in an
	 * 8-8-8 data phase and every 83 ns in a four line one, and pushing one
	 * costs this block's registers about 550 ns at the 144 MHz HCLK this
	 * board runs -- some eighty AHB cycles, which is what a QSPI register
	 * access costs here. Even a loop with nothing in it but the push would
	 * come out short, and a transmit that runs out of frames is a transmit
	 * the core abandons where it stands: a 256 byte page program fed this way
	 * left the flash with the first 110 bytes written and the rest as the
	 * erase left it, with no count and no error flag anywhere reporting it.
	 * An 8-8-8 receive has the same limit from the same place, and is kept
	 * inside it by length rather than by a channel -- see
	 * N32_QSPI_RX_CHUNK_FRAMES.
	 *
	 * Everything else stays with the CPU: a receive that is clocked by the
	 * frames the CPU pushes is paced by them, and the receive FIFO is drained
	 * a frame per pass as it fills.
	 *
	 * The instruction, address and any dummy cycles stay with the CPU in
	 * both cases. In enhanced mode they are two frames, and in Standard SPI
	 * mode they are not frames in memory at all -- each is one byte of a
	 * word the caller handed over -- so there is nothing for a channel to
	 * read even if it were worth the setup.
	 */
	pio_frames = tx_frames;

	if (plan.rx_frames == 0U && plan.payload_frames != 0U &&
	    tx_frames > N32_QSPI_FIFO_DEPTH) {
#if N32_QSPI_HAS_TX_DMA
		/*
		 * The CPU fills the FIFO nearly to the top and the channel takes
		 * over from the first frame that did not fit. Starting it on the
		 * frames still to be pushed rather than from the head of the
		 * payload keeps a FIFO of frames between the core and the
		 * channel's first write: whatever the channel's latency turns out
		 * to be, and whether its request waits for a device to be
		 * selected, there is nothing to starve for as long as those take
		 * to shift out.
		 *
		 * Nearly, not fully. A channel that is enabled hands over one
		 * frame before anything asks it to -- with the block enabled and a
		 * full FIFO, enabling the channel moves the level from 32 to 32
		 * while its own counter drops by one, and with 31 queued the same
		 * transfer moves the level to 32. The frame is not queued into a
		 * full FIFO, it is dropped, and the payload then arrives one frame
		 * short from that point on: the flash writes a byte stream to
		 * consecutive addresses, so a frame lost at the handover shifts
		 * every frame behind it and the page comes back with one byte of
		 * the payload missing rather than with a wrong byte. Leaving the
		 * slot open costs nothing, because the channel's first frame lands
		 * in it and the FIFO is at its full thirty-two either way.
		 */
		tx_dma = true;
		pio_frames = MIN(tx_frames, N32_QSPI_FIFO_DEPTH - 1U);
#else
		/*
		 * Refused, not attempted. The flash commits whatever arrives, so
		 * a page program that stops part way through leaves a page
		 * nobody can read back and nobody can repair, and the caller is
		 * the only one who can still do something about it.
		 */
		LOG_ERR("a %u frame transmit needs a feed this controller cannot"
			" give it from the CPU, and the device tree gives it no"
			" transmit DMA channel", tx_frames);
		return -ENOTSUP;
#endif
	}

	timeout_cyc = k_ms_to_cyc_ceil32(xfer->timeout != 0U ? xfer->timeout
							     : N32_QSPI_DEFAULT_TIMEOUT_MS);
	start = k_cycle_get_32();

	/* CTRL0, CTRL1, ENH_CTRL0 and BAUD are only writable while disabled. */
	cfg->regs->EN = 0U;

	cfg->regs->CTRL0 = plan.ctrl0;
	cfg->regs->CTRL1 = plan.ctrl1;
	cfg->regs->ENH_CTRL0 = plan.enh_ctrl0;
	cfg->regs->BAUD = data->dev_cfg.baud;
	cfg->regs->TXFT = QSPI_TXFT_TEI_0;
	cfg->regs->RXFT = QSPI_RXFT_TFI_0;
	cfg->regs->SLAVE_EN = 0U;
#if N32_QSPI_HAS_TX_DMA
	/*
	 * The block's own transmit request, left on for the packets that use
	 * it and cleared for the ones that do not, so that neither kind is at
	 * the mercy of what the last one left behind. Whether the request waits
	 * for a device to be selected before it fires is not something the
	 * datasheet says, which is why the FIFO is filled first and the channel
	 * started before CE either way: a transmit then always has a full FIFO
	 * of frames queued ahead of whatever the channel adds.
	 */
	cfg->regs->DMA_CTRL = tx_dma ? QSPI_DMA_CTRL_TX_DMA_EN : 0U;
	cfg->regs->DMATDL_CTRL = N32_QSPI_DMA_TX_LEVEL;
#endif
	cfg->regs->EN = QSPI_EN_QEN;

	/*
	 * Fill the TX FIFO before CE is asserted. The core does not shift
	 * anything until SLAVE_EN selects a device, so this is free, and for a
	 * transmit the CPU feeds by itself it is all the headroom there is:
	 * whatever can be pushed while those frames shift is the rest of it,
	 * which is why nothing longer than the FIFO is fed this way.
	 */
	while (next_tx < pio_frames && n32_qspi_tx_ready(cfg)) {
		n32_qspi_push(cfg, n32_qspi_tx_frame(&plan, pkt, next_tx));
		next_tx++;
	}

	prefilled = next_tx;

#if N32_QSPI_HAS_TX_DMA
	if (tx_dma) {
		/*
		 * The frames already in the FIFO came from the head of the
		 * packet, so the channel picks up at the frame the fill above
		 * stopped on and carries the rest of it.
		 */
		ret = n32_qspi_dma_tx(dev, &plan, pkt, prefilled, tx_frames - prefilled);
		if (ret < 0) {
			goto out;
		}
	}
#endif

	cfg->regs->SLAVE_EN = BIT(data->dev_id->dev_idx);

	for (;;) {
		bool progress = false;

		/*
		 * How hard the transmit FIFO is pushed depends on whether the
		 * packet is taking frames in at the same time.
		 *
		 * A packet that only shifts frames out is refilled to the top on
		 * every pass. A packet that is also receiving keeps only one
		 * frame per pass, because a full transmit FIFO is what stops the
		 * other direction working: filling it turns the 260 frame read
		 * after an erase into one that takes in 33 frames -- the FIFO
		 * depth -- and then nothing, with the core still shifting and
		 * discarding everything the flash sends, where one frame per pass
		 * takes all 260. The receive FIFO is the simple side of this: one
		 * frame per pass, as it has been; draining it in a burst was
		 * tried and the same read stopped in the same place.
		 *
		 * A transmitting packet whose payload is on a DMA channel has
		 * nothing left to push by the time this loop runs: the CPU filled
		 * the FIFO before CE and the channel took over from there, so the
		 * loop below finds next_tx already at pio_frames and leaves it
		 * alone.
		 */
		do {
			if (next_tx >= pio_frames || !n32_qspi_tx_ready(cfg)) {
				break;
			}

			n32_qspi_push(cfg, n32_qspi_tx_frame(&plan, pkt, next_tx));
			next_tx++;
			progress = true;
		} while (plan.rx_frames == 0U);

		if (popped < plan.rx_frames && n32_qspi_rx_ready(cfg)) {
			uint32_t word = n32_qspi_pop(cfg);

			if (popped >= plan.discard_frames) {
				pkt->data_buf[written] = (uint8_t)(word & 0xFFU);
				written++;
			}

			popped++;
			progress = true;
		}

		if (next_tx == pio_frames && popped == plan.rx_frames) {
			break;
		}

		/*
		 * The liveness check runs on a pass that moved nothing, which is
		 * when the engine is waiting on the core rather than on itself.
		 * That also keeps k_cycle_get_32() out of the loop while frames
		 * are still going out.
		 */
		if (!progress && n32_qspi_elapsed(start, timeout_cyc)) {
			LOG_ERR("transfer timed out: %u/%u frames sent, %u/%u received,"
				" %u fitted around CE, TXFN %u, RXFN %u",
				next_tx, tx_frames, popped, plan.rx_frames, prefilled,
				cfg->regs->TXFN, cfg->regs->RXFN);
			ret = -ETIMEDOUT;
			goto out;
		}
	}

#if N32_QSPI_HAS_TX_DMA
	if (tx_dma) {
		/*
		 * The channel owns the rest of the transmit, so this is where a
		 * packet that is not going to be sent in full is caught: the
		 * frames it has still to hand over are frames the flash will
		 * never see. Waiting for the retire first would not do it. An
		 * enhanced transmit ends when its FIFO runs dry, so a channel
		 * that stopped early leaves the core finishing what it has and
		 * reporting a clean transfer, and the caller would get a sector
		 * of silence back as a successful write -- which is the one
		 * outcome nobody downstream can detect or undo.
		 */
		uint32_t left;

		while ((left = n32_qspi_dma_left(cfg)) != 0U) {
			if (n32_qspi_elapsed(start, timeout_cyc)) {
				LOG_ERR("the transmit DMA channel stopped with %u of %u"
					" frames of the payload still to hand over",
					left, plan.payload_frames);
				ret = -EIO;
				goto out;
			}
		}
	}
#endif

	/*
	 * Wait for the transfer to retire before CE is released. In TX_AND_RX
	 * mode the core reports the last frame received, not the last one
	 * shifted out, so this is what holds CE through the tail of a read.
	 *
	 * A transmit has no frame count to end on -- NDF is only consulted for a
	 * receive -- so the core ends it when it runs out of frames to shift, and
	 * BUSY clearing is what says it has. In enhanced mode that is the only
	 * signal there is: the core leaves one entry behind in the transmit FIFO
	 * once the last frame is on the wire, so the level stops at one and
	 * waiting for it to reach zero, the way a receive can, runs into the
	 * timeout with nothing left to do on the bus. Standard mode does not do
	 * this -- a 260 frame page program retires with the FIFO genuinely empty
	 * -- so the relaxation is kept to the mode that needs it.
	 *
	 * BUSY has to be seen asserted first: between SLAVE_EN selecting the
	 * device and the core picking the transfer up, BUSY is still clear, and
	 * reading it as a finished transfer there would drop CE before the first
	 * frame went out.
	 */
	bool enhanced_tx = plan.enhanced && plan.rx_frames == 0U;
	bool started = false;

	while (!n32_qspi_idle(cfg)) {
		if ((cfg->regs->STS & QSPI_STS_BUSY) != 0U) {
			started = true;
		} else if (enhanced_tx && started) {
			break;
		}

		if (n32_qspi_elapsed(start, timeout_cyc)) {
			LOG_ERR("the transfer did not retire: STS %08x, TXFN %u, RXFN %u",
				cfg->regs->STS, cfg->regs->TXFN, cfg->regs->RXFN);
			ret = -ETIMEDOUT;
			goto out;
		}
	}

	/*
	 * Enhanced mode is left out of this: the core keeps one entry behind in
	 * the FIFO once the last frame is on the wire, as the retire wait above
	 * describes, so a level of one there says nothing about the packet. In
	 * Standard SPI mode the FIFO is genuinely empty when the count is done,
	 * and anything still in it is a frame the flash did not get.
	 */
	if (plan.rx_frames == 0U && !plan.enhanced && cfg->regs->TXFN != 0U) {
		LOG_WRN("the transfer ended with %u of %u frames still in the"
			" transmit FIFO, of which %u fitted around CE",
			cfg->regs->TXFN, tx_frames, prefilled);
	}

	ret = 0;

out:
	cfg->regs->SLAVE_EN = 0U;
#if N32_QSPI_HAS_TX_DMA
	if (tx_dma) {
		/*
		 * Before the block is disabled, not after: a channel still
		 * running would carry on writing frames into the data register
		 * of a block whose FIFO has just been flushed, and those frames
		 * would be waiting at the head of the next packet.
		 */
		n32_qspi_dma_stop(cfg);
	}
#endif
	cfg->regs->EN = 0U;

	/*
	 * Disabling the block flushes its FIFOs, and that flush is what keeps a
	 * frame left over from a transmit from going out at the head of the next
	 * packet, in front of that packet's instruction: an SPI flash reads
	 * whatever arrives first as an opcode, and one of the 256 byte values a
	 * page program puts on the bus is 0xb9, deep power down. The vendor HAL
	 * resets the whole block between transfers for the same reason. The
	 * levels are read rather than trusted -- if the flush did not happen the
	 * next packet starts with a stray frame on the bus, and it is the flash
	 * that pays for it.
	 */
	if (cfg->regs->TXFN != 0U || cfg->regs->RXFN != 0U) {
		uint32_t last = next_tx != 0U ? n32_qspi_tx_frame(&plan, pkt, next_tx - 1U) : 0U;

		LOG_ERR("the block was disabled with %u TX and %u RX frames still in"
			" the FIFOs; the last frame pushed was %08x",
			cfg->regs->TXFN, cfg->regs->RXFN, last);
	}

	return ret;
}

/*
 * Run one packet, as however many transactions the block can carry it in.
 *
 * A receive longer than a packet may be is the one case that is split. Each
 * transaction re-sends the instruction and address and reads the next stretch of
 * the same read, which is what the flash answers a single read of that run
 * with: it has no notion of a transaction at all, only of the clock edges in
 * front of it, and nothing in a NOR read is a function of where it started
 * beyond the address itself. N32_QSPI_RX_CHUNK_FRAMES is where the length comes
 * from.
 *
 * A transmit is never split, because a page program that stops and starts again
 * is a page the flash commits in pieces, with a program cycle for each.
 */
static int n32_qspi_send_packet(const struct device *dev, const struct mspi_xfer *xfer,
				const struct mspi_xfer_packet *pkt)
{
	uint32_t done = 0U;

	if (pkt->dir != MSPI_RX || pkt->num_bytes <= N32_QSPI_RX_CHUNK_FRAMES) {
		return n32_qspi_run_packet(dev, xfer, pkt);
	}

	while (done < pkt->num_bytes) {
		struct mspi_xfer_packet part = *pkt;
		int ret;

		part.num_bytes = MIN(pkt->num_bytes - done, N32_QSPI_RX_CHUNK_FRAMES);
		part.address = pkt->address + done;
		part.data_buf = pkt->data_buf + done;

		ret = n32_qspi_run_packet(dev, xfer, &part);
		if (ret < 0) {
			return ret;
		}

		done += part.num_bytes;
	}

	return 0;
}

static int mspi_n32_transceive(const struct device *dev,
			       const struct mspi_dev_id *dev_id,
			       const struct mspi_xfer *xfer)
{
	struct mspi_n32_data *data = dev->data;
	int ret;

	if (xfer->num_packet == 0U || xfer->packets == NULL ||
	    xfer->timeout > CONFIG_MSPI_COMPLETION_TIMEOUT_TOLERANCE) {
		return -EINVAL;
	}

	if (xfer->async) {
		return -ENOTSUP;
	}

	if (xfer->hold_ce) {
		LOG_ERR("hold_ce is not supported");
		return -ENOTSUP;
	}

	if (data->dev_id != dev_id) {
		LOG_ERR("mspi_dev_config() has not been called for this device");
		return -EINVAL;
	}

	if (data->clock_freq == 0U || data->dev_cfg.baud == 0U) {
		LOG_ERR("this device has no usable configuration");
		return -EINVAL;
	}

	if (data->in_use) {
		return -EBUSY;
	}

	data->in_use = true;

	for (uint32_t i = 0U; i < xfer->num_packet; i++) {
		const struct mspi_xfer_packet *pkt = &xfer->packets[i];

		if (pkt->num_bytes > 0U && pkt->data_buf == NULL) {
			ret = -EINVAL;
			goto out;
		}

		if (pkt->num_bytes + xfer->cmd_length + xfer->addr_length >
		    N32_QSPI_FRAMES_MAX) {
			LOG_ERR("packet %u is longer than the %u frames the block counts",
				i, N32_QSPI_FRAMES_MAX);
			ret = -EINVAL;
			goto out;
		}

		ret = n32_qspi_send_packet(dev, xfer, pkt);
		if (ret < 0) {
			goto out;
		}
	}

out:
	data->in_use = false;

	return ret;
}

static int mspi_n32_dev_config(const struct device *dev,
			       const struct mspi_dev_id *dev_id,
			       const enum mspi_dev_cfg_mask param_mask,
			       const struct mspi_dev_cfg *dev_cfg)
{
	const struct mspi_n32_config *cfg = dev->config;
	struct mspi_n32_data *data = dev->data;
	struct mspi_n32_dev_cfg *stored = &data->dev_cfg;
	uint32_t frf;
	uint32_t trans_type;

	if (dev_id == NULL || dev_id->dev_idx > N32_QSPI_DEV_IDX_MAX) {
		LOG_ERR("device index is out of range");
		return -EINVAL;
	}

	if (param_mask != MSPI_DEVICE_CONFIG_NONE && dev_cfg == NULL) {
		return -EINVAL;
	}

	/*
	 * The controller lock is taken here and held for the whole session,
	 * until the device hands it back through mspi_get_channel_status().
	 * A second device asking for it while it is held is told so rather than
	 * left waiting for the first one to finish.
	 */
	if (data->dev_id != dev_id) {
		if (k_mutex_lock(&data->lock,
				 K_MSEC(CONFIG_MSPI_COMPLETION_TIMEOUT_TOLERANCE)) != 0) {
			LOG_ERR("controller is held by another device");
			return -EBUSY;
		}
	}

	data->dev_id = dev_id;

	if (param_mask == MSPI_DEVICE_CONFIG_NONE && !cfg->mspicfg.sw_multi_periph) {
		/* Nothing to change, the device ID was all that was asked for. */
		return 0;
	}

	if ((param_mask & MSPI_DEVICE_CONFIG_IO_MODE) != 0U) {
		if (!n32_qspi_io_mode(dev_cfg->io_mode, &frf, &trans_type)) {
			LOG_ERR("I/O mode %d is not supported", dev_cfg->io_mode);
			return -ENOTSUP;
		}

		stored->io_mode = dev_cfg->io_mode;
	}

	if ((param_mask & MSPI_DEVICE_CONFIG_CPP) != 0U) {
		if (dev_cfg->cpp > MSPI_CPP_MODE_3) {
			LOG_ERR("clock mode %d is not supported", dev_cfg->cpp);
			return -ENOTSUP;
		}

		stored->cpp = dev_cfg->cpp;
	}

	if ((param_mask & MSPI_DEVICE_CONFIG_DATA_RATE) != 0U &&
	    dev_cfg->data_rate != MSPI_DATA_RATE_SINGLE) {
		/* DDR needs TXD_DRIVE_EDGE programmed from the divider and a
		 * device that wants it; nothing on this board does.
		 */
		LOG_ERR("data rate %d is not supported", dev_cfg->data_rate);
		return -ENOTSUP;
	}

	if ((param_mask & MSPI_DEVICE_CONFIG_FREQUENCY) != 0U) {
		if (dev_cfg->freq == 0U) {
			return -EINVAL;
		}

		if (dev_cfg->freq > data->clock_freq / N32_QSPI_BAUD_MIN) {
			LOG_ERR("clock %u Hz is above the %u Hz the block can drive",
				dev_cfg->freq, data->clock_freq / N32_QSPI_BAUD_MIN);
			return -EINVAL;
		}

		stored->baud = n32_qspi_baud(data->clock_freq, dev_cfg->freq);
	}

	return 0;
}

/*
 * API implementation of mspi_get_channel_status.
 *
 * On a busy controller this reports -EBUSY. On an idle one it ends the session
 * that mspi_dev_config() started and releases the lock, which is how
 * flash_mspi_nor.c hands the controller back between flash operations.
 */
static int mspi_n32_get_channel_status(const struct device *dev, uint8_t ch)
{
	const struct mspi_n32_config *cfg = dev->config;
	struct mspi_n32_data *data = dev->data;

	ARG_UNUSED(ch);

	if (data->in_use || !n32_qspi_idle(cfg)) {
		return -EBUSY;
	}

	if (data->dev_id != NULL) {
		data->dev_id = NULL;
		k_mutex_unlock(&data->lock);
	}

	return 0;
}

static int mspi_n32_config(const struct mspi_dt_spec *spec)
{
	const struct mspi_n32_config *cfg = spec->bus->config;
	struct mspi_n32_data *data = spec->bus->data;

	if (spec->config.op_mode != MSPI_OP_MODE_CONTROLLER ||
	    spec->config.duplex != MSPI_HALF_DUPLEX ||
	    spec->config.channel_num != 0U) {
		LOG_ERR("only one half duplex controller channel is supported");
		return -ENOTSUP;
	}

	if (data->in_use || data->dev_id != NULL) {
		LOG_ERR("controller is in use, cannot be reconfigured");
		return -EBUSY;
	}

	if (spec->config.re_init) {
		/*
		 * The block is programmed per packet, so re-initialising only
		 * means putting it back into a state where nothing is selected
		 * and no clock is running.
		 */
		cfg->regs->EN = 0U;
		cfg->regs->SLAVE_EN = 0U;
	}

	return 0;
}

static int mspi_n32_pm_action(const struct device *dev, enum pm_device_action action)
{
	const struct mspi_n32_config *cfg = dev->config;
	struct mspi_n32_data *data = dev->data;
	int ret;

	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		if (cfg->pcfg != NULL) {
			ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
			if (ret < 0) {
				LOG_ERR("cannot apply the default pin state: %d", ret);
				return ret;
			}
		}

		ret = clock_control_on(DEVICE_DT_GET(DT_NODELABEL(rcc)),
				       (clock_control_subsys_t)&cfg->clk_en);
		if (ret < 0) {
			LOG_ERR("cannot enable the QSPI clock: %d", ret);
			return ret;
		}

		ret = clock_control_get_rate(DEVICE_DT_GET(DT_NODELABEL(rcc)),
					     (clock_control_subsys_t)&cfg->clk_en,
					     &data->clock_freq);
		if (ret < 0) {
			LOG_ERR("cannot read the QSPI clock rate: %d", ret);
			return ret;
		}

		/* Whatever the block held before this, nothing is selected now. */
		cfg->regs->EN = 0U;
		cfg->regs->SLAVE_EN = 0U;

		return 0;

	case PM_DEVICE_ACTION_SUSPEND:
		if (data->in_use || data->dev_id != NULL) {
			LOG_ERR("controller is in use, cannot be suspended");
			return -EBUSY;
		}

		cfg->regs->EN = 0U;
		cfg->regs->SLAVE_EN = 0U;

		return clock_control_off(DEVICE_DT_GET(DT_NODELABEL(rcc)),
					 (clock_control_subsys_t)&cfg->clk_en);

	default:
		return -ENOTSUP;
	}
}

static int mspi_n32_init(const struct device *dev)
{
	const struct mspi_n32_config *cfg = dev->config;
	struct mspi_n32_data *data = dev->data;

	/*
	 * Leave the block in a state where nothing is selected and no clock is
	 * running until a device is configured. CTRL0 is set to what every
	 * packet below uses anyway, so a stray access before the first
	 * mspi_dev_config() puts nothing on the bus.
	 */
	cfg->regs->EN = 0U;
	cfg->regs->SLAVE_EN = 0U;
	cfg->regs->CTRL0 = N32_QSPI_CTRL0_DFS(N32_QSPI_FRAME_BITS) |
			   N32_QSPI_CTRL0_CFS(N32_QSPI_FRAME_BITS);
	cfg->regs->CTRL1 = 0U;
	cfg->regs->ENH_CTRL0 = 0U;
	cfg->regs->IMASK = 0U;

	data->dev_cfg.io_mode = MSPI_IO_MODE_SINGLE;
	data->dev_cfg.cpp = MSPI_CPP_MODE_0;

	return pm_device_driver_init(dev, mspi_n32_pm_action);
}

static DEVICE_API(mspi, mspi_n32_driver_api) = {
	.config = mspi_n32_config,
	.dev_config = mspi_n32_dev_config,
	.get_channel_status = mspi_n32_get_channel_status,
	.transceive = mspi_n32_transceive,
};

#define MSPI_N32_CONFIG(index)								\
	{										\
		.channel_num = 0,							\
		.op_mode = DT_INST_ENUM_IDX_OR(index, op_mode,				\
					       MSPI_OP_MODE_CONTROLLER),		\
		.duplex = DT_INST_ENUM_IDX_OR(index, duplex, MSPI_HALF_DUPLEX),		\
		.max_freq = DT_INST_PROP_OR(index, clock_frequency, 0),			\
		.dqs_support = false,							\
		.num_periph = DT_INST_CHILD_NUM(index),					\
		.sw_multi_periph = DT_INST_PROP(index, software_multiperipheral),	\
	}

#define MSPI_N32_TX_DMA(index)								\
	.dma = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(index, tx)),			\
	.dma_channel = DT_INST_DMAS_CELL_BY_NAME(index, tx, channel),			\
	.dma_slot = DT_INST_DMAS_CELL_BY_NAME(index, tx, request),			\
	.dma_priority = N32_DMA_CH_CFG_PRIORITY_GET(					\
		DT_INST_DMAS_CELL_BY_NAME(index, tx, config)),

#define MSPI_N32_INIT(index)								\
	PINCTRL_DT_INST_DEFINE(index);							\
											\
	PM_DEVICE_DT_INST_DEFINE(index, mspi_n32_pm_action);				\
											\
	static const struct mspi_n32_config mspi_n32_config_##index = {			\
		.regs = (QSPI_Module *)DT_INST_REG_ADDR(index),				\
		.mspicfg = MSPI_N32_CONFIG(index),					\
		.clk_en = DT_INST_CLOCKS_CELL(index, bits),				\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index),				\
		IF_ENABLED(N32_QSPI_HAS_TX_DMA, (MSPI_N32_TX_DMA(index)))		\
	};										\
											\
	static struct mspi_n32_data mspi_n32_data_##index = {				\
		.lock = Z_MUTEX_INITIALIZER(mspi_n32_data_##index.lock),		\
	};										\
											\
	DEVICE_DT_INST_DEFINE(index, mspi_n32_init, PM_DEVICE_DT_INST_GET(index),	\
			      &mspi_n32_data_##index, &mspi_n32_config_##index,		\
			      POST_KERNEL, CONFIG_MSPI_INIT_PRIORITY,			\
			      &mspi_n32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MSPI_N32_INIT)
