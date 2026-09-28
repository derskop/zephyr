/*
 * Copyright (c) 2024 Nations Technologies
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nsing_n32_dma

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/n32_clock_control.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util.h>

#include <n32g45x_dma.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dma_n32, CONFIG_DMA_LOG_LEVEL);

/*
 * Register layout (see DMA_Module / DMA_ChannelType in n32g45x.h):
 *
 *   DMA_Module + 0x00  INTSTS
 *   DMA_Module + 0x04  INTCLR
 *   DMA_Module + 0x08  channel 1: CHCFG, and then TXNUM/PADDR/MADDR/CHSEL
 *   ...                each channel occupies 0x14 bytes
 *   DMA_Module + 0xA8  CHMAPEN
 */
#define DMA_N32_CH_OFFSET(ch) (0x08U + 0x14U * (uint32_t)(ch))

/* Per-channel INTSTS/INTCLR bits, four per channel starting at bit 0 */
#define DMA_N32_GLBF(ch)  BIT(4U * (uint32_t)(ch) + 0U)
#define DMA_N32_TXCF(ch)  BIT(4U * (uint32_t)(ch) + 1U)
#define DMA_N32_HTXF(ch)  BIT(4U * (uint32_t)(ch) + 2U)
#define DMA_N32_ERRF(ch)  BIT(4U * (uint32_t)(ch) + 3U)
#define DMA_N32_ALLF(ch)  (DMA_N32_GLBF(ch) | DMA_N32_TXCF(ch) | DMA_N32_HTXF(ch) |  \
			   DMA_N32_ERRF(ch))

/* CHSEL is a 6 bit field; the vendor's request codes stop well short of it */
#define DMA_N32_CHSEL_MAX 0x3FU

/* TXNUM is 16 bits wide */
#define DMA_N32_BUF_SIZE_MAX 0xFFFFU

struct dma_n32_config {
	DMA_Module *dma;
	uint32_t channels;
	uint32_t clkid;
	bool mem2mem;
	void (*irq_configure)(void);
};

struct dma_n32_channel {
	dma_callback_t callback;
	void *user_data;
	uint32_t direction;
	uint32_t int_mask;
	bool busy;
	/*
	 * Transfer geometry, captured by dma_config()/dma_reload(). Gating CHEN
	 * keeps TXNUM but not the address registers: re-enabling restarts the
	 * channel from the addresses as programmed, so resuming correctly means
	 * re-deriving them from how many elements are left. Steps are bytes per
	 * element, or 0 for an address that does not advance.
	 *
	 * Block size and remaining are counted in elements, not bytes, because
	 * that is the unit TXNUM holds and the unit the hardware counter reads
	 * back. data_size is what converts them to the byte counts the DMA API
	 * uses.
	 */
	uint32_t block_size;
	uint32_t data_size;
	uint32_t maddr;
	uint32_t paddr;
	uint32_t mstep;
	uint32_t pstep;
	uint32_t remaining;
};

struct dma_n32_data {
	struct dma_context ctx;
	struct dma_n32_channel *channels;
	/*
	 * Which channels have been configured with an explicit CHSEL request
	 * code and which have been left on the hardwired default mapping.
	 * CHMAPEN is controller-wide, so the two sets cannot coexist; these
	 * masks are what lets dma_n32_config() reject an attempt to mix them.
	 * Only channels that actually use a peripheral request are tracked.
	 */
	uint32_t remap_mask;
	uint32_t default_mask;
};

static inline DMA_ChannelType *dma_n32_channel(const struct dma_n32_config *cfg, uint32_t ch)
{
	return (DMA_ChannelType *)((uintptr_t)cfg->dma + DMA_N32_CH_OFFSET(ch));
}

static inline uint32_t dma_n32_priority(uint32_t priority)
{
	return priority << 12U;
}

static inline uint32_t dma_n32_periph_width(uint32_t width)
{
	switch (width) {
	case 4:
		return DMA_PERIPH_DATA_SIZE_WORD;
	case 2:
		return DMA_PERIPH_DATA_SIZE_HALFWORD;
	default:
		return DMA_PERIPH_DATA_SIZE_BYTE;
	}
}

static inline uint32_t dma_n32_memory_width(uint32_t width)
{
	switch (width) {
	case 4:
		return DMA_MemoryDataSize_Word;
	case 2:
		return DMA_MemoryDataSize_HalfWord;
	default:
		return DMA_MemoryDataSize_Byte;
	}
}

/* Bytes an address advances per element, or 0 when it is held still */
static inline uint32_t dma_n32_step(uint32_t addr_adj, uint32_t data_size)
{
	return (addr_adj == DMA_ADDR_ADJ_INCREMENT) ? data_size : 0U;
}

/*
 * Program CHSEL and the controller-wide CHMAPEN bit.
 *
 * DMA_RequestRemap() is the vendor entry point for both, but its Cmd argument
 * is misleading: the disable branch ignores DMAyChx and clears CHMAPEN for the
 * whole controller. That is fine here because dma_n32_config() refuses to mix
 * remapped and default-mapped channels on one controller, so turning CHMAPEN
 * off cannot disturb a channel that still needs it.
 */
static void dma_n32_apply_remap(const struct dma_n32_config *cfg, uint32_t ch, uint32_t req)
{
	DMA_ChannelType *chx = dma_n32_channel(cfg, ch);

	DMA_RequestRemap(req, cfg->dma, chx, req != 0U ? ENABLE : DISABLE);

	if (req == 0U) {
		/* Leave nothing behind for a later CHMAPEN=1 to misinterpret */
		chx->CHSEL = 0U;
	}
}

static int dma_n32_config(const struct device *dev, uint32_t channel, struct dma_config *dma_cfg)
{
	const struct dma_n32_config *cfg = dev->config;
	struct dma_n32_data *data = dev->data;
	const struct dma_block_config *blk = dma_cfg->head_block;
	struct dma_n32_channel *ch;
	bool uses_request;
	uint32_t int_mask;
	uint32_t elements;
	DMA_InitType init;
	DMA_ChannelType *chx;

	if (channel >= cfg->channels) {
		LOG_ERR("channel must be < %u (%u)", cfg->channels, channel);
		return -EINVAL;
	}

	if (dma_cfg->block_count != 1U) {
		LOG_ERR("chained block transfer not supported");
		return -ENOTSUP;
	}

	if (blk == NULL) {
		LOG_ERR("head_block must not be NULL");
		return -EINVAL;
	}

	if (dma_cfg->channel_priority > 3U) {
		LOG_ERR("channel_priority must be < 4 (%u)", dma_cfg->channel_priority);
		return -EINVAL;
	}

	if (dma_cfg->source_data_size != 1U && dma_cfg->source_data_size != 2U &&
	    dma_cfg->source_data_size != 4U) {
		LOG_ERR("source_data_size must be 1, 2 or 4 (%u)", dma_cfg->source_data_size);
		return -EINVAL;
	}

	if (dma_cfg->dest_data_size != 1U && dma_cfg->dest_data_size != 2U &&
	    dma_cfg->dest_data_size != 4U) {
		LOG_ERR("dest_data_size must be 1, 2 or 4 (%u)", dma_cfg->dest_data_size);
		return -EINVAL;
	}

	/*
	 * One width drives both sides: TXNUM counts elements, so a transfer has
	 * a single well-defined element size or none at all. The hardware would
	 * not object to unequal widths, it would just spend a different number
	 * of counts on each side, which is never what a caller means.
	 */
	if (dma_cfg->source_data_size != dma_cfg->dest_data_size) {
		LOG_ERR("source_data_size and dest_data_size must match (%u vs %u)",
			dma_cfg->source_data_size, dma_cfg->dest_data_size);
		return -EINVAL;
	}

	/*
	 * block_size is bytes, TXNUM is elements. Refuse a block that is not a
	 * whole number of elements rather than silently rounding it down and
	 * leaving the tail of the buffer untouched.
	 */
	if (blk->block_size % dma_cfg->source_data_size != 0U) {
		LOG_ERR("block_size must be a multiple of the data size (%u vs %u)",
			blk->block_size, dma_cfg->source_data_size);
		return -EINVAL;
	}

	elements = blk->block_size / dma_cfg->source_data_size;
	if (elements > DMA_N32_BUF_SIZE_MAX) {
		LOG_ERR("block_size must be <= %u elements (%u)",
			DMA_N32_BUF_SIZE_MAX, elements);
		return -EINVAL;
	}

	if (blk->source_addr_adj != DMA_ADDR_ADJ_INCREMENT &&
	    blk->source_addr_adj != DMA_ADDR_ADJ_NO_CHANGE) {
		LOG_ERR("source_addr_adj must be DMA_ADDR_ADJ_INCREMENT or _NO_CHANGE (%u)",
			blk->source_addr_adj);
		return -ENOTSUP;
	}

	if (blk->dest_addr_adj != DMA_ADDR_ADJ_INCREMENT &&
	    blk->dest_addr_adj != DMA_ADDR_ADJ_NO_CHANGE) {
		LOG_ERR("dest_addr_adj must be DMA_ADDR_ADJ_INCREMENT or _NO_CHANGE (%u)",
			blk->dest_addr_adj);
		return -ENOTSUP;
	}

	if (dma_cfg->channel_direction > PERIPHERAL_TO_MEMORY) {
		LOG_ERR("channel_direction must be MEMORY_TO_MEMORY, MEMORY_TO_PERIPHERAL or "
			"PERIPHERAL_TO_MEMORY (%u)",
			dma_cfg->channel_direction);
		return -ENOTSUP;
	}

	if (dma_cfg->channel_direction == MEMORY_TO_MEMORY && !cfg->mem2mem) {
		LOG_ERR("MEMORY_TO_MEMORY not supported by this controller");
		return -ENOTSUP;
	}

	if (dma_cfg->half_complete_callback_en) {
		LOG_ERR("half transfer callback not supported");
		return -ENOTSUP;
	}

	if (dma_cfg->dma_slot > DMA_N32_CHSEL_MAX) {
		LOG_ERR("dma_slot must be <= 0x%02X (%u)", DMA_N32_CHSEL_MAX, dma_cfg->dma_slot);
		return -EINVAL;
	}

	/*
	 * A memory-to-memory channel ignores the request line entirely, so it
	 * neither needs CHMAPEN nor constrains the channels that do.
	 */
	uses_request = dma_cfg->channel_direction != MEMORY_TO_MEMORY;

	if (uses_request) {
		uint32_t others_default = data->default_mask & ~BIT(channel);
		uint32_t others_remapped = data->remap_mask & ~BIT(channel);

		if (dma_cfg->dma_slot != 0U && others_default != 0U) {
			LOG_ERR("cannot remap channel %u: channel(s) 0x%x use the default mapping",
				channel, others_default);
			return -EINVAL;
		}

		if (dma_cfg->dma_slot == 0U && others_remapped != 0U) {
			LOG_ERR("channel %u cannot use the default mapping: channel(s) 0x%x are "
				"remapped",
				channel, others_remapped);
			return -EINVAL;
		}
	}

	chx = dma_n32_channel(cfg, channel);

	/* Clears CHCFG, TXNUM, PADDR, MADDR and the interrupt flags. CHSEL and
	 * CHMAPEN are left alone, so they are programmed afterwards.
	 */
	DMA_DeInit(chx);

	init.PeriphAddr = (dma_cfg->channel_direction == MEMORY_TO_PERIPHERAL) ? blk->dest_address
									      : blk->source_address;
	init.MemAddr = (dma_cfg->channel_direction == MEMORY_TO_PERIPHERAL) ? blk->source_address
									   : blk->dest_address;
	init.BufSize = elements;
	init.PeriphInc = (dma_cfg->channel_direction == MEMORY_TO_PERIPHERAL)
				 ? ((blk->dest_addr_adj == DMA_ADDR_ADJ_INCREMENT)
					    ? DMA_PERIPH_INC_ENABLE
					    : DMA_PERIPH_INC_DISABLE)
				 : ((blk->source_addr_adj == DMA_ADDR_ADJ_INCREMENT)
					    ? DMA_PERIPH_INC_ENABLE
					    : DMA_PERIPH_INC_DISABLE);
	init.DMA_MemoryInc = (dma_cfg->channel_direction == MEMORY_TO_PERIPHERAL)
				     ? ((blk->source_addr_adj == DMA_ADDR_ADJ_INCREMENT)
						? DMA_MEM_INC_ENABLE
						: DMA_MEM_INC_DISABLE)
				     : ((blk->dest_addr_adj == DMA_ADDR_ADJ_INCREMENT)
						? DMA_MEM_INC_ENABLE
						: DMA_MEM_INC_DISABLE);
	init.PeriphDataSize = dma_n32_periph_width(
		(dma_cfg->channel_direction == MEMORY_TO_PERIPHERAL) ? dma_cfg->dest_data_size
								    : dma_cfg->source_data_size);
	init.MemDataSize = dma_n32_memory_width(
		(dma_cfg->channel_direction == MEMORY_TO_PERIPHERAL) ? dma_cfg->source_data_size
								    : dma_cfg->dest_data_size);
	init.CircularMode = dma_cfg->cyclic ? DMA_MODE_CIRCULAR : DMA_MODE_NORMAL;
	init.Priority = dma_n32_priority(dma_cfg->channel_priority);
	init.Mem2Mem = (dma_cfg->channel_direction == MEMORY_TO_MEMORY) ? DMA_M2M_ENABLE
								       : DMA_M2M_DISABLE;

	switch (dma_cfg->channel_direction) {
	case MEMORY_TO_PERIPHERAL:
		init.Direction = DMA_DIR_PERIPH_DST;
		break;
	default:
		/* MEMORY_TO_MEMORY and PERIPHERAL_TO_MEMORY both put the
		 * peripheral (or the source) side on PADDR.
		 */
		init.Direction = DMA_DIR_PERIPH_SRC;
		break;
	}

	DMA_Init(chx, &init);

	if (uses_request) {
		dma_n32_apply_remap(cfg, channel, dma_cfg->dma_slot);

		if (dma_cfg->dma_slot != 0U) {
			data->remap_mask |= BIT(channel);
			data->default_mask &= ~BIT(channel);
		} else {
			data->default_mask |= BIT(channel);
			data->remap_mask &= ~BIT(channel);
		}
	}

	int_mask = DMA_INT_TXC;
	if (!dma_cfg->error_callback_dis) {
		int_mask |= DMA_INT_ERR;
	}

	ch = &data->channels[channel];
	ch->callback = dma_cfg->dma_callback;
	ch->user_data = dma_cfg->user_data;
	ch->direction = dma_cfg->channel_direction;
	ch->int_mask = int_mask;
	ch->busy = false;

	/* Remember the transfer so dma_n32_resume() can put the address
	 * registers back where the channel was gated, not where it started.
	 * Both counts are in elements, to match TXNUM.
	 */
	ch->block_size = elements;
	ch->data_size = dma_cfg->source_data_size;
	ch->remaining = elements;
	ch->maddr = init.MemAddr;
	ch->paddr = init.PeriphAddr;

	if (dma_cfg->channel_direction == MEMORY_TO_PERIPHERAL) {
		ch->mstep = dma_n32_step(blk->source_addr_adj, dma_cfg->source_data_size);
		ch->pstep = dma_n32_step(blk->dest_addr_adj, dma_cfg->dest_data_size);
	} else {
		ch->mstep = dma_n32_step(blk->dest_addr_adj, dma_cfg->dest_data_size);
		ch->pstep = dma_n32_step(blk->source_addr_adj, dma_cfg->source_data_size);
	}

	return 0;
}

static int dma_n32_reload(const struct device *dev, uint32_t channel, uint32_t src, uint32_t dst,
			  size_t size)
{
	const struct dma_n32_config *cfg = dev->config;
	struct dma_n32_data *data = dev->data;
	struct dma_n32_channel *ch;
	DMA_ChannelType *chx;
	uint32_t elements;

	if (channel >= cfg->channels) {
		LOG_ERR("reload channel must be < %u (%u)", cfg->channels, channel);
		return -EINVAL;
	}

	ch = &data->channels[channel];

	/*
	 * Only dma_config() ever sets the element size, so a zero here means the
	 * channel was never configured and there is no element size to count a
	 * reload in.
	 */
	if (ch->data_size == 0U) {
		LOG_ERR("channel %u has not been configured", channel);
		return -EINVAL;
	}

	if (size % ch->data_size != 0U) {
		LOG_ERR("reload size must be a multiple of the data size (%zu vs %u)", size,
			ch->data_size);
		return -EINVAL;
	}

	elements = size / ch->data_size;
	if (elements > DMA_N32_BUF_SIZE_MAX) {
		LOG_ERR("reload size must be <= %u elements (%u)", DMA_N32_BUF_SIZE_MAX, elements);
		return -EINVAL;
	}

	if (ch->busy) {
		return -EBUSY;
	}

	chx = dma_n32_channel(cfg, channel);

	DMA_EnableChannel(chx, DISABLE);
	DMA_SetCurrDataCounter(chx, (uint16_t)elements);

	switch (ch->direction) {
	case MEMORY_TO_MEMORY:
	case PERIPHERAL_TO_MEMORY:
		chx->MADDR = dst;
		chx->PADDR = src;
		ch->maddr = dst;
		ch->paddr = src;
		break;
	case MEMORY_TO_PERIPHERAL:
		chx->MADDR = src;
		chx->PADDR = dst;
		ch->maddr = src;
		ch->paddr = dst;
		break;
	}

	/* dma_config() fixed the strides; only the bases and the length moved */
	ch->block_size = elements;
	ch->remaining = elements;

	DMA_EnableChannel(chx, ENABLE);

	return 0;
}

static int dma_n32_start(const struct device *dev, uint32_t channel)
{
	const struct dma_n32_config *cfg = dev->config;
	struct dma_n32_data *data = dev->data;
	DMA_ChannelType *chx;

	if (channel >= cfg->channels) {
		LOG_ERR("start channel must be < %u (%u)", cfg->channels, channel);
		return -EINVAL;
	}

	chx = dma_n32_channel(cfg, channel);

	/*
	 * A transfer can only have been armed by dma_config() or dma_reload().
	 * dma_config() leaves the channel disabled, so re-arm it here.
	 */
	DMA_ConfigInt(chx, data->channels[channel].int_mask, ENABLE);
	DMA_EnableChannel(chx, ENABLE);
	data->channels[channel].busy = true;

	return 0;
}

static int dma_n32_stop(const struct device *dev, uint32_t channel)
{
	const struct dma_n32_config *cfg = dev->config;
	struct dma_n32_data *data = dev->data;
	DMA_ChannelType *chx;

	if (channel >= cfg->channels) {
		LOG_ERR("stop channel must be < %u (%u)", cfg->channels, channel);
		return -EINVAL;
	}

	chx = dma_n32_channel(cfg, channel);

	DMA_ConfigInt(chx, DMA_INT_TXC | DMA_INT_HTX | DMA_INT_ERR, DISABLE);
	DMA_EnableChannel(chx, DISABLE);
	cfg->dma->INTCLR = DMA_N32_ALLF(channel);
	data->channels[channel].busy = false;

	return 0;
}

/*
 * Suspend masks this channel's interrupts and gates it off; resume puts the
 * address registers back and gates it on again.
 *
 * Masking the interrupt rather than clearing the flags matters: a completion
 * that latched in INTSTS before the channel was gated stays latched, so
 * unmasking reports it. Clearing it here would silently drop that completion
 * and strand a chained transfer.
 *
 * The addresses have to be rewritten because clearing CHEN keeps TXNUM but not
 * MADDR/PADDR: re-enabling restarts the channel from the addresses as
 * programmed, so a bare gate would replay the block from its start and never
 * touch the tail. TXNUM's value is the count of elements still outstanding,
 * which is exactly what is needed to walk the bases forward to the point the
 * channel actually stopped at.
 */
static int dma_n32_suspend(const struct device *dev, uint32_t channel)
{
	const struct dma_n32_config *cfg = dev->config;
	struct dma_n32_data *data = dev->data;
	DMA_ChannelType *chx;

	if (channel >= cfg->channels) {
		LOG_ERR("suspend channel must be < %u (%u)", cfg->channels, channel);
		return -EINVAL;
	}

	chx = dma_n32_channel(cfg, channel);

	DMA_ConfigInt(chx, data->channels[channel].int_mask, DISABLE);
	DMA_EnableChannel(chx, DISABLE);

	/* Read after the gate: whatever was in flight when CHEN cleared has
	 * landed by now, so this is the authoritative count of what is left.
	 */
	data->channels[channel].remaining = DMA_GetCurrDataCounter(chx);

	return 0;
}

static int dma_n32_resume(const struct device *dev, uint32_t channel)
{
	const struct dma_n32_config *cfg = dev->config;
	struct dma_n32_data *data = dev->data;
	struct dma_n32_channel *ch;
	DMA_ChannelType *chx;
	uint32_t moved;

	if (channel >= cfg->channels) {
		LOG_ERR("resume channel must be < %u (%u)", cfg->channels, channel);
		return -EINVAL;
	}

	chx = dma_n32_channel(cfg, channel);
	ch = &data->channels[channel];

	/*
	 * remaining == block_size means nothing has moved yet, so the registers
	 * are already where the transfer should pick up; a larger remaining
	 * means the channel was never armed, and there is nothing to rewind.
	 */
	if (ch->remaining < ch->block_size) {
		moved = ch->block_size - ch->remaining;

		chx->PADDR = ch->paddr + moved * ch->pstep;
		chx->MADDR = ch->maddr + moved * ch->mstep;
		DMA_SetCurrDataCounter(chx, (uint16_t)ch->remaining);
	}

	DMA_ConfigInt(chx, ch->int_mask, ENABLE);
	DMA_EnableChannel(chx, ENABLE);

	return 0;
}

static int dma_n32_get_status(const struct device *dev, uint32_t channel, struct dma_status *stat)
{
	const struct dma_n32_config *cfg = dev->config;
	struct dma_n32_data *data = dev->data;
	DMA_ChannelType *chx;

	if (channel >= cfg->channels) {
		LOG_ERR("channel must be < %u (%u)", cfg->channels, channel);
		return -EINVAL;
	}

	chx = dma_n32_channel(cfg, channel);

	/* The hardware counts elements; the API reports bytes */
	stat->pending_length =
		(size_t)DMA_GetCurrDataCounter(chx) * data->channels[channel].data_size;
	stat->dir = data->channels[channel].direction;
	stat->busy = data->channels[channel].busy;

	return 0;
}

/*
 * Each channel has its own interrupt vector but they all land here; the handler
 * walks every channel and services whatever is pending, which makes it
 * idempotent no matter which vector fired.
 */
static void dma_n32_isr(const struct device *dev)
{
	const struct dma_n32_config *cfg = dev->config;
	struct dma_n32_data *data = dev->data;

	for (uint32_t i = 0; i < cfg->channels; i++) {
		uint32_t flags = cfg->dma->INTSTS & DMA_N32_ALLF(i);
		int err;

		if (flags == 0U) {
			continue;
		}

		/* INTCLR is write-one-to-clear for every channel at once */
		cfg->dma->INTCLR = flags;

		if ((flags & (DMA_N32_TXCF(i) | DMA_N32_ERRF(i))) == 0U) {
			/* Half-transfer only; not used by this driver */
			continue;
		}

		err = (flags & DMA_N32_ERRF(i)) ? -EIO : 0;

		data->channels[i].busy = false;

		if (data->channels[i].callback != NULL) {
			data->channels[i].callback(dev, data->channels[i].user_data, i, err);
		}
	}
}

static DEVICE_API(dma, dma_n32_driver_api) = {
	.config = dma_n32_config,
	.reload = dma_n32_reload,
	.start = dma_n32_start,
	.stop = dma_n32_stop,
	.suspend = dma_n32_suspend,
	.resume = dma_n32_resume,
	.get_status = dma_n32_get_status,
};

static int dma_n32_init(const struct device *dev)
{
	const struct dma_n32_config *cfg = dev->config;
	int ret;

	ret = clock_control_on(DEVICE_DT_GET(DT_NODELABEL(rcc)),
			       (clock_control_subsys_t)&cfg->clkid);
	if (ret < 0) {
		LOG_ERR("Failed to enable DMA clock: %d", ret);
		return ret;
	}

	for (uint32_t i = 0; i < cfg->channels; i++) {
		DMA_ChannelType *chx = dma_n32_channel(cfg, i);

		DMA_DeInit(chx);
		chx->CHSEL = 0U;
	}

	cfg->dma->CHMAPEN = 0U;

	cfg->irq_configure();

	return 0;
}

#define DMA_N32_IRQ_CONFIGURE(n, inst)                                                             \
	IRQ_CONNECT(DT_INST_IRQ_BY_IDX(inst, n, irq), DT_INST_IRQ_BY_IDX(inst, n, priority),       \
		    dma_n32_isr, DEVICE_DT_INST_GET(inst), 0);                                     \
	irq_enable(DT_INST_IRQ_BY_IDX(inst, n, irq));

#define DMA_N32_CONFIGURE_ALL_IRQS(inst, n) LISTIFY(n, DMA_N32_IRQ_CONFIGURE, (), inst)

#define DMA_N32_INIT(inst)                                                                         \
	static void dma_n32_irq_configure_##inst(void)                                             \
	{                                                                                          \
		DMA_N32_CONFIGURE_ALL_IRQS(inst, DT_NUM_IRQS(DT_DRV_INST(inst)));                  \
	}                                                                                          \
                                                                                                   \
	static const struct dma_n32_config dma_n32_config_##inst = {                               \
		.dma = (DMA_Module *)DT_INST_REG_ADDR(inst),                                       \
		.channels = DT_INST_PROP(inst, dma_channels),                                      \
		.clkid = DT_INST_CLOCKS_CELL(inst, bits),                                          \
		.mem2mem = DT_INST_PROP(inst, nsing_mem2mem),                                      \
		.irq_configure = dma_n32_irq_configure_##inst,                                     \
	};                                                                                         \
                                                                                                   \
	static struct dma_n32_channel                                                              \
		dma_n32_channels_##inst[DT_INST_PROP(inst, dma_channels)];                         \
	ATOMIC_DEFINE(dma_n32_atomic_##inst, DT_INST_PROP(inst, dma_channels));                    \
	static struct dma_n32_data dma_n32_data_##inst = {                                         \
		.ctx = {                                                                           \
			.magic = DMA_MAGIC,                                                        \
			.atomic = dma_n32_atomic_##inst,                                           \
			.dma_channels = DT_INST_PROP(inst, dma_channels),                          \
		},                                                                                 \
		.channels = dma_n32_channels_##inst,                                               \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, dma_n32_init, NULL, &dma_n32_data_##inst,                      \
			      &dma_n32_config_##inst, POST_KERNEL, CONFIG_DMA_INIT_PRIORITY,       \
			      &dma_n32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(DMA_N32_INIT)
