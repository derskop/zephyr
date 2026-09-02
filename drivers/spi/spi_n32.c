/*
 * Copyright (c) 2024 Nations Technologies
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nsing_n32_spi

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/n32_clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/spi.h>

#include <n32g45x_spi.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(spi_n32);

#include "spi_context.h"

struct spi_n32_config {
	SPI_Module *regs;
	uint32_t apb_clk_en;
	const struct pinctrl_dev_config *pcfg;
};

struct spi_n32_data {
	struct spi_context ctx;
};

static int spi_n32_configure(const struct device *dev,
			     const struct spi_config *config)
{
	struct spi_n32_data *data = dev->data;
	const struct spi_n32_config *cfg = dev->config;
	SPI_InitType init;
	uint32_t bus_freq;
	int br;

	if (spi_context_configured(&data->ctx, config)) {
		return 0;
	}

	if (SPI_OP_MODE_GET(config->operation) == SPI_OP_MODE_SLAVE) {
		LOG_ERR("Slave mode not supported");
		return -ENOTSUP;
	}

	/* Disable SPI before reconfiguring */
	SPI_Enable(cfg->regs, DISABLE);

	SPI_InitStruct(&init);

	init.DataDirection = SPI_DIR_DOUBLELINE_FULLDUPLEX;
	init.SpiMode = SPI_MODE_MASTER;

	if (SPI_WORD_SIZE_GET(config->operation) == 8) {
		init.DataLen = SPI_DATA_SIZE_8BITS;
	} else {
		init.DataLen = SPI_DATA_SIZE_16BITS;
	}

	init.CLKPOL = (config->operation & SPI_MODE_CPOL) ?
		      SPI_CLKPOL_HIGH : SPI_CLKPOL_LOW;
	init.CLKPHA = (config->operation & SPI_MODE_CPHA) ?
		      SPI_CLKPHA_SECOND_EDGE : SPI_CLKPHA_FIRST_EDGE;
	init.NSS = SPI_NSS_SOFT;
	init.FirstBit = (config->operation & SPI_TRANSFER_LSB) ?
			SPI_FB_LSB : SPI_FB_MSB;

	/* Pick the smallest prescaler that respects the requested frequency */
	(void)clock_control_get_rate(DEVICE_DT_GET(DT_NODELABEL(rcc)),
				     (clock_control_subsys_t)&cfg->apb_clk_en,
				     &bus_freq);

	init.BaudRatePres = SPI_BR_PRESCALER_256;
	for (br = 0; br <= 7; br++) {
		if ((bus_freq >> (br + 1)) <= config->frequency) {
			init.BaudRatePres = (uint16_t)(br << 3);
			break;
		}
	}

	SPI_Init(cfg->regs, &init);
	SPI_Enable(cfg->regs, ENABLE);

	data->ctx.config = config;

	return 0;
}

static int spi_n32_frame_exchange(const struct device *dev)
{
	struct spi_n32_data *data = dev->data;
	const struct spi_n32_config *cfg = dev->config;
	struct spi_context *ctx = &data->ctx;
	uint8_t dfs = (SPI_WORD_SIZE_GET(ctx->config->operation) == 8) ? 1 : 2;
	uint16_t tx_frame = 0U;
	uint16_t rx_frame;

	/* Wait until the TX buffer is empty */
	while (SPI_I2S_GetStatus(cfg->regs, SPI_I2S_TE_FLAG) == RESET) {
	}

	if (dfs == 1) {
		if (spi_context_tx_buf_on(ctx)) {
			tx_frame = UNALIGNED_GET((uint8_t *)ctx->tx_buf);
		}
		SPI_I2S_TransmitData(cfg->regs, tx_frame);
		spi_context_update_tx(ctx, dfs, 1);
	} else {
		if (spi_context_tx_buf_on(ctx)) {
			tx_frame = UNALIGNED_GET((uint16_t *)ctx->tx_buf);
		}
		SPI_I2S_TransmitData(cfg->regs, tx_frame);
		spi_context_update_tx(ctx, dfs, 1);
	}

	/* Wait until a word is received */
	while (SPI_I2S_GetStatus(cfg->regs, SPI_I2S_RNE_FLAG) == RESET) {
	}

	if (dfs == 1) {
		rx_frame = SPI_I2S_ReceiveData(cfg->regs);
		if (spi_context_rx_buf_on(ctx)) {
			UNALIGNED_PUT(rx_frame, (uint8_t *)ctx->rx_buf);
		}
		spi_context_update_rx(ctx, dfs, 1);
	} else {
		rx_frame = SPI_I2S_ReceiveData(cfg->regs);
		if (spi_context_rx_buf_on(ctx)) {
			UNALIGNED_PUT(rx_frame, (uint16_t *)ctx->rx_buf);
		}
		spi_context_update_rx(ctx, dfs, 1);
	}

	return 0;
}

static int spi_n32_transceive(const struct device *dev,
			      const struct spi_config *config,
			      const struct spi_buf_set *tx_bufs,
			      const struct spi_buf_set *rx_bufs)
{
	struct spi_n32_data *data = dev->data;
	const struct spi_n32_config *cfg = dev->config;
	uint8_t dfs = (SPI_WORD_SIZE_GET(config->operation) == 8) ? 1 : 2;
	int ret;

	spi_context_lock(&data->ctx, false, NULL, NULL, config);

	ret = spi_n32_configure(dev, config);
	if (ret < 0) {
		goto out;
	}

	SPI_Enable(cfg->regs, ENABLE);

	spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, dfs);

	spi_context_cs_control(&data->ctx, true);

	do {
		ret = spi_n32_frame_exchange(dev);
		if (ret < 0) {
			break;
		}
	} while (spi_context_tx_on(&data->ctx) ||
		 spi_context_rx_on(&data->ctx));

	/* Wait until the last frame has been fully shifted out */
	while (SPI_I2S_GetStatus(cfg->regs, SPI_I2S_BUSY_FLAG) == SET) {
	}

	spi_context_cs_control(&data->ctx, false);

	SPI_Enable(cfg->regs, DISABLE);

out:
	spi_context_release(&data->ctx, ret);

	return ret;
}

static int spi_n32_release(const struct device *dev,
			   const struct spi_config *config)
{
	struct spi_n32_data *data = dev->data;

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static DEVICE_API(spi, spi_n32_driver_api) = {
	.transceive = spi_n32_transceive,
	.release = spi_n32_release,
};

static int spi_n32_init(const struct device *dev)
{
	struct spi_n32_data *data = dev->data;
	const struct spi_n32_config *cfg = dev->config;
	int ret;

	ret = clock_control_on(DEVICE_DT_GET(DT_NODELABEL(rcc)),
			       (clock_control_subsys_t)&cfg->apb_clk_en);
	if (ret < 0) {
		LOG_ERR("Failed to enable SPI clock: %d", ret);
		return ret;
	}

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("Failed to apply SPI pinctrl state: %d", ret);
		return ret;
	}

	ret = spi_context_cs_configure_all(&data->ctx);
	if (ret < 0) {
		return ret;
	}

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

#define SPI_N32_INIT(idx)							\
	PINCTRL_DT_INST_DEFINE(idx);						\
	static struct spi_n32_data spi_n32_data_##idx = {			\
		SPI_CONTEXT_INIT_LOCK(spi_n32_data_##idx, ctx),		\
		SPI_CONTEXT_INIT_SYNC(spi_n32_data_##idx, ctx),		\
		SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(idx), ctx) };	\
	static const struct spi_n32_config spi_n32_config_##idx = {		\
		.regs = (SPI_Module *)DT_INST_REG_ADDR(idx),			\
		.apb_clk_en = DT_INST_CLOCKS_CELL(idx, bits),			\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(idx),			\
	};									\
	SPI_DEVICE_DT_INST_DEFINE(idx, spi_n32_init, NULL,			\
				  &spi_n32_data_##idx, &spi_n32_config_##idx,	\
				  POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,	\
				  &spi_n32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SPI_N32_INIT);
