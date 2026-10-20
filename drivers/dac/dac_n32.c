/*
 * Copyright (c) 2026 Nations Technologies
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nsing_n32_dac

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/dac.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/kernel.h>

#include <n32g45x_dac.h>

struct dac_n32_config {
	const struct device *clock;
	uint32_t clock_id;
	const struct pinctrl_dev_config *pinctrl;
	uint8_t channels;
};

struct dac_n32_data {
	struct k_mutex lock;
	uint8_t resolution[2];
};

static int dac_n32_channel_setup(const struct device *dev,
				 const struct dac_channel_cfg *channel_cfg)
{
	const struct dac_n32_config *cfg = dev->config;
	struct dac_n32_data *data = dev->data;
	DAC_InitType init;
	uint8_t channel = channel_cfg->channel_id;
	uint32_t hw_channel;

	/* N32 channel identifiers follow the hardware numbering: 1 and 2. */
	if ((channel < 1U) || (channel > cfg->channels)) {
		return -EINVAL;
	}
	if ((channel_cfg->resolution != 8U) && (channel_cfg->resolution != 12U)) {
		return -ENOTSUP;
	}
	if (channel_cfg->internal) {
		return -ENOTSUP;
	}

	hw_channel = (channel == 1U) ? DAC_CHANNEL_1 : DAC_CHANNEL_2;
	k_mutex_lock(&data->lock, K_FOREVER);
	DAC_Enable(hw_channel, DISABLE);
	DAC_ClearStruct(&init);
	init.BufferOutput = channel_cfg->buffered ? DAC_BUFFOUTPUT_ENABLE :
						 DAC_BUFFOUTPUT_DISABLE;
	DAC_Init(hw_channel, &init);
	data->resolution[channel - 1U] = channel_cfg->resolution;
	DAC_Enable(hw_channel, ENABLE);
	k_mutex_unlock(&data->lock);

	return 0;
}

static int dac_n32_write_value(const struct device *dev, uint8_t channel, uint32_t value)
{
	const struct dac_n32_config *cfg = dev->config;
	struct dac_n32_data *data = dev->data;
	uint8_t resolution;

	if ((channel < 1U) || (channel > cfg->channels)) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	resolution = data->resolution[channel - 1U];
	if (resolution == 0U) {
		k_mutex_unlock(&data->lock);
		return -EINVAL;
	}
	if (value >= BIT(resolution)) {
		k_mutex_unlock(&data->lock);
		return -EINVAL;
	}

	uint32_t align = (resolution == 8U) ? DAC_ALIGN_R_8BIT : DAC_ALIGN_R_12BIT;

	if (channel == 1U) {
		DAC_SetCh1Data(align, (uint16_t)value);
	} else {
		DAC_SetCh2Data(align, (uint16_t)value);
	}
	k_mutex_unlock(&data->lock);
	return 0;
}

static int dac_n32_init(const struct device *dev)
{
	const struct dac_n32_config *cfg = dev->config;
	struct dac_n32_data *data = dev->data;
	int ret;

	if (!device_is_ready(cfg->clock)) {
		return -ENODEV;
	}
	ret = clock_control_on(cfg->clock, (clock_control_subsys_t)&cfg->clock_id);
	if (ret < 0) {
		return ret;
	}
	ret = pinctrl_apply_state(cfg->pinctrl, PINCTRL_STATE_DEFAULT);
	if ((ret < 0) && (ret != -ENOENT)) {
		return ret;
	}
	DAC_DeInit();
	k_mutex_init(&data->lock);
	return 0;
}

static DEVICE_API(dac, dac_n32_api) = {
	.channel_setup = dac_n32_channel_setup,
	.write_value = dac_n32_write_value,
};

#define DAC_N32_DEFINE(inst)                                                    \
	PINCTRL_DT_INST_DEFINE(inst);                                            \
	static const struct dac_n32_config dac_n32_config_##inst = {             \
		.clock = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),                 \
		.clock_id = DT_INST_CLOCKS_CELL(inst, bits),                      \
		.pinctrl = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                  \
		.channels = DT_INST_PROP(inst, num_channels),                     \
	};                                                                         \
	static struct dac_n32_data dac_n32_data_##inst;                           \
	DEVICE_DT_INST_DEFINE(inst, dac_n32_init, NULL, &dac_n32_data_##inst,     \
			      &dac_n32_config_##inst, POST_KERNEL,                 \
			      CONFIG_DAC_INIT_PRIORITY, &dac_n32_api);

DT_INST_FOREACH_STATUS_OKAY(DAC_N32_DEFINE)
