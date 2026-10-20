/*
 * Copyright (c) 2026 Nations Technologies
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nsing_n32_adc

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include <n32g45x_adc.h>
#include <n32g45x_rcc.h>

#define N32_ADC_CHANNELS 16U
#define N32_ADC_TIMEOUT_MS 100

struct adc_n32_config {
	ADC_Module *adc;
	const struct device *clock;
	uint32_t clock_id;
	const struct pinctrl_dev_config *pinctrl;
	uint8_t channels;
};

struct adc_n32_data {
	struct k_mutex lock;
	uint16_t configured;
	uint8_t sample_time[N32_ADC_CHANNELS];
};

/* ADC sample time codes correspond to 1.5, 7.5, ... 239.5 ADC clocks. */
static const uint16_t n32_sample_time_ceiling[] = {2, 8, 14, 29, 42, 56, 72, 240};

static int adc_n32_channel_setup(const struct device *dev,
				 const struct adc_channel_cfg *channel_cfg)
{
	const struct adc_n32_config *cfg = dev->config;
	struct adc_n32_data *data = dev->data;
	uint8_t channel = channel_cfg->channel_id;
	uint8_t sample_time = ADC_SAMP_TIME_41CYCLES5;

	/* The N32G45x external inputs are numbered from channel 1. */
	if ((channel == 0U) || (channel > cfg->channels)) {
		return -EINVAL;
	}
	if ((channel_cfg->gain != ADC_GAIN_1) ||
	    (channel_cfg->reference != ADC_REF_INTERNAL) ||
	    channel_cfg->differential) {
		return -ENOTSUP;
	}
	if (channel_cfg->acquisition_time != ADC_ACQ_TIME_DEFAULT) {
		if (ADC_ACQ_TIME_UNIT(channel_cfg->acquisition_time) != ADC_ACQ_TIME_TICKS) {
			return -ENOTSUP;
		}
		uint16_t ticks = ADC_ACQ_TIME_VALUE(channel_cfg->acquisition_time);

		for (sample_time = 0U; sample_time < ARRAY_SIZE(n32_sample_time_ceiling);
		     sample_time++) {
			if (ticks <= n32_sample_time_ceiling[sample_time]) {
				break;
			}
		}
		if (sample_time == ARRAY_SIZE(n32_sample_time_ceiling)) {
			return -ENOTSUP;
		}
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	data->sample_time[channel] = sample_time;
	data->configured |= BIT(channel);
	k_mutex_unlock(&data->lock);
	return 0;
}

static int adc_n32_calibrate(ADC_Module *adc)
{
	int64_t deadline = k_uptime_get() + N32_ADC_TIMEOUT_MS;

	ADC_StartCalibration(adc);
	while (ADC_GetCalibrationStatus(adc) == SET) {
		if (k_uptime_get() >= deadline) {
			return -ETIMEDOUT;
		}
		k_busy_wait(1);
	}
	return 0;
}

static int adc_n32_wait_ready(ADC_Module *adc)
{
	int64_t deadline = k_uptime_get() + N32_ADC_TIMEOUT_MS;

	while (ADC_GetFlagStatusNew(adc, ADC_FLAG_RDY) == RESET) {
		if (k_uptime_get() >= deadline) {
			return -ETIMEDOUT;
		}
		k_busy_wait(1);
	}
	return 0;
}

static int adc_n32_sample(ADC_Module *adc, uint8_t channel, uint8_t sample_time,
			  uint16_t *sample)
{
	int64_t deadline = k_uptime_get() + N32_ADC_TIMEOUT_MS;

	ADC_ConfigRegularChannel(adc, channel, 1U, sample_time);
	ADC_ClearFlag(adc, ADC_FLAG_ENDC);
	ADC_EnableSoftwareStartConv(adc, ENABLE);
	while (ADC_GetFlagStatus(adc, ADC_FLAG_ENDC) == RESET) {
		if (k_uptime_get() >= deadline) {
			return -ETIMEDOUT;
		}
		k_busy_wait(1);
	}
	/* The N32G457 errata requires at least two ADC clocks before reading DAT. */
	k_busy_wait(1);
	ADC_ClearFlag(adc, ADC_FLAG_ENDC);
	ADC_ClearFlag(adc, ADC_FLAG_STR);
	*sample = ADC_GetDat(adc);
	return 0;
}

static int adc_n32_read(const struct device *dev, const struct adc_sequence *sequence)
{
	const struct adc_n32_config *cfg = dev->config;
	struct adc_n32_data *data = dev->data;
	struct adc_sequence sequence_copy = *sequence;
	struct adc_sequence_options options_copy;
	const struct adc_sequence_options *options = sequence_copy.options;
	uint32_t channels = sequence->channels;
	uint16_t *samples = sequence->buffer;
	uint16_t sample_index = 0U;
	uint8_t count = POPCOUNT(channels);
	int ret = 0;

	if (options != NULL) {
		options_copy = *options;
		sequence_copy.options = &options_copy;
		options = &options_copy;
	}

	if ((channels == 0U) ||
	    ((channels & ~BIT_MASK((uint32_t)cfg->channels + 1U)) != 0U) ||
	    (channels & BIT(0))) {
		return -EINVAL;
	}
	if ((sequence->resolution != 6U) && (sequence->resolution != 8U) &&
	    (sequence->resolution != 10U) && (sequence->resolution != 12U)) {
		return -ENOTSUP;
	}
	if (sequence->oversampling != 0U) {
		return -ENOTSUP;
	}
	if ((samples == NULL) ||
	    (sequence->buffer_size < (size_t)count * sizeof(*samples) *
		((options == NULL) ? 1U : ((size_t)options->extra_samplings + 1U)))) {
		return -ENOMEM;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	if ((channels & data->configured) != channels) {
		ret = -EINVAL;
		goto out;
	}
	if (sequence->calibrate) {
		ret = adc_n32_calibrate(cfg->adc);
		if (ret < 0) {
			goto out;
		}
	}
	ADC_SetConvResultBitNum(cfg->adc, (sequence->resolution - 6U) / 2U);

	for (;;) {
		uint16_t *row = samples + (size_t)sample_index * count;
		uint8_t offset = 0U;

		for (uint8_t channel = 1U; channel <= cfg->channels; channel++) {
			if ((channels & BIT(channel)) == 0U) {
				continue;
			}
			ret = adc_n32_sample(cfg->adc, channel,
						data->sample_time[channel], &row[offset++]);
			if (ret < 0) {
				goto out;
			}
		}

		enum adc_action action = (options != NULL && options->callback != NULL) ?
			options->callback(dev, &sequence_copy, sample_index) : ADC_ACTION_CONTINUE;

		if (action == ADC_ACTION_FINISH || options == NULL ||
		    (action != ADC_ACTION_REPEAT && sample_index >= options->extra_samplings)) {
			break;
		}
		if (action != ADC_ACTION_REPEAT) {
			sample_index++;
		}
		if (options->interval_us != 0U) {
			k_sleep(K_USEC(options->interval_us));
		}
	}

out:
	k_mutex_unlock(&data->lock);
	return ret;
}

#ifdef CONFIG_ADC_ASYNC
static int adc_n32_read_async(const struct device *dev,
			      const struct adc_sequence *sequence,
			      struct k_poll_signal *async)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(sequence);
	ARG_UNUSED(async);
	return -ENOTSUP;
}
#endif

static int adc_n32_init(const struct device *dev)
{
	const struct adc_n32_config *cfg = dev->config;
	struct adc_n32_data *data = dev->data;
	RCC_ClocksType clocks;
	ADC_InitType init;
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

	RCC_GetClocksFreqValue(&clocks);
	ADC_DeInit(cfg->adc);
	ADC_ConfigClk(ADC_CTRL3_CKMOD_AHB,
		      (clocks.HclkFreq > 72000000U) ?
		      RCC_ADCHCLK_DIV2 : RCC_ADCHCLK_DIV1);
	/* The ADC timing clock must be 1 MHz; HSI is 8 MHz on N32G45x. */
	RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSI, RCC_ADC1MCLK_DIV8);
	ADC_InitStruct(&init);
	init.ExtTrigSelect = ADC_EXT_TRIGCONV_NONE;
	ADC_Init(cfg->adc, &init);
	ADC_SetConvResultBitNum(cfg->adc, ADC_RST_BIT_12);
	ADC_Enable(cfg->adc, ENABLE);
	ret = adc_n32_wait_ready(cfg->adc);
	if (ret < 0) {
		return ret;
	}
	ret = adc_n32_calibrate(cfg->adc);
	if (ret < 0) {
		return ret;
	}
	k_mutex_init(&data->lock);
	return 0;
}

static DEVICE_API(adc, adc_n32_api) = {
	.channel_setup = adc_n32_channel_setup,
	.read = adc_n32_read,
#ifdef CONFIG_ADC_ASYNC
	.read_async = adc_n32_read_async,
#endif
};

#define ADC_N32_DEFINE(inst)                                                    \
	PINCTRL_DT_INST_DEFINE(inst);                                            \
	static const struct adc_n32_config adc_n32_config_##inst = {             \
		.adc = (ADC_Module *)DT_INST_REG_ADDR(inst),                       \
		.clock = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),                 \
		.clock_id = DT_INST_CLOCKS_CELL(inst, bits),                      \
		.pinctrl = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                  \
		.channels = DT_INST_PROP(inst, channels),                         \
	};                                                                         \
	static struct adc_n32_data adc_n32_data_##inst;                           \
	DEVICE_DT_INST_DEFINE(inst, adc_n32_init, NULL, &adc_n32_data_##inst,     \
			      &adc_n32_config_##inst, POST_KERNEL,                 \
			      CONFIG_ADC_INIT_PRIORITY, &adc_n32_api);

DT_INST_FOREACH_STATUS_OKAY(ADC_N32_DEFINE)
