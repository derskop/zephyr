/*
 * Copyright (c) 2026 Nations Technologies
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/dac.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

static const uint16_t dac_values[] = {0, 1024, 2048, 3072, 4095};

int main(void)
{
	const struct device *adc = DEVICE_DT_GET(DT_NODELABEL(adc1));
	const struct device *dac = DEVICE_DT_GET(DT_NODELABEL(dac1));
	struct adc_channel_cfg adc_cfg = {
		.channel_id = 1,
		.gain = ADC_GAIN_1,
		.reference = ADC_REF_INTERNAL,
		.acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_TICKS, 240),
	};
	struct dac_channel_cfg dac_cfg = {
		.channel_id = 1,
		.resolution = 12,
		.buffered = true,
	};
	uint16_t sample;
	struct adc_sequence sequence = {
		.channels = BIT(1),
		.buffer = &sample,
		.buffer_size = sizeof(sample),
		.resolution = 12,
	};
	int ret;

	if (!device_is_ready(adc) || !device_is_ready(dac)) {
		printk("ADC or DAC device is not ready\n");
		return 0;
	}
	ret = adc_channel_setup(adc, &adc_cfg);
	if (ret == 0) {
		ret = dac_channel_setup(dac, &dac_cfg);
	}
	if (ret != 0) {
		printk("Channel setup failed: %d\n", ret);
		return 0;
	}

	while (true) {
		for (size_t i = 0; i < ARRAY_SIZE(dac_values); i++) {
			ret = dac_write_value(dac, 1, dac_values[i]);
			if (ret != 0) {
				printk("DAC write failed: %d\n", ret);
				return 0;
			}
			k_sleep(K_MSEC(20));
			ret = adc_read(adc, &sequence);
			if (ret != 0) {
				printk("ADC read failed: %d\n", ret);
				return 0;
			}
			printk("DAC1: %u, ADC1 channel 1: %u, delta: %d\n",
			       dac_values[i], sample, (int)sample - (int)dac_values[i]);
			k_sleep(K_SECONDS(1));
		}
	}
}
