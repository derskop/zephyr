/*
 * Copyright (c) 2023, Quincy.W <wangqyfm@foxmail.com>
 * Copyright (c) 2024 Nations Technologies
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nsing_n32_rcc

#include <soc.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/n32_clock_control.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#define GET_CLOCK_ID_OFFSET(id) (((id) >> 6U) & 0xFFU)
#define GET_CLOCK_ID_BIT(id)    ((id) & 0x1FU)

struct n32_clock_control_config {
	uint32_t base;
};

static int n32_cc_on(const struct device *dev, clock_control_subsys_t sub_system)
{
	const struct n32_clock_control_config *config = dev->config;
	uint32_t bits = *(uint32_t *)sub_system;
	volatile uint32_t *reg = (volatile uint32_t *)(config->base +
							GET_CLOCK_ID_OFFSET(bits));

	*reg |= BIT(GET_CLOCK_ID_BIT(bits));

	return 0;
}

static int n32_cc_off(const struct device *dev, clock_control_subsys_t sub_system)
{
	const struct n32_clock_control_config *config = dev->config;
	uint32_t bits = *(uint32_t *)sub_system;
	volatile uint32_t *reg = (volatile uint32_t *)(config->base +
							GET_CLOCK_ID_OFFSET(bits));

	*reg &= ~BIT(GET_CLOCK_ID_BIT(bits));

	return 0;
}

static int n32_cc_get_rate(const struct device *dev,
			   clock_control_subsys_t sub_system,
			   uint32_t *rate)
{
	uint32_t bits = *(uint32_t *)sub_system;
	RCC_ClocksType rcc_clocks;

	RCC_GetClocksFreqValue(&rcc_clocks);

	switch (GET_CLOCK_ID_OFFSET(bits)) {
	case N32_AHBPCLKEN_OFFSET:
		*rate = rcc_clocks.HclkFreq;
		break;
	case N32_APB2PCLKEN_OFFSET:
		*rate = rcc_clocks.Pclk2Freq;
		break;
	case N32_APB1PCLKEN_OFFSET:
		*rate = rcc_clocks.Pclk1Freq;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static DEVICE_API(clock_control, n32_cc_api) = {
	.on = n32_cc_on,
	.off = n32_cc_off,
	.get_rate = n32_cc_get_rate,
};

static const struct n32_clock_control_config n32_cc_cfg = {
	.base = DT_INST_REG_ADDR(0),
};

DEVICE_DT_INST_DEFINE(0, NULL, NULL, NULL, &n32_cc_cfg,
		      PRE_KERNEL_1, CONFIG_CLOCK_CONTROL_INIT_PRIORITY,
		      &n32_cc_api);
