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

/*
 * N32G45x follows the STM32 rule: when the APB prescaler is > 1 the timer
 * clocks (TIMx) run at 2x the APB clock.  All other APB peripherals run at
 * the raw PCLK rate.  Identify TIM clock IDs by their APB enable-register bit.
 */
static bool n32_is_timer_clock(uint32_t bits)
{
	uint32_t offset = GET_CLOCK_ID_OFFSET(bits);
	uint32_t bit = GET_CLOCK_ID_BIT(bits);

	if (offset == N32_APB1PCLKEN_OFFSET) {
		/* TIM2..TIM7 occupy APB1 enable bits 0..5 */
		return bit <= 5U;
	}
	if (offset == N32_APB2PCLKEN_OFFSET) {
		/* TIM1 (bit 11) and TIM8 (bit 13) are on APB2 */
		return (bit == 11U) || (bit == 13U);
	}
	return false;
}

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
	uint32_t offset = GET_CLOCK_ID_OFFSET(bits);
	RCC_ClocksType rcc_clocks;

	RCC_GetClocksFreqValue(&rcc_clocks);

	switch (offset) {
	case N32_AHBPCLKEN_OFFSET:
		*rate = rcc_clocks.HclkFreq;
		break;
	case N32_APB2PCLKEN_OFFSET:
		*rate = rcc_clocks.Pclk2Freq;
		if (n32_is_timer_clock(bits) &&
		    (rcc_clocks.HclkFreq > rcc_clocks.Pclk2Freq)) {
			/* APB2 prescaler > 1: timer clock is 2x PCLK2 */
			*rate *= 2U;
		}
		break;
	case N32_APB1PCLKEN_OFFSET:
		*rate = rcc_clocks.Pclk1Freq;
		if (n32_is_timer_clock(bits) &&
		    (rcc_clocks.HclkFreq > rcc_clocks.Pclk1Freq)) {
			/* APB1 prescaler > 1: timer clock is 2x PCLK1 */
			*rate *= 2U;
		}
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
