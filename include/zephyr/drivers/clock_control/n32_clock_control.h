/*
 * Copyright (c) 2023, Quincy.W <wangqyfm@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_N32_CLOCK_CONTROL_H_
#define ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_N32_CLOCK_CONTROL_H_

#include <zephyr/drivers/clock_control.h>
#include <zephyr/dt-bindings/clock/n32_clock.h>

#define N32_AHB_PRESCALER	CONFIG_CLOCK_N32_AHB_PRESCALER
#define N32_APB1_PRESCALER	CONFIG_CLOCK_N32_APB1_PRESCALER
#define N32_APB2_PRESCALER	CONFIG_CLOCK_N32_APB2_PRESCALER

struct n32_pclken {
	uint32_t bus;
	uint32_t enr;
};

#endif /* ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_N32_CLOCK_CONTROL_H_ */
