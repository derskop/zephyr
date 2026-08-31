/*
 * Copyright (c) 2024 Nations Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/drivers/pinctrl.h>
#include <zephyr/sys/util.h>

#include <pinctrl_soc.h>

#ifdef CONFIG_SOC_SERIES_N32G45X
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/n32_clock_control.h>
#include <zephyr/dt-bindings/pinctrl/n32g45x-pinctrl.h>

#include <n32_gpio_shared.h>
#include <n32g45x.h>
#endif /* CONFIG_SOC_SERIES_N32G45X */

#ifdef CONFIG_SOC_SERIES_N32G45X

/* Port clocks are enabled through the clock control driver (same pattern as
 * the GD32 pinctrl driver) instead of pm_device_runtime_get(): N32 GPIO
 * nodes are disabled by default in the SoC dtsi, so pin control must work
 * even when no GPIO device of a port is enabled.
 */
static int n32_port_clock_enable(uint8_t port)
{
	/* N32_CLOCK_GPIOA..G are consecutive (APB2PCLKEN bit 2..8) */
	uint32_t clkid = N32_CLOCK_GPIOA + port;

	return clock_control_on(DEVICE_DT_GET(DT_NODELABEL(rcc)),
				(clock_control_subsys_t)&clkid);
}

static int n32_afio_clock_enable(void)
{
	uint32_t clkid = N32_CLOCK_AFIO;

	return clock_control_on(DEVICE_DT_GET(DT_NODELABEL(rcc)),
				(clock_control_subsys_t)&clkid);
}

/**
 * Apply the remap configuration shared by all pins of a state.
 *
 * All pins of a state must agree on a single remap opcode, otherwise the
 * state is rejected with -EINVAL.
 */
static int n32g45x_pins_remap(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt)
{
	uint16_t remap = NO_REMAP;
	int ret;

	for (uint8_t i = 0U; i < pin_cnt; i++) {
		const uint16_t pin_rmp = FIELD_GET(N32_REMAP_Msk, pins[i]);

		if (remap == NO_REMAP) {
			remap = pin_rmp;
		} else if (pin_rmp == NO_REMAP) {
			continue;
		} else if (pin_rmp != remap) {
			return -EINVAL;
		}
	}

	if (remap == NO_REMAP) {
		return 0;
	}

	const uint8_t val = N32G45X_RM_VAL_GET(remap);
	const uint8_t w = N32G45X_RM_W_GET(remap);
	const uint8_t reg = N32G45X_RM_REG_GET(remap);
	const uint8_t bit = N32G45X_RM_BIT_GET(remap);

	/* The AFIO clock must be enabled before touching the remap registers. */
	ret = n32_afio_clock_enable();
	if (ret < 0) {
		return ret;
	}

	/* Clear-then-set the remap field: applying a REMAP0 opcode resets the
	 * field to the default mapping.
	 */
	if (reg == N32G45X_RMP_CFG) {
		const uint32_t msk = ((uint32_t)w + 1U) << bit;

		AFIO->RMP_CFG = (AFIO->RMP_CFG & ~msk) | ((uint32_t)val << bit);
	} else if (reg == N32G45X_RMP_CFG3) {
		const uint32_t msk = ((uint32_t)w + 1U) << bit;

		AFIO->RMP_CFG3 = (AFIO->RMP_CFG3 & ~msk) | ((uint32_t)val << bit);
	} else if (reg == N32G45X_RMP_CFG4) {
		const uint32_t msk = ((uint32_t)w + 1U) << bit;

		AFIO->RMP_CFG4 = (AFIO->RMP_CFG4 & ~msk) | ((uint32_t)val << bit);
	} else if (bit == 0U) {
		/* SPI1: remap bits split between RMP_CFG bit 0 and RMP_CFG3 bit 18 */
		AFIO->RMP_CFG = (AFIO->RMP_CFG & ~BIT(0)) |
				((uint32_t)(val & 1U) << 0U);
		AFIO->RMP_CFG3 = (AFIO->RMP_CFG3 & ~BIT(18)) |
				 ((uint32_t)((val >> 1) & 1U) << 18U);
	} else if (bit == 3U) {
		/* USART2: remap bits split between RMP_CFG bit 3 and RMP_CFG3 bit 19 */
		AFIO->RMP_CFG = (AFIO->RMP_CFG & ~BIT(3)) |
				((uint32_t)(val & 1U) << 3U);
		AFIO->RMP_CFG3 = (AFIO->RMP_CFG3 & ~BIT(19)) |
				 ((uint32_t)((val >> 1) & 1U) << 19U);
	} else {
		return -EINVAL;
	}

	return 0;
}

static int n32g45x_pin_configure(const pinctrl_soc_pin_t *pin)
{
	const uint32_t line = FIELD_GET(N32_PIN_Msk, *pin);
	const uint32_t port = FIELD_GET(N32_PORT_Msk, *pin);
	GPIO_Module *gpio;
	int ret;

	gpio = n32_gpioport_get_base(port);
	if (gpio == NULL) {
		return -ENODEV;
	}

	ret = n32_port_clock_enable(port);
	if (ret < 0) {
		return ret;
	}

	return n32_gpioport_configure_pin(gpio, line, *pin, true);
}

#endif /* CONFIG_SOC_SERIES_N32G45X */

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt,
			   uintptr_t reg)
{
	ARG_UNUSED(reg);

#ifdef CONFIG_SOC_SERIES_N32G45X
	int ret = n32g45x_pins_remap(pins, pin_cnt);

	if (ret < 0) {
		return ret;
	}

	for (uint8_t i = 0U; i < pin_cnt; i++) {
		ret = n32g45x_pin_configure(&pins[i]);
		if (ret < 0) {
			return ret;
		}
	}
#endif /* CONFIG_SOC_SERIES_N32G45X */

	return 0;
}
