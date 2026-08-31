/*
 * Copyright (c) 2024 Nations Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/util_macro.h>

#include <n32_gpio_shared.h>

#ifdef CONFIG_SOC_SERIES_N32G45X

/* Utility macro that expands to the GPIO port register base if the node
 * exists in the devicetree (same pattern as the GD32 pinctrl driver).
 */
#define N32_PORT_ADDR_OR_NONE(nodelabel)                                   \
	COND_CODE_1(DT_NODE_EXISTS(DT_NODELABEL(nodelabel)),               \
		   ((GPIO_Module *)DT_REG_ADDR(DT_NODELABEL(nodelabel)),), ())

/* GPIO port register bases, derived from devicetree */
static GPIO_Module *const n32_gpio_base[] = {
	N32_PORT_ADDR_OR_NONE(gpioa)
	N32_PORT_ADDR_OR_NONE(gpiob)
	N32_PORT_ADDR_OR_NONE(gpioc)
	N32_PORT_ADDR_OR_NONE(gpiod)
	N32_PORT_ADDR_OR_NONE(gpioe)
	N32_PORT_ADDR_OR_NONE(gpiof)
	N32_PORT_ADDR_OR_NONE(gpiog)
};

GPIO_Module *n32_gpioport_get_base(uint8_t port)
{
	if (port >= ARRAY_SIZE(n32_gpio_base)) {
		return NULL;
	}

	return n32_gpio_base[port];
}

int n32_gpioport_configure_pin(GPIO_Module *gpio, uint8_t pin,
			       pinctrl_soc_pin_t conf, bool apply_out_level)
{
	/* CNFMODE field, 4 bits: low 2 = PMODE, high 2 = PCFG (same layout
	 * as the per-pin field in PL_CFG/PH_CFG).
	 */
	const uint32_t cnfmode = FIELD_GET(N32_CNFMODE_Msk, conf);
	/* General purpose output: not input/analog (PMODE != 0) and not
	 * alternate function (PCFG bit 1 == 0).
	 */
	const bool is_gp_output = ((cnfmode & 0x3U) != 0U) &&
				  ((cnfmode & 0x8U) == 0U);

	/* Input pull direction / output initial level via POD (PBSC set,
	 * PBC clear). Applied for pull-up/down inputs and, when requested,
	 * for general purpose outputs. This must happen before the
	 * configuration write below.
	 */
	if ((cnfmode == N32_CNFMODE_INPUT_PUPD) ||
	    (apply_out_level && is_gp_output)) {
		if ((conf & N32_POD_Msk) != 0U) {
			gpio->PBSC = BIT(pin);
		} else {
			gpio->PBC = BIT(pin);
		}
	}

	/* Write the 4-bit CNF/MODE field in PL_CFG (pins 0-7) or PH_CFG
	 * (pins 8-15), leaving the other pins untouched.
	 */
	volatile uint32_t *cfg = (pin <= 7U) ? &gpio->PL_CFG : &gpio->PH_CFG;
	const uint32_t shift = (pin & 7U) << 2U;

	*cfg = (*cfg & ~(0xFU << shift)) | (cnfmode << shift);

	return 0;
}

#endif /* CONFIG_SOC_SERIES_N32G45X */
