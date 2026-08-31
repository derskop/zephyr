/*
 * Copyright (c) 2024 Nations Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SOC_ARM_NATIONS_COMMON_N32_GPIO_SHARED_H_
#define ZEPHYR_SOC_ARM_NATIONS_COMMON_N32_GPIO_SHARED_H_

#include <zephyr/types.h>

#include <pinctrl_soc.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef CONFIG_SOC_SERIES_N32G45X
#include <n32g45x.h>

/**
 * @brief Get the register base of a GPIO port.
 *
 * @param port Port index (0 = GPIOA).
 * @return Register base, or NULL if the port does not exist in devicetree.
 */
GPIO_Module *n32_gpioport_get_base(uint8_t port);

/**
 * @brief Configure a single GPIO pin.
 *
 * Writes the 4-bit CNF/MODE field in PL_CFG (pins 0-7) or PH_CFG (pins
 * 8-15), and applies the POD bit (input pull direction / output level)
 * for pull-up/down inputs and, when @p apply_out_level is set, for
 * general purpose outputs.
 *
 * This function is shared by the GPIO and pinctrl drivers.
 *
 * @param gpio GPIO port register base.
 * @param pin Pin number 0-15.
 * @param conf Pin configuration, see @ref pinctrl_soc_pin_t.
 * @param apply_out_level Apply the POD bit for general purpose outputs.
 * @return 0 on success, negative errno otherwise.
 */
int n32_gpioport_configure_pin(GPIO_Module *gpio, uint8_t pin,
			       pinctrl_soc_pin_t conf, bool apply_out_level);
#endif /* CONFIG_SOC_SERIES_N32G45X */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SOC_ARM_NATIONS_COMMON_N32_GPIO_SHARED_H_ */
