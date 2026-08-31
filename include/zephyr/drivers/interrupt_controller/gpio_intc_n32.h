/*
 * Copyright (c) 2024 Nations Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_INTERRUPT_CONTROLLER_GPIO_INTC_N32_H_
#define ZEPHYR_INCLUDE_DRIVERS_INTERRUPT_CONTROLLER_GPIO_INTC_N32_H_

#include <zephyr/drivers/gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

/** EXTI line number (0-15). */
typedef uint8_t n32_gpio_irq_line_t;

/* Interrupt trigger selectors for n32_gpio_intc_select_line_trigger() */
#define N32_GPIO_IRQ_TRIG_RISING  0
#define N32_GPIO_IRQ_TRIG_FALLING 1
#define N32_GPIO_IRQ_TRIG_BOTH    2

/**
 * @brief Select the GPIO port feeding an EXTI line (AFIO_EXTI_CFG).
 *
 * @param line EXTI line number (0-15).
 * @param port GPIO port index (0 = GPIOA).
 * @return 0 on success, -EINVAL if @p line is out of range.
 */
int n32_gpio_intc_set_line_src_port(n32_gpio_irq_line_t line, uint8_t port);

/**
 * @brief Enable an EXTI line.
 *
 * Any pending flag latched while the line was disabled is cleared first,
 * so enabling never fires a spurious interrupt for a stale edge.
 *
 * @param line EXTI line number (0-15). Out-of-range lines are ignored.
 */
void n32_gpio_intc_enable_line(n32_gpio_irq_line_t line);

/**
 * @brief Disable an EXTI line.
 *
 * @param line EXTI line number (0-15). Out-of-range lines are ignored.
 */
void n32_gpio_intc_disable_line(n32_gpio_irq_line_t line);

/**
 * @brief Select the interrupt trigger of an EXTI line.
 *
 * @param line EXTI line number (0-15).
 * @param trig N32_GPIO_IRQ_TRIG_* selector.
 * @return 0 on success, -EINVAL if @p line is out of range or @p trig
 *         is not a valid selector.
 */
int n32_gpio_intc_select_line_trigger(n32_gpio_irq_line_t line, uint8_t trig);

/** EXTI line interrupt callback. */
typedef void (*n32_gpio_irq_cb_t)(gpio_port_pins_t pin, void *user);

/**
 * @brief Register a callback for an EXTI line.
 *
 * Registering the same (@p cb, @p user) pair again replaces the existing
 * registration and returns 0, so the GPIO driver can reconfigure a line
 * without disabling it first.
 *
 * @param line EXTI line number (0-15).
 * @param cb Callback to call when the line triggers.
 * @param user Argument to pass to the callback.
 * @return 0 on success, -EBUSY if the line already has a different
 *         callback, -EINVAL if @p line is out of range.
 */
int n32_gpio_intc_set_irq_callback(n32_gpio_irq_line_t line,
				   n32_gpio_irq_cb_t cb, void *user);

/**
 * @brief Remove the callback of an EXTI line.
 *
 * @param line EXTI line number (0-15). Out-of-range lines are ignored.
 */
void n32_gpio_intc_remove_irq_callback(n32_gpio_irq_line_t line);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_INTERRUPT_CONTROLLER_GPIO_INTC_N32_H_ */
