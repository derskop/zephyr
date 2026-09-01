/*
 * Copyright (c) 2024 Nations Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/interrupt_controller/gpio_intc_n32.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#include <n32g45x.h>

/*
 * L3 functional GPIO tests. Same pin choices as the L2 suite (no
 * conflicts with USART1 PA9/PA10, I2C1 PB6/PB7, LEDs PA8/PB4/PB5,
 * SWD PA13-15).
 *
 * test_real_edge_count: real edges driven from PA0 (output) and received
 * on PA2 (interrupt input) - requires a jumper wire between the two.
 * test_cross_port_line_source: fully automatic (SWIE + AFIO register
 * readback), no wiring.
 */
#define GPIOA_DEV DEVICE_DT_GET(DT_NODELABEL(gpioa))
#define GPIOB_DEV DEVICE_DT_GET(DT_NODELABEL(gpiob))

#define OUT_PIN  0U
#define IRQ_PIN  2U
#define IRQ_LINE 2U /* EXTI line number == pin number */

static volatile uint32_t irq_count_a;
static volatile uint32_t irq_count_b;
static struct gpio_callback cb_a;
static struct gpio_callback cb_b;

static void irq_cb_a(const struct device *dev, struct gpio_callback *cb,
		     uint32_t pins)
{
	if (pins & BIT(IRQ_PIN)) {
		irq_count_a++;
	}
}

static void irq_cb_b(const struct device *dev, struct gpio_callback *cb,
		     uint32_t pins)
{
	if (pins & BIT(IRQ_PIN)) {
		irq_count_b++;
	}
}

/* Busy-wait until *count reaches target (max ~500ms). Serializing on the
 * count is mandatory: the EXTI pending bit is a single latch - an edge
 * arriving while it is still set (ISR not yet run) would be lost.
 */
static bool wait_count(volatile uint32_t *count, uint32_t target)
{
	for (int i = 0; i < 5000 && *count < target; i++) {
		k_busy_wait(100);
	}

	return *count == target;
}

/*
 * Drive `edges` counted edges on PA0 and verify every one of them is
 * delivered to the PA2 callback. `start_high` picks the initial level;
 * each iteration toggles the output, so with a RISING trigger only the
 * low->high transitions count, with FALLING only high->low, and with
 * BOTH every toggle (100 edges = 50 full cycles).
 */
static void run_edge_phase(enum gpio_int_trig trig, bool start_high,
			   uint32_t edges)
{
	bool level = start_high;
	uint32_t fired = 0;

	zassert_ok(gpio_pin_set(GPIOA_DEV, OUT_PIN, start_high));
	k_busy_wait(100); /* settle before arming the trigger */

	irq_count_a = 0;
	zassert_ok(gpio_pin_interrupt_configure(GPIOA_DEV, IRQ_PIN,
					GPIO_INT_MODE_EDGE | trig));
	/* The line was re-armed above, which clears any stale pending
	 * bit, so the count can only grow from the edges we drive.
	 */
	/* Toggle the output until `edges` *matching* edges have been seen.
	 * With a RISING/FALLING trigger only every other toggle produces a
	 * matching edge, so non-matching toggles are not waited on.
	 */
	for (uint32_t i = 0; fired < edges; i++) {
		bool prev = level;

		level = !level;
		zassert_ok(gpio_pin_set(GPIOA_DEV, OUT_PIN, level));

		if ((trig == GPIO_INT_TRIG_BOTH) ||
		    (trig == GPIO_INT_TRIG_HIGH && !prev && level) ||
		    (trig == GPIO_INT_TRIG_LOW && prev && !level)) {
			zassert_true(wait_count(&irq_count_a, ++fired));
		}
	}

	zassert_equal(irq_count_a, edges);
}

ZTEST(gpio_nsing_l3, test_real_edge_count)
{
	/* Requires jumper wire: PA0 (output) -> PA2 (interrupt input). */

	/* test_cross_port_line_source (alphabetically earlier) ends with
	 * EXTI line 2's AFIO source on port B; restore it to port A (0)
	 * so the real edges from PA0 reach this line.
	 */
	zassert_ok(n32_gpio_intc_set_line_src_port(IRQ_LINE, 0));

	zassert_ok(gpio_pin_configure(GPIOA_DEV, OUT_PIN,
				      GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW));
	zassert_ok(gpio_pin_configure(GPIOA_DEV, IRQ_PIN,
				      GPIO_INPUT | GPIO_PULL_DOWN));
	gpio_init_callback(&cb_a, irq_cb_a, BIT(IRQ_PIN));
	zassert_ok(gpio_add_callback(GPIOA_DEV, &cb_a));

	run_edge_phase(GPIO_INT_TRIG_HIGH, false, 100); /* rising x100 */
	run_edge_phase(GPIO_INT_TRIG_LOW, true, 100);   /* falling x100 */
	run_edge_phase(GPIO_INT_TRIG_BOTH, true, 100);  /* 100 edges, 50 cycles */

	zassert_ok(gpio_pin_interrupt_configure(GPIOA_DEV, IRQ_PIN,
						GPIO_INT_MODE_DISABLED));
	zassert_ok(gpio_remove_callback(GPIOA_DEV, &cb_a));
}

ZTEST(gpio_nsing_l3, test_cross_port_line_source)
{
	/* EXTI line 2 -> port A (PA2) */
	zassert_ok(gpio_pin_configure(GPIOA_DEV, IRQ_PIN,
				      GPIO_INPUT | GPIO_PULL_DOWN));
	gpio_init_callback(&cb_a, irq_cb_a, BIT(IRQ_PIN));
	zassert_ok(gpio_add_callback(GPIOA_DEV, &cb_a));
	zassert_ok(gpio_pin_interrupt_configure(GPIOA_DEV, IRQ_PIN,
						GPIO_INT_MODE_EDGE |
						GPIO_INT_TRIG_HIGH));
	/* 4-bit line source field of EXTI line 2 lives in EXTI_CFG[0]
	 * bits 8-11: 0 = port A.
	 */
	zassert_equal((AFIO->EXTI_CFG[0] >> 8) & 0xF, 0);
	irq_count_a = 0;
	EXTI->SWIE |= BIT(IRQ_LINE);
	k_busy_wait(1000);
	zassert_equal(irq_count_a, 1);
	zassert_ok(gpio_pin_interrupt_configure(GPIOA_DEV, IRQ_PIN,
						GPIO_INT_MODE_DISABLED));
	zassert_ok(gpio_remove_callback(GPIOA_DEV, &cb_a));

	/* EXTI line 2 -> port B (PB2): the AFIO source field is rewritten
	 * and the SWIE-triggered dispatch must follow the new device.
	 * PB2 is free on this board (LEDs PB4/PB5, I2C1 PB6/PB7).
	 */
	zassert_ok(gpio_pin_configure(GPIOB_DEV, IRQ_PIN,
				      GPIO_INPUT | GPIO_PULL_DOWN));
	gpio_init_callback(&cb_b, irq_cb_b, BIT(IRQ_PIN));
	zassert_ok(gpio_add_callback(GPIOB_DEV, &cb_b));
	zassert_ok(gpio_pin_interrupt_configure(GPIOB_DEV, IRQ_PIN,
						GPIO_INT_MODE_EDGE |
						GPIO_INT_TRIG_HIGH));
	zassert_equal((AFIO->EXTI_CFG[0] >> 8) & 0xF, 1); /* port B */
	irq_count_b = 0;
	EXTI->SWIE |= BIT(IRQ_LINE);
	k_busy_wait(1000);
	zassert_equal(irq_count_b, 1);
	zassert_equal(irq_count_a, 1); /* not re-delivered to port A */

	zassert_ok(gpio_pin_interrupt_configure(GPIOB_DEV, IRQ_PIN,
						GPIO_INT_MODE_DISABLED));
	zassert_ok(gpio_remove_callback(GPIOB_DEV, &cb_b));
}

ZTEST_SUITE(gpio_nsing_l3, NULL, NULL, NULL, NULL, NULL);
