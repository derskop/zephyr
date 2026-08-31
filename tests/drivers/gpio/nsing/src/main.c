/*
 * Copyright (c) 2024 Nations Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#include <n32g45x.h>

/*
 * Test pins are chosen to not conflict with board resources:
 * USART1 on PA9/PA10, I2C1 on PB6/PB7, LEDs on PA8/PB4/PB5, SWD on PA13-15.
 */
#define GPIO_DEV DEVICE_DT_GET(DT_NODELABEL(gpioa))

#define TEST_OUT_PIN  0U
#define TEST_IN_PIN   1U
#define TEST_IRQ_PIN  2U
#define TEST_OD_PIN   3U

static volatile uint32_t irq_count;
static struct gpio_callback irq_cb;

static void irq_cb_handler(const struct device *dev, struct gpio_callback *cb,
			   uint32_t pins)
{
	if (pins & BIT(TEST_IRQ_PIN)) {
		irq_count++;
	}
}

ZTEST(gpio_nsing, test_flags_config)
{
	/* General purpose push-pull output, initial high, 50MHz */
	zassert_ok(gpio_pin_configure(GPIO_DEV, TEST_OUT_PIN,
				      GPIO_OUTPUT | GPIO_OUTPUT_INIT_HIGH));
	zassert_equal(GPIOA->PL_CFG & 0xF, 0x3);
	zassert_equal((GPIOA->POD >> TEST_OUT_PIN) & 0x1, 1);

	/* Initial low */
	zassert_ok(gpio_pin_configure(GPIO_DEV, TEST_OUT_PIN,
				      GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW));
	zassert_equal((GPIOA->POD >> TEST_OUT_PIN) & 0x1, 0);

	/* Open-drain output */
	zassert_ok(gpio_pin_configure(GPIO_DEV, TEST_OD_PIN,
				      GPIO_OUTPUT | GPIO_SINGLE_ENDED |
				      GPIO_LINE_OPEN_DRAIN));
	zassert_equal((GPIOA->PL_CFG >> (TEST_OD_PIN * 4)) & 0xF, 0x5);

	/* Floating input */
	zassert_ok(gpio_pin_configure(GPIO_DEV, TEST_IN_PIN, GPIO_INPUT));
	zassert_equal((GPIOA->PL_CFG >> (TEST_IN_PIN * 4)) & 0xF, 0x4);

	/* Input with pull-up */
	zassert_ok(gpio_pin_configure(GPIO_DEV, TEST_IN_PIN,
				      GPIO_INPUT | GPIO_PULL_UP));
	zassert_equal((GPIOA->PL_CFG >> (TEST_IN_PIN * 4)) & 0xF, 0x8);
	zassert_equal((GPIOA->POD >> TEST_IN_PIN) & 0x1, 1);

	/* Input with pull-down */
	zassert_ok(gpio_pin_configure(GPIO_DEV, TEST_IN_PIN,
				      GPIO_INPUT | GPIO_PULL_DOWN));
	zassert_equal((GPIOA->POD >> TEST_IN_PIN) & 0x1, 0);

	/* Disconnected (high impedance) */
	zassert_ok(gpio_pin_configure(GPIO_DEV, TEST_IN_PIN, GPIO_DISCONNECTED));
	zassert_equal((GPIOA->PL_CFG >> (TEST_IN_PIN * 4)) & 0xF, 0x0);

	/* Invalid flag combinations */
	zassert_equal(gpio_pin_configure(GPIO_DEV, TEST_OUT_PIN,
					 GPIO_INPUT | GPIO_OUTPUT), -ENOTSUP);
	zassert_equal(gpio_pin_configure(GPIO_DEV, TEST_OUT_PIN,
					 GPIO_INPUT | GPIO_PULL_UP |
					 GPIO_PULL_DOWN), -ENOTSUP);
}

ZTEST(gpio_nsing, test_port_io)
{
	zassert_ok(gpio_pin_configure(GPIO_DEV, TEST_OUT_PIN,
				      GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW));

	zassert_ok(gpio_pin_set(GPIO_DEV, TEST_OUT_PIN, 1));
	zassert_equal((GPIOA->POD >> TEST_OUT_PIN) & 0x1, 1);

	zassert_ok(gpio_pin_set(GPIO_DEV, TEST_OUT_PIN, 0));
	zassert_equal((GPIOA->POD >> TEST_OUT_PIN) & 0x1, 0);

	zassert_ok(gpio_pin_toggle(GPIO_DEV, TEST_OUT_PIN));
	zassert_equal((GPIOA->POD >> TEST_OUT_PIN) & 0x1, 1);

	gpio_port_value_t val;

	zassert_ok(gpio_port_get(GPIO_DEV, &val));
	zassert_equal(val & BIT(TEST_OUT_PIN), BIT(TEST_OUT_PIN));
}

ZTEST(gpio_nsing, test_interrupt)
{
	/* Edge-triggered interrupt, fired through EXTI->SWIE so no external
	 * signal is required. Covers the line source selection, trigger
	 * configuration, shared-vector ISR and callback dispatch.
	 */
	zassert_ok(gpio_pin_configure(GPIO_DEV, TEST_IRQ_PIN,
				      GPIO_INPUT | GPIO_PULL_DOWN));
	gpio_init_callback(&irq_cb, irq_cb_handler, BIT(TEST_IRQ_PIN));
	zassert_ok(gpio_add_callback(GPIO_DEV, &irq_cb));

	zassert_ok(gpio_pin_interrupt_configure(GPIO_DEV, TEST_IRQ_PIN,
						GPIO_INT_MODE_EDGE |
						GPIO_INT_TRIG_HIGH));

	EXTI->SWIE |= BIT(TEST_IRQ_PIN);
	k_busy_wait(1000);

	zassert_equal(irq_count, 1);

	zassert_ok(gpio_pin_interrupt_configure(GPIO_DEV, TEST_IRQ_PIN,
						GPIO_INT_MODE_DISABLED));
	zassert_ok(gpio_remove_callback(GPIO_DEV, &irq_cb));
}

ZTEST_SUITE(gpio_nsing, NULL, NULL, NULL, NULL, NULL);
