/*
 * Copyright (c) 2026 Nations Technologies
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/dt-bindings/reset/n32g45x-reset.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>
#include <zephyr/ztest.h>

/* TIM6 is unused by the board and its reset line is on RCC APB1PRST. */
#define N32_TEST_RESET_ID N32_RESET_TIM6
#define N32_TEST_RESET_REG (DT_REG_ADDR(DT_NODELABEL(rcc)) + N32_APB1PRST_OFFSET)
#define N32_TEST_RESET_BIT 4U

static const struct device *const reset_dev = DEVICE_DT_GET(DT_NODELABEL(rctl));

static void *reset_nsing_setup(void)
{
	zassert_true(device_is_ready(reset_dev), "N32 reset controller is not ready");

	/* Leave the peripheral out of reset before and after every test. */
	zassert_ok(reset_line_deassert(reset_dev, N32_TEST_RESET_ID));
	return NULL;
}

static void reset_nsing_after(void *fixture)
{
	ARG_UNUSED(fixture);
	zassert_ok(reset_line_deassert(reset_dev, N32_TEST_RESET_ID));
}

ZTEST_SUITE(reset_nsing, NULL, reset_nsing_setup, NULL, reset_nsing_after, NULL);

ZTEST(reset_nsing, test_assert_deassert_and_status)
{
	uint8_t status;

	zassert_ok(reset_status(reset_dev, N32_TEST_RESET_ID, &status));
	zassert_equal(status, 0U, "TIM6 must initially be out of reset");

	zassert_ok(reset_line_assert(reset_dev, N32_TEST_RESET_ID));
	zassert_ok(reset_status(reset_dev, N32_TEST_RESET_ID, &status));
	zassert_equal(status, 1U, "TIM6 reset bit was not set");
	zassert_true((sys_read32(N32_TEST_RESET_REG) & BIT(N32_TEST_RESET_BIT)) != 0U,
		     "RCC APB1PRST.TIM6RST was not set");

	/* Assert is idempotent. */
	zassert_ok(reset_line_assert(reset_dev, N32_TEST_RESET_ID));
	zassert_ok(reset_status(reset_dev, N32_TEST_RESET_ID, &status));
	zassert_equal(status, 1U, "TIM6 reset bit was unexpectedly cleared");

	zassert_ok(reset_line_deassert(reset_dev, N32_TEST_RESET_ID));
	zassert_ok(reset_status(reset_dev, N32_TEST_RESET_ID, &status));
	zassert_equal(status, 0U, "TIM6 reset bit was not cleared");
	zassert_equal(sys_read32(N32_TEST_RESET_REG) & BIT(N32_TEST_RESET_BIT), 0U,
		      "RCC APB1PRST.TIM6RST was not cleared");
}

ZTEST(reset_nsing, test_toggle_leaves_reset_deasserted)
{
	uint8_t status;

	zassert_ok(reset_line_assert(reset_dev, N32_TEST_RESET_ID));
	zassert_ok(reset_line_toggle(reset_dev, N32_TEST_RESET_ID));
	zassert_ok(reset_status(reset_dev, N32_TEST_RESET_ID, &status));
	zassert_equal(status, 0U, "toggle must leave TIM6 out of reset");
	zassert_equal(sys_read32(N32_TEST_RESET_REG) & BIT(N32_TEST_RESET_BIT), 0U,
		      "toggle must clear RCC APB1PRST.TIM6RST");
}
