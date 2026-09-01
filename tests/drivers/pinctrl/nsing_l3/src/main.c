/*
 * Copyright (c) 2024 Nations Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#include <n32g45x.h>

/*
 * L3 functional pinctrl test: switch PA0 between two DT-defined states
 * through the complete framework path (pinctrl_apply_state -> driver
 * configure_pins), asserting both the register writes and - through the
 * PA0 -> PA2 jumper wire - the real level seen at the pin. Requires the
 * jumper (same fixture as the GPIO L3 suite).
 */
#define TEST_PINCTRL DT_NODELABEL(test_pinctrl)
PINCTRL_DT_DEV_CONFIG_DECLARE(TEST_PINCTRL);
static const struct pinctrl_dev_config *pcfg =
	PINCTRL_DT_DEV_CONFIG_GET(TEST_PINCTRL);

#define GPIOA_DEV DEVICE_DT_GET(DT_NODELABEL(gpioa))
#define OUT_PIN 0U
#define OBS_PIN 2U /* PA2, jumper to PA0 */

ZTEST(pinctrl_l3, test_state_switch)
{
	/* PA2 pull-down input: reads what PA0 drives through the wire
	 * (0 when PA0 is floating, 1 when driven high).
	 */
	zassert_ok(gpio_pin_configure(GPIOA_DEV, OBS_PIN,
				      GPIO_INPUT | GPIO_PULL_DOWN));

	/* "default": PA0 floating input */
	zassert_ok(pinctrl_apply_state(pcfg, PINCTRL_STATE_DEFAULT));
	zassert_equal(GPIOA->PL_CFG & 0xF, 0x4); /* CNFMODE: floating input */
	zassert_equal((GPIOA->POD >> OUT_PIN) & 0x1, 0);
	k_busy_wait(200);
	zassert_equal((GPIOA->PID >> OBS_PIN) & 0x1, 0); /* high-Z -> 0 */

	/* "drive": PA0 push-pull output, initial high (output-high prop).
	 * CNFMODE = PCFG 00 (GP push-pull) | PMODE 01 (2MHz) = 0x1: the
	 * pinctrl driver picks PMODE from the slew-rate property, which
	 * defaults to "2mhz" - not the GPIO driver's 50MHz default.
	 */
	zassert_ok(pinctrl_apply_state(pcfg, PINCTRL_STATE_DRIVE));
	zassert_equal(GPIOA->PL_CFG & 0xF, 0x1); /* CNFMODE: GP PP 2MHz */
	zassert_equal((GPIOA->POD >> OUT_PIN) & 0x1, 1); /* POD = 1 */
	k_busy_wait(200);
	zassert_equal((GPIOA->PID >> OBS_PIN) & 0x1, 1); /* driven high -> 1 */

	/* back to "default": floating again */
	zassert_ok(pinctrl_apply_state(pcfg, PINCTRL_STATE_DEFAULT));
	k_busy_wait(200);
	zassert_equal((GPIOA->PID >> OBS_PIN) & 0x1, 0);
}

ZTEST_SUITE(pinctrl_l3, NULL, NULL, NULL, NULL, NULL);
