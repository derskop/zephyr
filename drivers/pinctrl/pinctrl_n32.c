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

/* Apply one remap field. reg is N32G45X_RMP_CFG/CFG3/CFG4 or RMP_SPLIT
 * for the SPI1/USART2 fields whose bits are spread over two registers.
 * w is the field width encoding (0 = 1 bit, 1 = 2 bits), so the field
 * mask is (w + 1) bits starting at bit.
 * Clear-then-set: applying a value 0 opcode resets the field to the
 * default mapping.
 */
static int n32g45x_apply_remap(uint8_t reg, uint8_t bit, uint8_t w, uint8_t val)
{
	const uint32_t msk = (BIT((uint32_t)w + 1U) - 1U) << bit;

	/*
	 * RMP_CFG write guard. SW_JTAG_CFG (bits 26:24) is documented as
	 * "read value undefined" in the manual, so a read-modify-write
	 * must never write the undefined read-back back into the SWJ
	 * field: a value such as 100 would disable the SWD/JTAG debug
	 * port. The SDK guards it the same way in GPIO_ConfigPinRemap
	 * (&= DBGAFR_SWJCFG_MASK, then |= ~DBGAFR_SWJCFG_MASK): every
	 * write forces the field to a fixed value. SWJ = 111 is a
	 * no-effect combination per the manual's SW_JTAG_CFG table.
	 * The SDK forces bits 27:24 to 1111; bit 27 is Reserved and
	 * must stay 0 per the manual, so here only the 3 SWJ bits are
	 * forced to 111.
	 */
	const uint32_t swj_clr_mask = 0x00FFFFFFU; /* clear SWJ + Reserved on read */
	const uint32_t swj_nop_val = 0x07000000U; /* SWJ (bits 26:24) = 111: no effect */

	if (reg == N32G45X_RMP_CFG) {
		AFIO->RMP_CFG = (AFIO->RMP_CFG & ~msk & swj_clr_mask) |
				((uint32_t)val << bit) | swj_nop_val;
	} else if (reg == N32G45X_RMP_CFG3) {
		AFIO->RMP_CFG3 = (AFIO->RMP_CFG3 & ~msk) | ((uint32_t)val << bit);
	} else if (reg == N32G45X_RMP_CFG4) {
		AFIO->RMP_CFG4 = (AFIO->RMP_CFG4 & ~msk) | ((uint32_t)val << bit);
	} else if (reg == N32G45X_RMP_SPLIT) {
		if (bit == 0U) {
			/* SPI1: bits split between RMP_CFG bit 0 and RMP_CFG3 bit 18 */
			AFIO->RMP_CFG = (AFIO->RMP_CFG & ~BIT(0) & swj_clr_mask) |
					((uint32_t)(val & 1U) << 0U) | swj_nop_val;
			AFIO->RMP_CFG3 = (AFIO->RMP_CFG3 & ~BIT(18)) |
					 ((uint32_t)((val >> 1) & 1U) << 18U);
		} else if (bit == 3U) {
			/* USART2: bits split between RMP_CFG bit 3 and RMP_CFG3 bit 19 */
			AFIO->RMP_CFG = (AFIO->RMP_CFG & ~BIT(3) & swj_clr_mask) |
					((uint32_t)(val & 1U) << 3U) | swj_nop_val;
			AFIO->RMP_CFG3 = (AFIO->RMP_CFG3 & ~BIT(19)) |
					 ((uint32_t)((val >> 1) & 1U) << 19U);
		} else {
			return -EINVAL;
		}
	} else {
		return -EINVAL;
	}

	return 0;
}

/**
 * Apply the remap configuration of a state.
 *
 * Pins of a state may combine remaps of several peripherals: each distinct
 * (register, bit) field is applied once. Pins sharing a field must agree
 * on its value, otherwise the state is rejected with -EINVAL.
 */
static int n32g45x_pins_remap(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt)
{
	/* One slot per distinct remap field. Far beyond any real board
	 * configuration combines more remap fields in a single state.
	 */
#define N32_REMAP_SLOT_MAX 8U
	uint16_t keys[N32_REMAP_SLOT_MAX];
	uint8_t vals[N32_REMAP_SLOT_MAX];
	uint8_t ws[N32_REMAP_SLOT_MAX];
	uint8_t slots = 0U;
	int ret;

	/* Collect the distinct (register, bit) fields and their values */
	for (uint8_t i = 0U; i < pin_cnt; i++) {
		const uint16_t pin_rmp = FIELD_GET(N32_REMAP_Msk, pins[i]);
		uint8_t j;

		if (pin_rmp == NO_REMAP) {
			continue;
		}

		const uint16_t key = ((uint16_t)N32G45X_RM_REG_GET(pin_rmp) << 8U) |
				     N32G45X_RM_BIT_GET(pin_rmp);
		const uint8_t val = N32G45X_RM_VAL_GET(pin_rmp);
		const uint8_t w = N32G45X_RM_W_GET(pin_rmp);

		for (j = 0U; j < slots; j++) {
			if (keys[j] == key) {
				break;
			}
		}

		if (j < slots) {
			/* Same field: the value (and width) must agree */
			if ((vals[j] != val) || (ws[j] != w)) {
				return -EINVAL;
			}
			continue;
		}

		if (slots >= N32_REMAP_SLOT_MAX) {
			return -EINVAL;
		}

		keys[slots] = key;
		vals[slots] = val;
		ws[slots] = w;
		slots++;
	}

	if (slots == 0U) {
		return 0;
	}

	/* The AFIO clock must be enabled before touching the remap registers. */
	ret = n32_afio_clock_enable();
	if (ret < 0) {
		return ret;
	}

	for (uint8_t i = 0U; i < slots; i++) {
		ret = n32g45x_apply_remap(keys[i] >> 8U, keys[i] & 0xFFU,
					  ws[i], vals[i]);
		if (ret < 0) {
			return ret;
		}
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
