/*
 * Copyright (c) 2024 Nations Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/pinctrl.h>
#include <zephyr/sys/util.h>
#include <zephyr/ztest.h>

#include <n32g45x.h>

/* Pin configuration for the test device */
#define TEST_DEVICE DT_NODELABEL(test_device)

PINCTRL_DT_DEV_CONFIG_DECLARE(TEST_DEVICE);
static const struct pinctrl_dev_config *pcfg =
	PINCTRL_DT_DEV_CONFIG_GET(TEST_DEVICE);

ZTEST(pinctrl_nsing, test_dt_extract)
{
	const struct pinctrl_state *scfg;
	pinctrl_soc_pin_t pin;

	zassert_equal(pcfg->state_cnt, 1U);

	scfg = &pcfg->states[0];

	zassert_equal(scfg->id, PINCTRL_STATE_DEFAULT);
	zassert_equal(scfg->pin_cnt, 8U);

	/* A0: alternate function, push-pull, 50MHz */
	pin = scfg->pins[0];
	zassert_equal(FIELD_GET(N32_PORT_Msk, pin), 0);
	zassert_equal(FIELD_GET(N32_PIN_Msk, pin), 0);
	zassert_equal(FIELD_GET(N32_CNFMODE_Msk, pin),
		      N32_PCFG_AF_PP | N32_PMODE_50MHZ);
	zassert_equal(FIELD_GET(N32_POD_Msk, pin), 0);
	zassert_equal(FIELD_GET(N32_REMAP_Msk, pin), NO_REMAP);

	/* B1: input with pull-up */
	pin = scfg->pins[1];
	zassert_equal(FIELD_GET(N32_PORT_Msk, pin), 1);
	zassert_equal(FIELD_GET(N32_PIN_Msk, pin), 1);
	zassert_equal(FIELD_GET(N32_CNFMODE_Msk, pin), N32_CNFMODE_INPUT_PUPD);
	zassert_equal(FIELD_GET(N32_POD_Msk, pin), 1);

	/* C2: input with pull-down */
	pin = scfg->pins[2];
	zassert_equal(FIELD_GET(N32_PORT_Msk, pin), 2);
	zassert_equal(FIELD_GET(N32_PIN_Msk, pin), 2);
	zassert_equal(FIELD_GET(N32_CNFMODE_Msk, pin), N32_CNFMODE_INPUT_PUPD);
	zassert_equal(FIELD_GET(N32_POD_Msk, pin), 0);

	/* A3: general purpose output, high level, default 2MHz */
	pin = scfg->pins[3];
	zassert_equal(FIELD_GET(N32_PORT_Msk, pin), 0);
	zassert_equal(FIELD_GET(N32_PIN_Msk, pin), 3);
	zassert_equal(FIELD_GET(N32_CNFMODE_Msk, pin),
		      N32_PCFG_GP_PP | N32_PMODE_2MHZ);
	zassert_equal(FIELD_GET(N32_POD_Msk, pin), 1);

	/* B4: alternate function, open-drain, 10MHz */
	pin = scfg->pins[4];
	zassert_equal(FIELD_GET(N32_PORT_Msk, pin), 1);
	zassert_equal(FIELD_GET(N32_PIN_Msk, pin), 4);
	zassert_equal(FIELD_GET(N32_CNFMODE_Msk, pin),
		      N32_PCFG_AF_OD | N32_PMODE_10MHZ);
	zassert_equal(FIELD_GET(N32_POD_Msk, pin), 0);

	/* C5: analog */
	pin = scfg->pins[5];
	zassert_equal(FIELD_GET(N32_PORT_Msk, pin), 2);
	zassert_equal(FIELD_GET(N32_PIN_Msk, pin), 5);
	zassert_equal(FIELD_GET(N32_CNFMODE_Msk, pin), N32_CNFMODE_ANALOG);
	zassert_equal(FIELD_GET(N32_POD_Msk, pin), 0);

	/* A6: general purpose output, open-drain, low level */
	pin = scfg->pins[6];
	zassert_equal(FIELD_GET(N32_PORT_Msk, pin), 0);
	zassert_equal(FIELD_GET(N32_PIN_Msk, pin), 6);
	zassert_equal(FIELD_GET(N32_CNFMODE_Msk, pin),
		      N32_PCFG_GP_OD | N32_PMODE_2MHZ);
	zassert_equal(FIELD_GET(N32_POD_Msk, pin), 0);

	/* B8: alternate function, open-drain, I2C1 remapped */
	pin = scfg->pins[7];
	zassert_equal(FIELD_GET(N32_PORT_Msk, pin), 1);
	zassert_equal(FIELD_GET(N32_PIN_Msk, pin), 8);
	zassert_equal(FIELD_GET(N32_CNFMODE_Msk, pin),
		      N32_PCFG_AF_OD | N32_PMODE_2MHZ);
	zassert_equal(FIELD_GET(N32_REMAP_Msk, pin), I2C1_REMAP1);
}

ZTEST(pinctrl_nsing, test_configure)
{
	const struct pinctrl_state *scfg = &pcfg->states[0];
	int ret;

	ret = pinctrl_configure_pins(scfg->pins, scfg->pin_cnt, PINCTRL_REG_NONE);
	zassert_ok(ret);

	/* PL_CFG: A0 = 0xB (AF push-pull 50MHz), A3 = 0x1 (GP PP 2MHz),
	 * A6 = 0x5 (GP OD 2MHz)
	 */
	zassert_equal(GPIOA->PL_CFG & 0xF, 0xB);
	zassert_equal((GPIOA->PL_CFG >> 12) & 0xF, 0x1);
	zassert_equal((GPIOA->PL_CFG >> 24) & 0xF, 0x5);

	/* GPIOB->PL_CFG: B1 = 0x8 (input pull), B4 = 0xE (AF OD 10MHz) */
	zassert_equal((GPIOB->PL_CFG >> 4) & 0xF, 0x8);
	zassert_equal((GPIOB->PL_CFG >> 16) & 0xF, 0xE);

	/* GPIOC->PL_CFG: C2 = 0x8 (input pull), C5 = 0x0 (analog) */
	zassert_equal((GPIOC->PL_CFG >> 8) & 0xF, 0x8);
	zassert_equal((GPIOC->PL_CFG >> 20) & 0xF, 0x0);

	/* PH_CFG: B8 = 0xD (AF OD 2MHz) */
	zassert_equal(GPIOB->PH_CFG & 0xF, 0xD);

	/* POD: A3 high, A6 low, B1 pull-up, C2 pull-down */
	zassert_equal((GPIOA->POD >> 3) & 0x1, 1);
	zassert_equal((GPIOA->POD >> 6) & 0x1, 0);
	zassert_equal((GPIOB->POD >> 1) & 0x1, 1);
	zassert_equal((GPIOC->POD >> 2) & 0x1, 0);

	/* I2C1 remap written (RMP_CFG bit 1); restore the register */
	zassert_equal(AFIO->RMP_CFG & BIT(1), BIT(1));
	AFIO->RMP_CFG &= ~BIT(1);
}

ZTEST(pinctrl_nsing, test_remap_conflict)
{
	/* Two pins sharing a remap field with different values must be
	 * rejected: USART3_REMAP1 (value 1) vs USART3_REMAP2 (value 3),
	 * both on RMP_CFG bits 5:4.
	 */
	static const pinctrl_soc_pin_t conflict[2] = {
		N32G45X_PMUX2PCFG_PORT_LINE_REMAP(
			N32G45X_PINMUX('B', 8, ALTERNATE, USART3_REMAP1)) |
			((N32_PCFG_AF_PP | N32_PMODE_2MHZ) << N32_CNFMODE_Pos),
		N32G45X_PMUX2PCFG_PORT_LINE_REMAP(
			N32G45X_PINMUX('B', 9, ALTERNATE, USART3_REMAP2)) |
			((N32_PCFG_AF_PP | N32_PMODE_2MHZ) << N32_CNFMODE_Pos),
	};

	zassert_equal(pinctrl_configure_pins(conflict, ARRAY_SIZE(conflict),
					     PINCTRL_REG_NONE),
		      -EINVAL);
}

ZTEST(pinctrl_nsing, test_remap_multi_field)
{
	/* Different peripherals in one state may remap independently:
	 * I2C1_REMAP1 (RMP_CFG bit 1) and USART1_REMAP1 (RMP_CFG bit 2)
	 * coexist, both fields must end up set.
	 */
	static const pinctrl_soc_pin_t multi[2] = {
		N32G45X_PMUX2PCFG_PORT_LINE_REMAP(
			N32G45X_PINMUX('B', 8, ALTERNATE, I2C1_REMAP1)) |
			((N32_PCFG_AF_OD | N32_PMODE_2MHZ) << N32_CNFMODE_Pos),
		N32G45X_PMUX2PCFG_PORT_LINE_REMAP(
			N32G45X_PINMUX('B', 9, ALTERNATE, USART1_REMAP1)) |
			((N32_PCFG_AF_PP | N32_PMODE_2MHZ) << N32_CNFMODE_Pos),
	};

	zassert_ok(pinctrl_configure_pins(multi, ARRAY_SIZE(multi),
					  PINCTRL_REG_NONE));
	zassert_equal(AFIO->RMP_CFG & BIT(1), BIT(1));
	zassert_equal(AFIO->RMP_CFG & BIT(2), BIT(2));

	/* Restore the register for the following tests */
	AFIO->RMP_CFG &= ~(BIT(1) | BIT(2));
}

ZTEST(pinctrl_nsing, test_remap_paths)
{
	/* RMP_CFG3 path: UART4_REMAP1 -> RMP_CFG3 bits 21:20 = 01 */
	static pinctrl_soc_pin_t uart4[1] = {
		N32G45X_PMUX2PCFG_PORT_LINE_REMAP(
			N32G45X_PINMUX('B', 10, ALTERNATE, UART4_REMAP1)) |
			((N32_PCFG_AF_PP | N32_PMODE_2MHZ) << N32_CNFMODE_Pos),
	};
	/* RMP_CFG4 path: COMP1_REMAP1 -> RMP_CFG4 bits 1:0 = 01 */
	static pinctrl_soc_pin_t comp1[1] = {
		N32G45X_PMUX2PCFG_PORT_LINE_REMAP(
			N32G45X_PINMUX('B', 11, ALTERNATE, COMP1_REMAP1)) |
			((N32_PCFG_AF_PP | N32_PMODE_2MHZ) << N32_CNFMODE_Pos),
	};
	/* Split path: SPI1_REMAP2 -> RMP_CFG bit0 = 0, RMP_CFG3 bit18 = 1 */
	static pinctrl_soc_pin_t spi1[1] = {
		N32G45X_PMUX2PCFG_PORT_LINE_REMAP(
			N32G45X_PINMUX('B', 12, ALTERNATE, SPI1_REMAP2)) |
			((N32_PCFG_AF_PP | N32_PMODE_2MHZ) << N32_CNFMODE_Pos),
	};
	/* Split path: USART2_REMAP1 -> RMP_CFG bit3 = 1, RMP_CFG3 bit19 = 0 */
	static pinctrl_soc_pin_t usart2[1] = {
		N32G45X_PMUX2PCFG_PORT_LINE_REMAP(
			N32G45X_PINMUX('B', 13, ALTERNATE, USART2_REMAP1)) |
			((N32_PCFG_AF_PP | N32_PMODE_2MHZ) << N32_CNFMODE_Pos),
	};

	zassert_ok(pinctrl_configure_pins(uart4, ARRAY_SIZE(uart4),
					  PINCTRL_REG_NONE));
	zassert_equal((AFIO->RMP_CFG3 >> 20) & 0x3, 0x1);

	zassert_ok(pinctrl_configure_pins(comp1, ARRAY_SIZE(comp1),
					  PINCTRL_REG_NONE));
	zassert_equal(AFIO->RMP_CFG4 & 0x3, 0x1);

	zassert_ok(pinctrl_configure_pins(spi1, ARRAY_SIZE(spi1),
					  PINCTRL_REG_NONE));
	zassert_equal(AFIO->RMP_CFG & BIT(0), 0);
	zassert_equal(AFIO->RMP_CFG3 & BIT(18), BIT(18));

	zassert_ok(pinctrl_configure_pins(usart2, ARRAY_SIZE(usart2),
					  PINCTRL_REG_NONE));
	zassert_equal(AFIO->RMP_CFG & BIT(3), BIT(3));
	zassert_equal(AFIO->RMP_CFG3 & BIT(19), 0);

	/* Applying a REMAP0 opcode resets the field (clear-then-set) */
	uart4[0] = N32G45X_PMUX2PCFG_PORT_LINE_REMAP(
		N32G45X_PINMUX('B', 10, ALTERNATE, UART4_REMAP0)) |
		((N32_PCFG_AF_PP | N32_PMODE_2MHZ) << N32_CNFMODE_Pos);
	zassert_ok(pinctrl_configure_pins(uart4, 1, PINCTRL_REG_NONE));
	zassert_equal((AFIO->RMP_CFG3 >> 20) & 0x3, 0x0);

	spi1[0] = N32G45X_PMUX2PCFG_PORT_LINE_REMAP(
		N32G45X_PINMUX('B', 12, ALTERNATE, SPI1_REMAP0)) |
		((N32_PCFG_AF_PP | N32_PMODE_2MHZ) << N32_CNFMODE_Pos);
	zassert_ok(pinctrl_configure_pins(spi1, 1, PINCTRL_REG_NONE));
	zassert_equal(AFIO->RMP_CFG & BIT(0), 0);
	zassert_equal(AFIO->RMP_CFG3 & BIT(18), 0);

	usart2[0] = N32G45X_PMUX2PCFG_PORT_LINE_REMAP(
		N32G45X_PINMUX('B', 13, ALTERNATE, USART2_REMAP0)) |
		((N32_PCFG_AF_PP | N32_PMODE_2MHZ) << N32_CNFMODE_Pos);
	zassert_ok(pinctrl_configure_pins(usart2, 1, PINCTRL_REG_NONE));
	zassert_equal(AFIO->RMP_CFG & BIT(3), 0);
	zassert_equal(AFIO->RMP_CFG3 & BIT(19), 0);

	comp1[0] = N32G45X_PMUX2PCFG_PORT_LINE_REMAP(
		N32G45X_PINMUX('B', 11, ALTERNATE, COMP1_REMAP0)) |
		((N32_PCFG_AF_PP | N32_PMODE_2MHZ) << N32_CNFMODE_Pos);
	zassert_ok(pinctrl_configure_pins(comp1, 1, PINCTRL_REG_NONE));
	zassert_equal(AFIO->RMP_CFG4 & 0x3, 0x0);
}

ZTEST_SUITE(pinctrl_nsing, NULL, NULL, NULL, NULL, NULL);
