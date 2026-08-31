/*
 * Copyright (c) 2024 Nations Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SOC_ARM_NATIONS_COMMON_PINCTRL_SOC_H_
#define ZEPHYR_SOC_ARM_NATIONS_COMMON_PINCTRL_SOC_H_

#include <zephyr/devicetree.h>
#include <zephyr/types.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef CONFIG_SOC_SERIES_N32G45X
#include <zephyr/dt-bindings/pinctrl/n32g45x-pinctrl.h>

/** Type for N32G45x pin. */
typedef uint32_t pinctrl_soc_pin_t;

/*
 * N32G45x pin configuration bit fields:
 *   [4:0]  PORT
 *   [8:5]  PIN
 *   [12:9] CNFMODE: PCFG[1:0] + PMODE[1:0], same layout as the 4-bit
 *          per-pin field in PL_CFG/PH_CFG
 *   [13]   POD: input pull direction (1 = pull-up) or output initial
 *          level (1 = high)
 *   [23:14] REMAP opcode (see N32G45X_REMAP in the pinctrl bindings)
 */
#define N32_PORT_Pos     0
#define N32_PORT_Msk     (0x1F << N32_PORT_Pos)
#define N32_PIN_Pos      5
#define N32_PIN_Msk      (0xF << N32_PIN_Pos)
#define N32_CNFMODE_Pos  9
#define N32_CNFMODE_Msk  (0xF << N32_CNFMODE_Pos)
#define N32_PMODE_Pos    N32_CNFMODE_Pos
#define N32_PMODE_Msk    (0x3 << N32_PMODE_Pos)
#define N32_PCFG_Pos     (N32_CNFMODE_Pos + 2)
#define N32_PCFG_Msk     (0x3 << N32_PCFG_Pos)
#define N32_POD_Pos      13
#define N32_POD_Msk      (0x1 << N32_POD_Pos)
#define N32_REMAP_Pos    14
#define N32_REMAP_Msk    (0x3FF << N32_REMAP_Pos)

/* CNFMODE values (PCFG + PMODE packed) */
#define N32_CNFMODE_ANALOG      0x0U
#define N32_CNFMODE_INPUT_FLOAT 0x4U
#define N32_CNFMODE_INPUT_PUPD  0x8U

/*
 * PMODE values.
 *
 * NOTE - conflicting references:
 *   - AFIO reference manual (CN_UM_N32G45x_AFIO_V0, "IO mode and
 *     configuration table"): PMODE 01 = max 10MHz, 10 = max 2MHz,
 *     11 = max 50MHz (same encoding as STM32F1).
 *   - Nations SDK (n32g45x_gpio.h GPIO_SpeedType): GPIO_Speed_2MHz = 1,
 *     GPIO_Speed_10MHz = 2, GPIO_Speed_50MHz = 3 - the 2MHz/10MHz values
 *     are swapped relative to the manual.
 *   The values below follow the official SDK so the Zephyr layer stays
 *   consistent with the rest of the SDK ecosystem. Verify the actual
 *   slew rate on silicon (e.g. by measuring the output toggle frequency)
 *   before changing these; if the manual turns out to be correct, swap
 *   N32_PMODE_2MHZ and N32_PMODE_10MHZ and the slew-rate lookup below
 *   follows automatically.
 */
#define N32_PMODE_2MHZ  0x1U
#define N32_PMODE_10MHZ 0x2U
#define N32_PMODE_50MHZ 0x3U

/* PCFG values */
#define N32_PCFG_GP_PP  0x0U
#define N32_PCFG_GP_OD  0x4U
#define N32_PCFG_AF_PP  0x8U
#define N32_PCFG_AF_OD  0xCU

/* POD values */
#define N32_POD_0 0x0U
#define N32_POD_1 N32_POD_Msk

/* Carry pinmux PORT/LINE/REMAP fields into the pin configuration. */
#define N32G45X_PMUX2PCFG_PORT_LINE_REMAP(pinmux)                          \
	(((((pinmux) >> N32G45X_PORT_SHIFT) & N32G45X_PORT_MASK)            \
		<< N32_PORT_Pos) |                                           \
	 ((((pinmux) >> N32G45X_LINE_SHIFT) & N32G45X_LINE_MASK)            \
		<< N32_PIN_Pos) |                                            \
	 ((((pinmux) >> N32G45X_RM_SHIFT) & N32G45X_RM_MASK)                \
		<< N32_REMAP_Pos))

/* Input mode: bias-* properties select floating or pull-up/pull-down. */
#define N32G45X_GET_INPUT_CNFMODE_POD(node_id)                             \
	(DT_PROP_OR(node_id, bias_pull_up, 0) ?                             \
		(N32_CNFMODE_INPUT_PUPD | N32_POD_1) :                       \
	 (DT_PROP_OR(node_id, bias_pull_down, 0) ?                          \
		N32_CNFMODE_INPUT_PUPD : N32_CNFMODE_INPUT_FLOAT))

/* Output/alternate modes: slew-rate selects PMODE, drive-open-drain PCFG.
 * Slew-rate enum order comes from nsing,n32g45-pinctrl.yaml:
 * "2mhz" = 0, "10mhz" = 1, "50mhz" = 2.
 */
#define N32G45X_SLEW_RATE_TO_PMODE(idx)                                    \
	((idx) == 0U ? N32_PMODE_2MHZ :                                    \
	 (idx) == 1U ? N32_PMODE_10MHZ : N32_PMODE_50MHZ)

#define N32G45X_GET_OUTPUT_CNFMODE(node_id, is_alt)                        \
	((is_alt ? N32_PCFG_AF_PP : N32_PCFG_GP_PP) |                       \
	 (DT_PROP_OR(node_id, drive_open_drain, 0) ? N32_PCFG_GP_OD : 0U) | \
	 N32G45X_SLEW_RATE_TO_PMODE(DT_ENUM_IDX(node_id, slew_rate)))

/* Output initial level via POD. */
#define N32G45X_GET_POD(node_id)                                           \
	(DT_PROP_OR(node_id, output_high, false) ? N32_POD_1 :              \
	 (DT_PROP_OR(node_id, output_low, false) ? N32_POD_0 : 0U))

/* Assemble the full pin configuration from a DT pin node. */
#define Z_N32G45X_CNFMODE_POD(node_id, mode)                               \
	((mode) == ANALOG ? N32_CNFMODE_ANALOG :                            \
	 (mode) == GPIO_IN ? N32G45X_GET_INPUT_CNFMODE_POD(node_id) :       \
	 N32G45X_GET_OUTPUT_CNFMODE(node_id, ((mode) == ALTERNATE))) |      \
	N32G45X_GET_POD(node_id)

#define Z_PINCTRL_N32G45X_PIN_INIT(node_id)                                \
	(N32G45X_PMUX2PCFG_PORT_LINE_REMAP(DT_PROP(node_id, pinmux)) |      \
	 Z_N32G45X_CNFMODE_POD(node_id,                                    \
		((DT_PROP(node_id, pinmux) >> N32G45X_MODE_SHIFT) &          \
		 N32G45X_MODE_MASK)))

#define Z_PINCTRL_STATE_PIN_INIT(node_id, prop, idx)                       \
	Z_PINCTRL_N32G45X_PIN_INIT(DT_PROP_BY_IDX(node_id, prop, idx)),

#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop)                           \
	{DT_FOREACH_PROP_ELEM(node_id, prop, Z_PINCTRL_STATE_PIN_INIT)}
#endif /* CONFIG_SOC_SERIES_N32G45X */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SOC_ARM_NATIONS_COMMON_PINCTRL_SOC_H_ */
