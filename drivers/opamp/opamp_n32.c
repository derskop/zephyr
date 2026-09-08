/*
 * Copyright (c) 2026 Nations Technologies
 * SPDX-License-Identifier: Apache-2.0
 *
 * N32G45x operational amplifier (OPAMP) driver.
 *
 * Four independent OPAMPs share one APB1 peripheral block.  Each OPAMP is
 * exposed as its own Zephyr opamp device, identified by its own devicetree
 * node whose "reg" points at the OPAMPx_CS register block
 * (0x40002000 + x*0x10).
 *
 * N32G45x register support (per UM_N32G45x_OPAMP):
 *  - modes: external (standalone) / internal PGA / internal follower
 *  - internal PGA gains: x2, x4, x8, x16, x32
 *  - the OPAMP outputs feed the ADC internally (OPAMPx -> ADCx)
 */

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/util_macro.h>
#include <zephyr/drivers/opamp.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/n32_clock_control.h>

#include <soc.h>
#include <n32g45x_opamp.h>

LOG_MODULE_REGISTER(opamp_n32, CONFIG_OPAMP_LOG_LEVEL);

#define DT_DRV_COMPAT nsing_n32_opamp

/* OPAMP peripheral block base and stride between two CS registers */
#define N32_OPAMP_BASE_ADDR	0x40002000UL
#define N32_OPAMP_STRIDE	0x10UL

/* Instance index of the device (0..3), derived from the node address. */
#define N32_OPAMP_INST(inst) \
	((DT_INST_REG_ADDR(inst) - N32_OPAMP_BASE_ADDR) / N32_OPAMP_STRIDE)

/*
 * positive-input mapping tables.
 *
 * Row index  : OPAMP instance (0..3).
 * Column     : index of the DT enum value in the "positive-input" list
 *              of nsing,n32-opamp.yaml (must be kept in sync).
 * Cell value : CS.VPSEL mux option index, -1 = not reachable for the
 *              instance.
 */
enum {
	N32_OPAMP_VP_COLS = 10,
	N32_OPAMP_VM_COLS = 8,
};

static const int8_t n32_opamp_vp_map[4][N32_OPAMP_VP_COLS] = {
	/*            PA1  PA3  PA7  PB0  PE8  PC9  PC3  PC5  DAC1 DAC2 */
	/* OPAMP1 */ {  0,   1,   3,  -1,  -1,  -1,  -1,  -1,  -1,   2 },
	/* OPAMP2 */ { -1,  -1,   0,   1,   2,  -1,  -1,  -1,  -1,  -1 },
	/* OPAMP3 */ {  1,  -1,  -1,  -1,  -1,   0,   3,  -1,  -1,   2 },
	/* OPAMP4 */ { -1,  -1,  -1,  -1,  -1,  -1,   0,   2,   1,  -1 },
};

/*
 * negative-input mapping table.
 *
 * Same convention as above but for CS.VMSEL.  Cell value -1 means the
 * input is not reachable for the instance.
 */
static const int8_t n32_opamp_vm_map[4][N32_OPAMP_VM_COLS] = {
	/*            PA2  PA3  PA5  PC4  PB10 PC9  PD8  FLOAT */
	/* OPAMP1 */ {  1,   0,  -1,  -1,  -1,  -1,  -1,    3 },
	/* OPAMP2 */ {  0,  -1,   1,  -1,  -1,  -1,  -1,    3 },
	/* OPAMP3 */ { -1,  -1,  -1,   0,   1,  -1,  -1,    3 },
	/* OPAMP4 */ { -1,  -1,  -1,  -1,   0,   1,   2,    3 },
};

/* Column index of the FLOAT entry (used to force VM floating). */
#define N32_OPAMP_VM_FLOAT_COL	7

struct n32_opamp_config {
	uint8_t idx;			  /* 0..3 (HAL OPAMPX = idx * 4) */
	uint32_t clk_cfg;		  /* N32_CLOCK_OPAMP, from DT */
	uint8_t functional_mode;	  /* enum opamp_functional_mode */
	uint8_t vp_col;			  /* DT enum idx, 0xFF if unset */
	uint8_t vm_col;			  /* DT enum idx, 0xFF if unset */
};

struct n32_opamp_data {
	struct k_mutex mtx;		  /* serialize register accesses */
};

#define N32_OPAMP_COL_OR_NONE(inst, prop)						\
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, prop),					\
		    (DT_INST_ENUM_IDX(inst, prop)), (0xFF))

/* Get the HAL instance selector used by the StdPeriph library. */
static OPAMPX n32_opamp_x(const struct n32_opamp_config *cfg)
{
	return (OPAMPX)(cfg->idx * 4);
}

static int n32_opamp_resolve_input(const struct device *dev)
{
	const struct n32_opamp_config *cfg = dev->config;
	int val;

	if (cfg->vp_col == 0xFF) {
		LOG_ERR("%s: positive-input is required", dev->name);
		return -EINVAL;
	}

	val = n32_opamp_vp_map[cfg->idx][cfg->vp_col];
	if (val < 0) {
		LOG_ERR("%s: positive-input not reachable for OPAMP%d",
			dev->name, cfg->idx + 1);
		return -EINVAL;
	}
	OPAMP_SetVpSel(n32_opamp_x(cfg), (OPAMP_CS_VPSEL)((uint32_t)val << 8));

	if (cfg->functional_mode == OPAMP_FUNCTIONAL_MODE_STANDALONE) {
		/* negative-input is meaningful only in standalone mode */
		if (cfg->vm_col == 0xFF) {
			LOG_ERR("%s: negative-input is required in standalone mode",
				dev->name);
			return -EINVAL;
		}
		val = n32_opamp_vm_map[cfg->idx][cfg->vm_col];
		if (val < 0) {
			LOG_ERR("%s: negative-input not reachable for OPAMP%d",
				dev->name, cfg->idx + 1);
			return -EINVAL;
		}
		OPAMP_SetVmSel(n32_opamp_x(cfg),
			       (OPAMP_CS_VMSEL)((uint32_t)val << 6));
	} else {
		/*
		 * Follower / PGA modes: the inverting input must not be
		 * connected to a pad; the mode bits tie it internally
		 * (follower: to the output, PGA: floating).
		 */
		if (cfg->vm_col != 0xFF) {
			LOG_WRN("%s: negative-input ignored (not standalone mode)",
				dev->name);
		}
		OPAMP_SetVmSel(n32_opamp_x(cfg),
			       (OPAMP_CS_VMSEL)((uint32_t)
				n32_opamp_vm_map[cfg->idx][N32_OPAMP_VM_FLOAT_COL] << 6));
	}

	return 0;
}

static int n32_opamp_set_functional_mode(const struct device *dev)
{
	const struct n32_opamp_config *cfg = dev->config;
	OPAMP_InitType init;

	OPAMP_StructInit(&init);
	init.TimeAutoMuxEn = DISABLE;

	switch (cfg->functional_mode) {
	case OPAMP_FUNCTIONAL_MODE_STANDALONE:
		/* External feedback network defines the gain. */
		init.Mod = OPAMP_CS_EXT_OPAMP;
		break;
	case OPAMP_FUNCTIONAL_MODE_FOLLOWER:
		/* VMSEL is internally tied to the output by hardware. */
		init.Mod = OPAMP_CS_FOLLOW;
		break;
	case OPAMP_FUNCTIONAL_MODE_NON_INVERTING:
		/* Internal resistor feedback network. */
		init.Mod = OPAMP_CS_PGA_EN;
		break;
	default:
		LOG_ERR("%s: functional-mode not supported on N32 (mode %d)",
			dev->name, cfg->functional_mode);
		return -EINVAL;
	}

	OPAMP_Init(n32_opamp_x(cfg), &init);

	return 0;
}

static int n32_opamp_set_gain(const struct device *dev, enum opamp_gain gain)
{
	const struct n32_opamp_config *cfg = dev->config;
	struct n32_opamp_data *data = dev->data;
	OPAMP_CS_PGA_GAIN hal_gain;

	/*
	 * In follower / standalone mode the gain is fixed (x1) or defined by
	 * the external resistor network, so runtime gain changes are no-ops,
	 * as is the case for the other Zephyr opamp drivers.
	 */
	if (cfg->functional_mode == OPAMP_FUNCTIONAL_MODE_FOLLOWER ||
	    cfg->functional_mode == OPAMP_FUNCTIONAL_MODE_STANDALONE) {
		return 0;
	}

	/*
	 * Non-inverting PGA gains available on N32G45x: x2, x4, x8, x16,
	 * x32.  OPAMP_GAIN_1 is folded onto x2 to mirror the gain ratio
	 * semantics used by the STM32 driver (ratio 1 -> x2 non-inverting).
	 */
	switch (gain) {
	case OPAMP_GAIN_1:
	case OPAMP_GAIN_2:
		hal_gain = OPAMP_CS_PGA_GAIN_2;
		break;
	case OPAMP_GAIN_4:
		hal_gain = OPAMP_CS_PGA_GAIN_4;
		break;
	case OPAMP_GAIN_8:
		hal_gain = OPAMP_CS_PGA_GAIN_8;
		break;
	case OPAMP_GAIN_16:
		hal_gain = OPAMP_CS_PGA_GAIN_16;
		break;
	case OPAMP_GAIN_32:
		hal_gain = OPAMP_CS_PGA_GAIN_32;
		break;
	default:
		LOG_ERR("%s: gain %d not available (x2/x4/x8/x16/x32)",
			dev->name, gain);
		return -EINVAL;
	}

	k_mutex_lock(&data->mtx, K_FOREVER);
	OPAMP_SetPgaGain(n32_opamp_x(cfg), hal_gain);
	k_mutex_unlock(&data->mtx);

	return 0;
}

static int n32_opamp_init(const struct device *dev)
{
	const struct n32_opamp_config *cfg = dev->config;
	int ret;

	ret = clock_control_on(DEVICE_DT_GET(DT_NODELABEL(rcc)),
			       (clock_control_subsys_t *)&cfg->clk_cfg);
	if (ret != 0) {
		LOG_ERR("%s: failed to enable OPAMP clock (%d)", dev->name, ret);
		return ret;
	}

	ret = n32_opamp_set_functional_mode(dev);
	if (ret != 0) {
		return ret;
	}

	ret = n32_opamp_resolve_input(dev);
	if (ret != 0) {
		return ret;
	}

	OPAMP_Enable(n32_opamp_x(cfg), ENABLE);

	LOG_INF("%s: OPAMP%d ready", dev->name, cfg->idx + 1);

	return 0;
}

static DEVICE_API(opamp, n32_opamp_api) = {
	.set_gain = n32_opamp_set_gain,
};

#define N32_OPAMP_DEVICE(inst)								\
	static const struct n32_opamp_config n32_opamp_config_##inst = {		\
		.idx = N32_OPAMP_INST(inst),						\
		.clk_cfg = DT_INST_CLOCKS_CELL(inst, bits),				\
		.functional_mode = DT_INST_ENUM_IDX(inst, functional_mode),		\
		.vp_col = N32_OPAMP_COL_OR_NONE(inst, positive_input),			\
		.vm_col = N32_OPAMP_COL_OR_NONE(inst, negative_input),			\
	};										\
											\
	static struct n32_opamp_data n32_opamp_data_##inst = {				\
		.mtx = Z_MUTEX_INITIALIZER(n32_opamp_data_##inst.mtx),			\
	};										\
											\
	DEVICE_DT_INST_DEFINE(inst, n32_opamp_init, NULL,				\
			      &n32_opamp_data_##inst,					\
			      &n32_opamp_config_##inst,				\
			      POST_KERNEL, CONFIG_OPAMP_INIT_PRIORITY,		\
			      &n32_opamp_api);

DT_INST_FOREACH_STATUS_OKAY(N32_OPAMP_DEVICE)
