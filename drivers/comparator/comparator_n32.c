/*
 * Copyright (c) 2026 Nations Technologies
 * SPDX-License-Identifier: Apache-2.0
 *
 * N32G45x comparator (COMP) driver.
 *
 * Seven independent comparators share one APB1 peripheral block.  Each
 * comparator is exposed as its own Zephyr comparator device, identified
 * by its own devicetree node whose "reg" points at the COMPx_CTRL
 * register block (0x40002410 + (x-1)*0x10).
 *
 * The comparator interrupt lines are shared between groups of
 * comparators: COMP1/2/3 -> IRQ 82, COMP4/5/6 -> IRQ 83, COMP7 -> IRQ 84.
 * Only one comparator of a group can therefore use trigger callbacks at
 * the same time.  The SoC dtsi wires "interrupts" on the first
 * comparator of each group (comp1, comp4, comp7).
 *
 * N32G45x interrupt behaviour (per UM_N32G45x_COMP): with the interrupt
 * enabled the pending flag is set while the (post-polarity) output is
 * high; COMP_INTSTS bits are cleared by writing 0.  There is no
 * configurable falling-edge interrupt, so only
 * COMPARATOR_TRIGGER_RISING_EDGE is reported as supported; falling-edge
 * detection can be approximated externally by inverting one of the
 * inputs.
 */

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/util_macro.h>
#include <zephyr/drivers/comparator.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/n32_clock_control.h>

#include <soc.h>
#include <n32g45x_comp.h>

LOG_MODULE_REGISTER(comp_n32, CONFIG_COMPARATOR_LOG_LEVEL);

#define DT_DRV_COMPAT nsing_n32_comp

/* COMP peripheral block base / stride between two COMPx_CTRL registers */
#define N32_COMP_BASE_ADDR	0x40002400UL
#define N32_COMP_STRIDE		0x10UL

/* Instance index of the device (0..6), derived from the node address. */
#define N32_COMP_INST(inst) \
	((DT_INST_REG_ADDR(inst) - N32_COMP_BASE_ADDR) / N32_COMP_STRIDE - 1)

/*
 * positive-input mapping table.
 *
 * Row index  : comparator instance (0..6).
 * Column     : index of the DT enum value in the "positive-input" list
 *              of nsing,n32-comp.yaml (must be kept in sync).
 * Cell value : COMPx_CTRL.INPSEL mux option index, -1 = not reachable.
 */
enum {
	N32_COMP_VP_COLS = 14,
	N32_COMP_VM_COLS = 16,
	/* Column index of VREF1 / VREF2 in the negative-input list */
	N32_COMP_VREF1_COL = 14,
	N32_COMP_VREF2_COL = 15,
};

static const int8_t n32_comp_vp_map[7][N32_COMP_VP_COLS] = {
	/* COMP1 */ {  0,   1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1 },
	/* COMP2 */ {  0,  -1,   1,   2,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1 },
	/* COMP3 */ { -1,  -1,  -1,  -1,   0,   1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1 },
	/* COMP4 */ { -1,  -1,  -1,  -1,   0,   1,   2,   3,  -1,  -1,  -1,  -1,  -1,  -1 },
	/* COMP5 */ { -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,   0,   1,   2,  -1,  -1,  -1 },
	/* COMP6 */ { -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,   0,   1,  -1,   2,   3,  -1 },
	/* COMP7 */ { -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,   0 },
};

/*
 * negative-input mapping table (same convention as above, for
 * COMPx_CTRL.INMSEL).
 */
static const int8_t n32_comp_vm_map[7][N32_COMP_VM_COLS] = {
	/* COMP1 */ {  0,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,   1,   2,   3,   4 },
	/* COMP2 */ { -1,   0,  -1,  -1,  -1,  -1,  -1,   1,  -1,  -1,  -1,  -1,   2,   3,   4,   5 },
	/* COMP3 */ { -1,  -1,   0,  -1,  -1,  -1,  -1,  -1,   1,  -1,  -1,  -1,   2,   3,   4,   5 },
	/* COMP4 */ { -1,  -1,  -1,   0,  -1,  -1,  -1,  -1,  -1,   1,  -1,  -1,   2,   3,   4,   5 },
	/* COMP5 */ { -1,  -1,  -1,  -1,   0,  -1,  -1,  -1,  -1,  -1,   1,  -1,   2,   3,   4,   5 },
	/* COMP6 */ { -1,  -1,  -1,  -1,  -1,   0,  -1,  -1,  -1,  -1,  -1,   1,   2,   3,   4,   5 },
	/* COMP7 */ { -1,  -1,  -1,  -1,  -1,  -1,   0,  -1,  -1,  -1,  -1,  -1,   1,   2,   3,   4 },
};

/* hysteresis DT enum index -> COMPx_CTRL.HYST register value */
static const uint32_t n32_comp_hyst_values[4] = {
	COMP_CTRL_HYST_NO,
	COMP_CTRL_HYST_LOW,
	COMP_CTRL_HYST_MID,
	COMP_CTRL_HYST_HIGH,
};

struct n32_comp_config {
	uint8_t idx;		   /* 0..6 (COMPX) */
	uint32_t clk_cfg;	   /* N32_CLOCK_COMP, from DT */
	const struct pinctrl_dev_config *pincfg;
	uint8_t vp_col;		   /* DT enum idx, 0xFF if unset */
	uint8_t vm_col;		   /* DT enum idx, 0xFF if unset */
	uint8_t hyst_idx;	   /* DT enum idx (NONE/LOW/MEDIUM/HIGH) */
	bool invert_output;
	bool lock_enable;
	uint8_t vref1_trim;
	uint8_t vref2_trim;
	uint32_t irq_nr;	   /* 0 when no interrupt is wired */
	void (*irq_init)(void);	   /* NULL when no interrupt is wired */
};

struct n32_comp_data {
	comparator_callback_t callback;
	void *user_data;
};

#define N32_COMP_COL_OR_NONE(inst, prop)					\
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, prop),				\
		    (DT_INST_ENUM_IDX(inst, prop)), (0xFF))

#define N32_COMP_ENUM_IDX_OR_DEFAULT(inst, prop)				\
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, prop),				\
		    (DT_INST_ENUM_IDX(inst, prop)), (0))

static COMP_Module *n32_comp_module(void)
{
	return (COMP_Module *)N32_COMP_BASE_ADDR;
}

static int n32_comp_get_output(const struct device *dev)
{
	const struct n32_comp_config *cfg = dev->config;

	return COMP_GetOutStatus((COMPX)cfg->idx) == SET ? 1 : 0;
}

static int n32_comp_set_trigger(const struct device *dev, enum comparator_trigger trigger)
{
	const struct n32_comp_config *cfg = dev->config;
	struct n32_comp_data *data = dev->data;
	COMP_Module *comp = n32_comp_module();
	uint32_t bit = BIT(cfg->idx);

	switch (trigger) {
	case COMPARATOR_TRIGGER_NONE:
		comp->INTEN &= ~bit;
		if (cfg->irq_nr != 0U) {
			irq_disable(cfg->irq_nr);
		}
		return 0;
	case COMPARATOR_TRIGGER_RISING_EDGE:
		/*
		 * The N32 comparator asserts its interrupt while the output
		 * is high (rising condition).  Only one comparator of each
		 * shared IRQ group may enable it at a time.
		 */
		if (cfg->irq_nr == 0U) {
			return -ENOTSUP;
		}
		comp->INTEN |= bit;
		comp->INTSTS &= ~bit;	/* clear any stale pending flag */
		if (data->callback != NULL) {
			irq_enable(cfg->irq_nr);
		}
		return 0;
	case COMPARATOR_TRIGGER_FALLING_EDGE:
	case COMPARATOR_TRIGGER_BOTH_EDGES:
		/* no falling-edge interrupt on N32G45x */
		return -ENOTSUP;
	default:
		return -EINVAL;
	}
}

static int n32_comp_set_trigger_callback(const struct device *dev, comparator_callback_t callback,
					 void *user_data)
{
	const struct n32_comp_config *cfg = dev->config;
	struct n32_comp_data *data = dev->data;
	COMP_Module *comp = n32_comp_module();

	if (callback != NULL && cfg->irq_nr == 0U) {
		return -ENOTSUP;
	}

	data->callback = callback;
	data->user_data = user_data;

	if (cfg->irq_nr != 0U) {
		if (callback != NULL) {
			comp->INTSTS &= ~BIT(cfg->idx);
			irq_enable(cfg->irq_nr);
		} else {
			irq_disable(cfg->irq_nr);
		}
	}

	return 0;
}

static int n32_comp_trigger_is_pending(const struct device *dev)
{
	const struct n32_comp_config *cfg = dev->config;
	COMP_Module *comp = n32_comp_module();

	if (COMP_GetIntStsOneComp((COMPX)cfg->idx) == SET) {
		/* clear by writing 0 */
		comp->INTSTS &= ~BIT(cfg->idx);
		return 1;
	}

	return 0;
}

static void n32_comp_isr(const struct device *dev)
{
	const struct n32_comp_config *cfg = dev->config;
	struct n32_comp_data *data = dev->data;
	COMP_Module *comp = n32_comp_module();

	if (COMP_GetIntStsOneComp((COMPX)cfg->idx) == RESET) {
		return;
	}

	comp->INTSTS &= ~BIT(cfg->idx);

	if (data->callback != NULL) {
		data->callback(dev, data->user_data);
	}
}

static int n32_comp_vref_cfg(const struct device *dev)
{
	const struct n32_comp_config *cfg = dev->config;
	COMP_Module *comp = n32_comp_module();
	uint32_t tmp = comp->VREFSCL;

	if (cfg->vm_col == N32_COMP_VREF1_COL) {
		tmp &= ~(COMP_VREFSCL_VV1TRM_MSK | COMP_VREFSCL_VV1EN_MSK);
		tmp |= ((uint32_t)cfg->vref1_trim << 1) | COMP_VREFSCL_VV1EN_MSK;
	} else if (cfg->vm_col == N32_COMP_VREF2_COL) {
		tmp &= ~(COMP_VREFSCL_VV2TRM_MSK | COMP_VREFSCL_VV2EN_MSK);
		tmp |= ((uint32_t)cfg->vref2_trim << 8) | COMP_VREFSCL_VV2EN_MSK;
	}

	comp->VREFSCL = tmp;

	return 0;
}

static int n32_comp_init(const struct device *dev)
{
	const struct n32_comp_config *cfg = dev->config;
	COMP_InitType init;
	int ret;

	ret = clock_control_on(DEVICE_DT_GET(DT_NODELABEL(rcc)),
			       (clock_control_subsys_t *)&cfg->clk_cfg);
	if (ret != 0) {
		LOG_ERR("%s: failed to enable COMP clock (%d)", dev->name, ret);
		return ret;
	}

	ret = pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0 && ret != -ENOENT) {
		LOG_ERR("%s: pinctrl setup failed (%d)", dev->name, ret);
		return ret;
	}

	if (cfg->vp_col == 0xFF || cfg->vm_col == 0xFF) {
		LOG_ERR("%s: positive-input and negative-input are required",
			dev->name);
		return -EINVAL;
	}

	if (n32_comp_vp_map[cfg->idx][cfg->vp_col] < 0) {
		LOG_ERR("%s: positive-input not reachable for COMP%d",
			dev->name, cfg->idx + 1);
		return -EINVAL;
	}

	if (n32_comp_vm_map[cfg->idx][cfg->vm_col] < 0) {
		LOG_ERR("%s: negative-input not reachable for COMP%d",
			dev->name, cfg->idx + 1);
		return -EINVAL;
	}

	/* Reference scalers used by the comparator must be enabled first. */
	ret = n32_comp_vref_cfg(dev);
	if (ret != 0) {
		return ret;
	}

	COMP_StructInit(&init);
	init.Blking = COMP_CTRL_BLKING_NO;
	init.Hyst = (COMP_CTRL_HYST)n32_comp_hyst_values[cfg->hyst_idx];
	init.PolRev = cfg->invert_output;
	init.OutSel = COMPX_CTRL_OUTSEL_NC;
	init.InpSel = (COMP_CTRL_INPSEL)((uint32_t)n32_comp_vp_map[cfg->idx][cfg->vp_col] << 4);
	init.InmSel = (COMP_CTRL_INMSEL)((uint32_t)n32_comp_vm_map[cfg->idx][cfg->vm_col] << 1);
	init.FilterEn = false;
	init.En = true;

	COMP_Init((COMPX)cfg->idx, &init);

	if (cfg->lock_enable) {
		n32_comp_module()->LOCK |= BIT(cfg->idx);
	}

	if (cfg->irq_init != NULL) {
		cfg->irq_init();
	}

	LOG_INF("%s: COMP%d ready", dev->name, cfg->idx + 1);

	return 0;
}

static DEVICE_API(comparator, n32_comp_api) = {
	.get_output = n32_comp_get_output,
	.set_trigger = n32_comp_set_trigger,
	.set_trigger_callback = n32_comp_set_trigger_callback,
	.trigger_is_pending = n32_comp_trigger_is_pending,
};

#define N32_COMP_IRQ_FN(inst)	n32_comp_irq_init_##inst

#define N32_COMP_IRQ_DEFINE(inst)						\
	static void N32_COMP_IRQ_FN(inst)(void)					\
	{									\
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority),	\
			    n32_comp_isr, DEVICE_DT_INST_GET(inst), 0);		\
	}

#define N32_COMP_IRQ_NR(inst)							\
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, interrupts),			\
		    (DT_INST_IRQN(inst)), (0))

#define N32_COMP_DEVICE(inst)							\
	PINCTRL_DT_INST_DEFINE(inst);						\
											\
	IF_ENABLED(DT_INST_NODE_HAS_PROP(inst, interrupts),			\
		   (N32_COMP_IRQ_DEFINE(inst)))					\
											\
	static struct n32_comp_data n32_comp_data_##inst;			\
											\
	static const struct n32_comp_config n32_comp_config_##inst = {		\
		.idx = N32_COMP_INST(inst),					\
		.clk_cfg = DT_INST_CLOCKS_CELL(inst, bits),			\
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),			\
		.vp_col = N32_COMP_COL_OR_NONE(inst, positive_input),		\
		.vm_col = N32_COMP_COL_OR_NONE(inst, negative_input),		\
		.hyst_idx = N32_COMP_ENUM_IDX_OR_DEFAULT(inst, hysteresis),	\
		.invert_output =							\
			N32_COMP_ENUM_IDX_OR_DEFAULT(inst, invert_output) != 0,	\
		.lock_enable = DT_INST_PROP_OR(inst, lock_enable, false),	\
		.vref1_trim = DT_INST_PROP_OR(inst, n32_vref1_trim, 0),		\
		.vref2_trim = DT_INST_PROP_OR(inst, n32_vref2_trim, 0),		\
		.irq_nr = N32_COMP_IRQ_NR(inst),				\
		.irq_init = COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, interrupts), \
					(N32_COMP_IRQ_FN(inst)), (NULL)),	\
	};									\
											\
	DEVICE_DT_INST_DEFINE(inst, n32_comp_init, NULL,			\
			      &n32_comp_data_##inst,				\
			      &n32_comp_config_##inst,				\
			      POST_KERNEL, CONFIG_COMPARATOR_INIT_PRIORITY,	\
			      &n32_comp_api);

DT_INST_FOREACH_STATUS_OKAY(N32_COMP_DEVICE)
