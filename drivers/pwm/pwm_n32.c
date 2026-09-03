/*
 * Copyright (c) 2026 Nations Technologies
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nsing_n32_pwm

#include <errno.h>

#include <soc.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(pwm_n32, CONFIG_PWM_LOG_LEVEL);

struct pwm_n32_config {
	TIM_Module *timer;
	const struct device *clock;
	uint32_t clock_id;
	uint16_t prescaler;
	uint8_t channels;
	bool advanced;
	const struct pinctrl_dev_config *pcfg;
};

struct pwm_n32_data {
	uint32_t tim_clk;
};

static void (*const set_compare[])(TIM_Module *timer, uint16_t value) = {
	TIM_SetCmp1, TIM_SetCmp2, TIM_SetCmp3, TIM_SetCmp4,
};

static void (*const set_polarity[])(TIM_Module *timer, uint16_t polarity) = {
	TIM_ConfigOc1Polarity, TIM_ConfigOc2Polarity,
	TIM_ConfigOc3Polarity, TIM_ConfigOc4Polarity,
};

static int pwm_n32_set_cycles(const struct device *dev, uint32_t channel,
			      uint32_t period, uint32_t pulse, pwm_flags_t flags)
{
	const struct pwm_n32_config *cfg = dev->config;
	TIM_Module *timer = cfg->timer;

	if ((channel >= cfg->channels) || (channel >= ARRAY_SIZE(set_compare))) {
		return -EINVAL;
	}
	if ((period > UINT16_MAX) || (pulse > period)) {
		return -ENOTSUP;
	}

	if (period == 0U) {
		TIM_EnableCapCmpCh(timer, channel * 4U, TIM_CAP_CMP_DISABLE);
		return 0;
	}

	/* The counter runs from zero through AR, so AR is period - 1. */
	TIM_SetAutoReload(timer, (uint16_t)(period - 1U));
	set_compare[channel](timer, (uint16_t)pulse);
	set_polarity[channel](timer, (flags & PWM_POLARITY_INVERTED) ?
			      TIM_OC_POLARITY_LOW : TIM_OC_POLARITY_HIGH);
	TIM_SelectOcMode(timer, channel * 4U, TIM_OCMODE_PWM1);
	TIM_ConfigArPreload(timer, ENABLE);
	TIM_GenerateEvent(timer, TIM_EVT_SRC_UPDATE);
	TIM_EnableCapCmpCh(timer, channel * 4U, TIM_CAP_CMP_ENABLE);

	return 0;
}

static int pwm_n32_get_cycles_per_sec(const struct device *dev, uint32_t channel,
				       uint64_t *cycles)
{
	const struct pwm_n32_config *cfg = dev->config;
	struct pwm_n32_data *data = dev->data;

	if (channel >= cfg->channels) {
		return -EINVAL;
	}
	*cycles = data->tim_clk / ((uint32_t)cfg->prescaler + 1U);
	return 0;
}

static int pwm_n32_init(const struct device *dev)
{
	const struct pwm_n32_config *cfg = dev->config;
	struct pwm_n32_data *data = dev->data;
	TIM_TimeBaseInitType timebase;
	int err;

	err = clock_control_on(cfg->clock, (clock_control_subsys_t)&cfg->clock_id);
	if (err != 0) {
		return err;
	}
	err = clock_control_get_rate(cfg->clock, (clock_control_subsys_t)&cfg->clock_id,
				     &data->tim_clk);
	if (err != 0) {
		return err;
	}
	if (cfg->pcfg != NULL) {
		err = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
		if (err != 0) {
			return err;
		}
	}

	TIM_InitTimBaseStruct(&timebase);
	timebase.Prescaler = cfg->prescaler;
	timebase.CntMode = TIM_CNT_MODE_UP;
	timebase.Period = UINT16_MAX;
	timebase.ClkDiv = TIM_CLK_DIV1;
	TIM_InitTimeBase(cfg->timer, &timebase);
	if (cfg->advanced) {
		TIM_EnableCtrlPwmOutputs(cfg->timer, ENABLE);
	}
	TIM_Enable(cfg->timer, ENABLE);
	return 0;
}

static DEVICE_API(pwm, pwm_n32_api) = {
	.set_cycles = pwm_n32_set_cycles,
	.get_cycles_per_sec = pwm_n32_get_cycles_per_sec,
};

#define PWM_N32_DEFINE(inst)                                                   \
	PINCTRL_DT_INST_DEFINE(inst);                                            \
	static struct pwm_n32_data pwm_n32_data_##inst;                          \
	static const struct pwm_n32_config pwm_n32_cfg_##inst = {                \
		.timer = (TIM_Module *)DT_REG_ADDR(DT_INST_PARENT(inst)),          \
		.clock = DEVICE_DT_GET(DT_CLOCKS_CTLR(DT_INST_PARENT(inst))),       \
		.clock_id = DT_CLOCKS_CELL(DT_INST_PARENT(inst), bits),             \
		.prescaler = DT_PROP(DT_INST_PARENT(inst), prescaler),              \
		.channels = DT_PROP(DT_INST_PARENT(inst), channels),                \
		.advanced = DT_PROP(DT_INST_PARENT(inst), advanced),                \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                       \
	};                                                                         \
	DEVICE_DT_INST_DEFINE(inst, pwm_n32_init, NULL, &pwm_n32_data_##inst,     \
			      &pwm_n32_cfg_##inst, POST_KERNEL, CONFIG_PWM_INIT_PRIORITY, \
			      &pwm_n32_api);

DT_INST_FOREACH_STATUS_OKAY(PWM_N32_DEFINE)
