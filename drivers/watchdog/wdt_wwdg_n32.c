/*
 * Copyright (c) 2026 Nations Technologies
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nsing_n32_wwdg

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/irq.h>
#include <zephyr/sys/clock.h>
#include <zephyr/sys/util.h>
#include <errno.h>
#include <soc.h>

#define N32_WWDG_COUNTER_MIN	0x40U
#define N32_WWDG_COUNTER_MAX	0x7fU
#define N32_WWDG_ENABLE_BIT	BIT(7)
#define N32_WWDG_EWI_BIT	BIT(9)
#define N32_WWDG_EWI_FLAG	BIT(0)
#define N32_WWDG_PRESCALER_SHIFT	7U
#define N32_WWDG_INTERNAL_DIVIDER	4096U

struct n32_wwdg_config {
	const struct device *clock;
	uint32_t clock_id;
};

struct n32_wwdg_data {
	uint8_t counter;
	wdt_callback_t callback;
	bool timeout_installed;
	bool started;
};

static void n32_wwdg_isr(const struct device *dev)
{
	struct n32_wwdg_data *data = dev->data;

	if ((WWDG->STS & N32_WWDG_EWI_FLAG) != 0U) {
		WWDG->STS = 0U;
		if (data->callback != NULL) {
			data->callback(dev, 0);
		}
	}
}

static int n32_wwdg_install_timeout(const struct device *dev,
					    const struct wdt_timeout_cfg *cfg)
{
	const struct n32_wwdg_config *config = dev->config;
	struct n32_wwdg_data *data = dev->data;
	uint32_t pclk;
	uint32_t prescaler;
	uint64_t tick_us;
	uint64_t ticks;
	uint32_t counter = 0U;
	uint32_t window;

	if (data->started) {
		return -EBUSY;
	}
	if (cfg->window.max == 0U ||
	    ((cfg->flags & WDT_FLAG_RESET_MASK) != WDT_FLAG_RESET_SOC)) {
		return -EINVAL;
	}
	if (data->timeout_installed) {
		return -ENOMEM;
	}
	if (clock_control_get_rate(config->clock,
			(clock_control_subsys_t)&config->clock_id, &pclk) != 0 || pclk == 0U) {
		return -EIO;
	}

	for (prescaler = 0U; prescaler <= 3U; prescaler++) {
		tick_us = ((uint64_t)N32_WWDG_INTERNAL_DIVIDER << prescaler) *
			  USEC_PER_SEC / pclk;
		ticks = DIV_ROUND_UP((uint64_t)cfg->window.max * USEC_PER_MSEC,
				     tick_us);
		if (ticks >= 1U && ticks <= 64U) {
			counter = N32_WWDG_COUNTER_MIN + ticks - 1U;
			break;
		}
	}
	if (counter == 0U) {
		return -EINVAL;
	}

	if (cfg->window.min == 0U) {
		window = counter;
	} else {
		ticks = ((uint64_t)cfg->window.min * USEC_PER_MSEC) / tick_us;
		if (ticks >= (counter - N32_WWDG_COUNTER_MIN + 1U)) {
			return -EINVAL;
		}
		window = counter - ticks;
	}

	WWDG->CFG = (prescaler << N32_WWDG_PRESCALER_SHIFT) | window;
	if (cfg->callback != NULL) {
		data->callback = cfg->callback;
	}
	data->counter = counter;
	data->timeout_installed = true;
	return 0;
}

static int n32_wwdg_setup(const struct device *dev, uint8_t options)
{
	struct n32_wwdg_data *data = dev->data;

	if (options != 0U) {
		return -ENOTSUP;
	}
	if (data->started) {
		return -EBUSY;
	}
	if (!data->timeout_installed) {
		return -EINVAL;
	}
	WWDG->STS = 0U;
	NVIC_ClearPendingIRQ(DT_INST_IRQN(0));
	WWDG->CTRL = N32_WWDG_ENABLE_BIT | data->counter;
	if (data->callback != NULL) {
		WWDG->CFG |= N32_WWDG_EWI_BIT;
		irq_enable(DT_INST_IRQN(0));
	}
	data->started = true;
	return 0;
}

static int n32_wwdg_disable(const struct device *dev)
{
	ARG_UNUSED(dev);
	return -EPERM;
}

static int n32_wwdg_feed(const struct device *dev, int channel_id)
{
	struct n32_wwdg_data *data = dev->data;

	if (channel_id != 0) {
		return -EINVAL;
	}
	WWDG->CTRL = data->counter;
	return 0;
}

static void n32_wwdg_irq_config(const struct device *dev)
{
	WWDG->STS = 0U;
	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority), n32_wwdg_isr,
		    DEVICE_DT_INST_GET(0), 0);
	NVIC_ClearPendingIRQ(DT_INST_IRQN(0));
	irq_disable(DT_INST_IRQN(0));
}

static int n32_wwdg_init(const struct device *dev)
{
	const struct n32_wwdg_config *config = dev->config;
	int ret = clock_control_on(config->clock,
			(clock_control_subsys_t)&config->clock_id);

	if (ret != 0) {
		return ret;
	}
	WWDG->CFG &= ~N32_WWDG_EWI_BIT;
	n32_wwdg_irq_config(dev);
	return 0;
}

static DEVICE_API(wdt, n32_wwdg_api) = {
	.setup = n32_wwdg_setup,
	.disable = n32_wwdg_disable,
	.install_timeout = n32_wwdg_install_timeout,
	.feed = n32_wwdg_feed,
};

static struct n32_wwdg_data n32_wwdg_data;
static const struct n32_wwdg_config n32_wwdg_config = {
	.clock = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(0)),
	.clock_id = DT_INST_CLOCKS_CELL(0, bits),
};

DEVICE_DT_INST_DEFINE(0, n32_wwdg_init, NULL, &n32_wwdg_data, &n32_wwdg_config,
		      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &n32_wwdg_api);
