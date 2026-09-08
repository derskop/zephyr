/*
 * Copyright (c) 2026 Nations Technologies
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nsing_n32_iwdg

#include <zephyr/drivers/watchdog.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/clock.h>
#include <zephyr/sys/util.h>
#include <errno.h>
#include <soc.h>

#define N32_IWDG_KEY_ENABLE	0xCCCCU
#define N32_IWDG_KEY_RELOAD	0xAAAAU
#define N32_IWDG_KEY_WRITE_ENABLE	0x5555U
#define N32_IWDG_RELOAD_MAX	0x0fffU
#define N32_IWDG_PRESCALER_MAX	6U
#define N32_IWDG_PVU_FLAG	BIT(0)
#define N32_IWDG_CRVU_FLAG	BIT(1)

struct n32_iwdg_data {
	uint16_t reload;
	uint8_t prescaler;
	bool timeout_installed;
	bool started;
};

static int n32_iwdg_install_timeout(const struct device *dev,
					    const struct wdt_timeout_cfg *cfg)
{
	struct n32_iwdg_data *data = dev->data;
	uint64_t ticks;
	uint32_t divider = 4U;
	uint8_t prescaler = 0U;
	uint32_t reload;
	uint32_t lsi_hz = DT_PROP(DT_NODELABEL(clk_lsi), clock_frequency);

	if (cfg->window.max == 0U) {
		return -EINVAL;
	}
	if ((cfg->callback != NULL) || (cfg->window.min != 0U) ||
	    ((cfg->flags & WDT_FLAG_RESET_MASK) != WDT_FLAG_RESET_SOC)) {
		return -ENOTSUP;
	}
	if (data->started) {
		return -EBUSY;
	}
	if (data->timeout_installed) {
		return -ENOMEM;
	}

	ticks = (uint64_t)cfg->window.max * lsi_hz / MSEC_PER_SEC;
	while ((ticks / divider) > (N32_IWDG_RELOAD_MAX + 1U) &&
	       prescaler < N32_IWDG_PRESCALER_MAX) {
		prescaler++;
		divider <<= 1U;
	}
	if (ticks < divider || (ticks / divider) > (N32_IWDG_RELOAD_MAX + 1U)) {
		return -EINVAL;
	}

	reload = (uint32_t)(ticks / divider) - 1U;
	data->prescaler = prescaler;
	data->reload = reload;
	data->timeout_installed = true;
	return 0;
}

static int n32_iwdg_setup(const struct device *dev, uint8_t options)
{
	struct n32_iwdg_data *data = dev->data;
	uint32_t start;

	if (options != 0U) {
		return -ENOTSUP;
	}
	if (data->started) {
		return -EBUSY;
	}
	if (!data->timeout_installed) {
		return -EINVAL;
	}

	IWDG->KEY = N32_IWDG_KEY_WRITE_ENABLE;
	IWDG->PREDIV = data->prescaler;
	IWDG->RELV = data->reload;
	start = k_uptime_get_32();
	while ((IWDG->STS & (N32_IWDG_PVU_FLAG | N32_IWDG_CRVU_FLAG)) != 0U) {
		if ((k_uptime_get_32() - start) > 100U) {
			return -EIO;
		}
	}
	IWDG->KEY = N32_IWDG_KEY_ENABLE;
	IWDG->KEY = N32_IWDG_KEY_RELOAD;
	data->started = true;
	return 0;
}

static int n32_iwdg_disable(const struct device *dev)
{
	ARG_UNUSED(dev);
	return -EPERM;
}

static int n32_iwdg_feed(const struct device *dev, int channel_id)
{
	ARG_UNUSED(dev);
	if (channel_id != 0) {
		return -EINVAL;
	}
	IWDG->KEY = N32_IWDG_KEY_RELOAD;
	return 0;
}

static DEVICE_API(wdt, n32_iwdg_api) = {
	.setup = n32_iwdg_setup,
	.disable = n32_iwdg_disable,
	.install_timeout = n32_iwdg_install_timeout,
	.feed = n32_iwdg_feed,
};

static struct n32_iwdg_data n32_iwdg_data;

DEVICE_DT_INST_DEFINE(0, NULL, NULL, &n32_iwdg_data, NULL, POST_KERNEL,
		      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &n32_iwdg_api);
