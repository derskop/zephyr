/*
 * Copyright (c) 2026 Nations Technologies
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nsing_n32_rcc_rctl

#include <zephyr/arch/common/sys_bitops.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/reset.h>

#define N32_RESET_OFFSET(id) (((id) >> 6U) & 0xFFU)
#define N32_RESET_BIT(id)    ((id) & 0x1FU)

struct reset_n32_config {
	mem_addr_t base;
};

static int reset_n32_status(const struct device *dev, uint32_t id, uint8_t *status)
{
	const struct reset_n32_config *config = dev->config;

	*status = !!sys_test_bit(config->base + N32_RESET_OFFSET(id), N32_RESET_BIT(id));
	return 0;
}

static int reset_n32_line_assert(const struct device *dev, uint32_t id)
{
	const struct reset_n32_config *config = dev->config;

	sys_set_bit(config->base + N32_RESET_OFFSET(id), N32_RESET_BIT(id));
	return 0;
}

static int reset_n32_line_deassert(const struct device *dev, uint32_t id)
{
	const struct reset_n32_config *config = dev->config;

	sys_clear_bit(config->base + N32_RESET_OFFSET(id), N32_RESET_BIT(id));
	return 0;
}

static int reset_n32_line_toggle(const struct device *dev, uint32_t id)
{
	(void)reset_n32_line_assert(dev, id);
	(void)reset_n32_line_deassert(dev, id);

	return 0;
}

static DEVICE_API(reset, reset_n32_driver_api) = {
	.status = reset_n32_status,
	.line_assert = reset_n32_line_assert,
	.line_deassert = reset_n32_line_deassert,
	.line_toggle = reset_n32_line_toggle,
};

static const struct reset_n32_config reset_n32_config = {
	.base = DT_REG_ADDR(DT_INST_PARENT(0)),
};

DEVICE_DT_INST_DEFINE(0, NULL, NULL, NULL, &reset_n32_config, PRE_KERNEL_1,
		      CONFIG_RESET_INIT_PRIORITY, &reset_n32_driver_api);
