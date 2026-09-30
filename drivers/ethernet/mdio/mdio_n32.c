/*
 * Copyright (c) 2026 Nations Technologies
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nsing_n32_mdio

#include <zephyr/device.h>
#include <zephyr/drivers/mdio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/mdio.h>

#include <n32g457_eth.h>
#include <n32g45x_rcc.h>

LOG_MODULE_REGISTER(mdio_n32, CONFIG_MDIO_LOG_LEVEL);

struct mdio_n32_dev_data {
	struct k_mutex mutex;
};

struct mdio_n32_dev_cfg {
	const struct pinctrl_dev_config *pcfg;
};

/*
 * ETH_ReadPHYRegister() and ETH_WritePHYRegister() clear the CSR clock range
 * field of MACMIIADDR, so the divider is programmed before every access.
 */
static void mdio_n32_clock_range_set(void)
{
	RCC_ClocksType clocks;
	uint32_t cr;

	RCC_GetClocksFreqValue(&clocks);

	if (clocks.HclkFreq < 35000000U) {
		cr = ETH_MACMIIADDR_CR_DIV16;
	} else if (clocks.HclkFreq < 60000000U) {
		cr = ETH_MACMIIADDR_CR_DIV26;
	} else if (clocks.HclkFreq <= 100000000U) {
		cr = ETH_MACMIIADDR_CR_DIV42;
	} else {
		cr = ETH_MACMIIADDR_CR_DIV62;
	}

	ETH->MACMIIADDR = (ETH->MACMIIADDR & ~ETH_MACMIIADDR_CR) | cr;
}

static int mdio_n32_read(const struct device *dev, uint8_t prtad, uint8_t regad, uint16_t *data)
{
	struct mdio_n32_dev_data *dev_data = dev->data;
	uint16_t value;
	int ret = 0;

	k_mutex_lock(&dev_data->mutex, K_FOREVER);

	mdio_n32_clock_range_set();
	value = ETH_ReadPHYRegister(prtad, regad);

	/* MB is still set, which means the HAL timed out waiting for the access. */
	if ((ETH->MACMIIADDR & ETH_MACMIIADDR_MB) != 0U) {
		ret = -EIO;
	} else {
		*data = value;
	}

	k_mutex_unlock(&dev_data->mutex);

	return ret;
}

static int mdio_n32_write(const struct device *dev, uint8_t prtad, uint8_t regad, uint16_t data)
{
	struct mdio_n32_dev_data *dev_data = dev->data;
	int ret;

	k_mutex_lock(&dev_data->mutex, K_FOREVER);

	mdio_n32_clock_range_set();
	ret = (ETH_WritePHYRegister(prtad, regad, data) == ETH_SUCCESS) ? 0 : -EIO;

	k_mutex_unlock(&dev_data->mutex);

	return ret;
}

static int mdio_n32_init(const struct device *dev)
{
	struct mdio_n32_dev_data *dev_data = dev->data;
	const struct mdio_n32_dev_cfg *cfg = dev->config;

	k_mutex_init(&dev_data->mutex);

	return pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
}

static DEVICE_API(mdio, mdio_n32_api) = {
	.read = mdio_n32_read,
	.write = mdio_n32_write,
};

#define MDIO_N32_DEVICE(inst)                                                                      \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
                                                                                                   \
	static struct mdio_n32_dev_data mdio_n32_data_##inst;                                      \
                                                                                                   \
	static const struct mdio_n32_dev_cfg mdio_n32_config_##inst = {                            \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                      \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, mdio_n32_init, NULL, &mdio_n32_data_##inst,                    \
			      &mdio_n32_config_##inst, POST_KERNEL, CONFIG_MDIO_INIT_PRIORITY,     \
			      &mdio_n32_api);

DT_INST_FOREACH_STATUS_OKAY(MDIO_N32_DEVICE)
