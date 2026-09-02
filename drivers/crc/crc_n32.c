/*
 * Copyright (c) 2024 Nations Technologies
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nsing_n32_crc

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/n32_clock_control.h>
#include <zephyr/drivers/crc.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/crc.h>

#include <n32g45x_crc.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(crc_n32, CONFIG_CRC_LOG_LEVEL);

struct crc_n32_config {
	uint32_t apb_clk_en;
};

struct crc_n32_data {
	struct k_sem dev_lock;
};

static void crc_n32_release(const struct device *dev, struct crc_ctx *ctx)
{
	struct crc_n32_data *data = dev->data;

	ctx->state = CRC_STATE_IDLE;
	k_sem_give(&data->dev_lock);
}

/*
 * CRC API implementation
 */
static int crc_n32_begin(const struct device *dev, struct crc_ctx *ctx)
{
	struct crc_n32_data *data = dev->data;

	/*
	 * The N32G45 CRC unit implements a single fixed algorithm:
	 * CRC-32/MPEG-2 (poly 0x04C11DB7, no input/output reflection,
	 * init 0xFFFFFFFF, no final XOR). Any other configuration
	 * cannot be realized in hardware.
	 */
	if ((ctx->type != CRC32_MPEG2) ||
	    (ctx->polynomial != 0x04C11DB7U) ||
	    (ctx->seed != CRC32_MPEG2_INIT_VAL) ||
	    (ctx->reversed != 0)) {
		return -ENOTSUP;
	}

	/* Take ownership of the CRC peripheral */
	k_sem_take(&data->dev_lock, K_FOREVER);

	/* Reset CRC unit (loads the 0xFFFFFFFF initial value) */
	CRC32_ResetCrc();

	ctx->state = CRC_STATE_IN_PROGRESS;

	return 0;
}

static int crc_n32_update(const struct device *dev, struct crc_ctx *ctx,
			  const void *buffer, size_t bufsize)
{
	const uint8_t *buf = buffer;
	size_t i;

	if (ctx->state != CRC_STATE_IN_PROGRESS) {
		return -EINVAL;
	}

	/*
	 * Feed data 32 bits at a time. The first byte of each 4-byte
	 * chunk goes into the most significant byte of the word (the
	 * hardware processes the word MSB-first, non-reflected).
	 */
	for (i = 0; i + sizeof(uint32_t) <= bufsize; i += sizeof(uint32_t)) {
		uint32_t word = ((uint32_t)buf[i] << 24) |
				((uint32_t)buf[i + 1] << 16) |
				((uint32_t)buf[i + 2] << 8) |
				(uint32_t)buf[i + 3];

		CRC32_CalcCrc(word);
	}

	/*
	 * Feed the remaining 0..3 bytes, MSB-first. The least significant
	 * bytes are zero-padded, which appends zero bytes to the message
	 * for lengths that are not a multiple of 4 (a hardware limitation,
	 * as the CRC32 unit only accepts 32-bit writes).
	 */
	if (i < bufsize) {
		uint32_t word = 0;

		for (size_t j = 0; j < bufsize - i; j++) {
			word |= (uint32_t)buf[i + j] << (24 - 8 * j);
		}
		CRC32_CalcCrc(word);
	}

	return 0;
}

static int crc_n32_finish(const struct device *dev, struct crc_ctx *ctx)
{
	if (ctx->state != CRC_STATE_IN_PROGRESS) {
		return -EINVAL;
	}

	/* CRC-32/MPEG-2 has no final XOR */
	ctx->result = CRC32_GetCrc();

	crc_n32_release(dev, ctx);

	return 0;
}

static DEVICE_API(crc, crc_n32_driver_api) = {
	.begin = crc_n32_begin,
	.update = crc_n32_update,
	.finish = crc_n32_finish,
};

static int crc_n32_init(const struct device *dev)
{
	const struct crc_n32_config *config = dev->config;
	struct crc_n32_data *data = dev->data;
	int ret;

	ret = clock_control_on(DEVICE_DT_GET(DT_NODELABEL(rcc)),
			       (clock_control_subsys_t)&config->apb_clk_en);
	if (ret < 0) {
		LOG_ERR("Failed to enable CRC clock: %d", ret);
		return ret;
	}

	k_sem_init(&data->dev_lock, 1, 1);

	return 0;
}

#define CRC_N32_INIT(inst)						\
	static const struct crc_n32_config crc_n32_config_##inst = {	\
		.apb_clk_en = DT_INST_CLOCKS_CELL(inst, bits),		\
	};								\
									\
	static struct crc_n32_data crc_n32_data_##inst;			\
									\
	DEVICE_DT_INST_DEFINE(inst, crc_n32_init, NULL,			\
			      &crc_n32_data_##inst, &crc_n32_config_##inst, \
			      POST_KERNEL, CONFIG_CRC_DRIVER_INIT_PRIORITY, \
			      &crc_n32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(CRC_N32_INIT);
