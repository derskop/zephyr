/*
 * Copyright (c) 2026 Nations Technologies
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nsing_n32_flash_controller

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include <n32g45x_flash.h>

#define N32_FLASH_NODE       DT_INST(0, soc_nv_flash)
#define N32_FLASH_BASE       DT_REG_ADDR(N32_FLASH_NODE)
#define N32_FLASH_SIZE       DT_REG_SIZE(N32_FLASH_NODE)
#define N32_FLASH_WRITE_SIZE DT_PROP(N32_FLASH_NODE, write_block_size)
#define N32_FLASH_PAGE_SIZE  DT_PROP(N32_FLASH_NODE, erase_block_size)

BUILD_ASSERT(N32_FLASH_WRITE_SIZE == sizeof(uint32_t),
	     "N32G45x flash supports only 32-bit programming");
BUILD_ASSERT((N32_FLASH_SIZE % N32_FLASH_PAGE_SIZE) == 0,
	     "Flash size must be a multiple of its page size");

struct flash_n32_data {
	struct k_sem lock;
};

static struct flash_n32_data flash_n32_data;

static const struct flash_parameters flash_n32_parameters = {
	.write_block_size = N32_FLASH_WRITE_SIZE,
	.erase_value = 0xff,
};

static bool flash_n32_valid_range(off_t offset, size_t len)
{
	return (offset >= 0) && ((size_t)offset <= N32_FLASH_SIZE) &&
	       (len <= (N32_FLASH_SIZE - (size_t)offset));
}

static int flash_n32_status_to_errno(FLASH_STS status)
{
	switch (status) {
	case FLASH_COMPL:
		return 0;
	case FLASH_TIMEOUT:
		return -ETIMEDOUT;
	case FLASH_ERR_ADD:
		return -EINVAL;
	case FLASH_ERR_WRP:
		return -EACCES;
	default:
		return -EIO;
	}
}

static int flash_n32_read(const struct device *dev, off_t offset, void *data, size_t len)
{
	ARG_UNUSED(dev);

	if (len == 0U) {
		return 0;
	}

	if ((data == NULL) || !flash_n32_valid_range(offset, len)) {
		return -EINVAL;
	}

	memcpy(data, (const void *)(N32_FLASH_BASE + (uintptr_t)offset), len);

	return 0;
}

static int flash_n32_write(const struct device *dev, off_t offset, const void *data, size_t len)
{
	struct flash_n32_data *dev_data = dev->data;
	const uint8_t *src = data;
	bool relock;
	int ret = 0;

	if (len == 0U) {
		return 0;
	}

	if ((data == NULL) || !flash_n32_valid_range(offset, len) ||
	    !IS_ALIGNED(offset, N32_FLASH_WRITE_SIZE) || !IS_ALIGNED(len, N32_FLASH_WRITE_SIZE)) {
		return -EINVAL;
	}

	k_sem_take(&dev_data->lock, K_FOREVER);
	relock = (FLASH->CTRL & FLASH_CTRL_LOCK) != 0U;
	if (relock) {
		FLASH_Unlock();
	}

	for (size_t pos = 0U; pos < len; pos += N32_FLASH_WRITE_SIZE) {
		uint32_t value;

		memcpy(&value, src + pos, sizeof(value));
		ret = flash_n32_status_to_errno(
			FLASH_ProgramWord(N32_FLASH_BASE + (uint32_t)offset + pos, value));
		if (ret < 0) {
			break;
		}
	}

	if (relock) {
		FLASH_Lock();
	}
	FLASH_iCacheRST();
	k_sem_give(&dev_data->lock);

	return ret;
}

static int flash_n32_erase(const struct device *dev, off_t offset, size_t size)
{
	struct flash_n32_data *dev_data = dev->data;
	bool relock;
	int ret = 0;

	if (size == 0U) {
		return 0;
	}

	if (!flash_n32_valid_range(offset, size) || !IS_ALIGNED(offset, N32_FLASH_PAGE_SIZE) ||
	    !IS_ALIGNED(size, N32_FLASH_PAGE_SIZE)) {
		return -EINVAL;
	}

	k_sem_take(&dev_data->lock, K_FOREVER);
	relock = (FLASH->CTRL & FLASH_CTRL_LOCK) != 0U;
	if (relock) {
		FLASH_Unlock();
	}

	for (size_t pos = 0U; pos < size; pos += N32_FLASH_PAGE_SIZE) {
		ret = flash_n32_status_to_errno(
			FLASH_EraseOnePage(N32_FLASH_BASE + (uint32_t)offset + pos));
		if (ret < 0) {
			break;
		}
	}

	if (relock) {
		FLASH_Lock();
	}
	FLASH_iCacheRST();
	k_sem_give(&dev_data->lock);

	return ret;
}

static const struct flash_parameters *flash_n32_get_parameters(const struct device *dev)
{
	ARG_UNUSED(dev);

	return &flash_n32_parameters;
}

static int flash_n32_get_size(const struct device *dev, uint64_t *size)
{
	ARG_UNUSED(dev);

	*size = N32_FLASH_SIZE;

	return 0;
}

#ifdef CONFIG_FLASH_PAGE_LAYOUT
static void flash_n32_page_layout(const struct device *dev,
				  const struct flash_pages_layout **layout, size_t *layout_size)
{
	static const struct flash_pages_layout flash_layout = {
		.pages_count = N32_FLASH_SIZE / N32_FLASH_PAGE_SIZE,
		.pages_size = N32_FLASH_PAGE_SIZE,
	};

	ARG_UNUSED(dev);

	*layout = &flash_layout;
	*layout_size = 1U;
}
#endif

static DEVICE_API(flash, flash_n32_api) = {
	.read = flash_n32_read,
	.write = flash_n32_write,
	.erase = flash_n32_erase,
	.get_parameters = flash_n32_get_parameters,
	.get_size = flash_n32_get_size,
#ifdef CONFIG_FLASH_PAGE_LAYOUT
	.page_layout = flash_n32_page_layout,
#endif
};

static int flash_n32_init(const struct device *dev)
{
	struct flash_n32_data *data = dev->data;

	k_sem_init(&data->lock, 1, 1);

	return 0;
}

DEVICE_DT_INST_DEFINE(0, flash_n32_init, NULL, &flash_n32_data, NULL, POST_KERNEL,
		      CONFIG_FLASH_INIT_PRIORITY, &flash_n32_api);
