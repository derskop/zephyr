/*
 * Copyright (c) 2026 Nations Technologies
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include <zephyr/drivers/hwinfo.h>
#include <zephyr/sys/util.h>

#include <n32g45x_dbg.h>

ssize_t z_impl_hwinfo_get_device_id(uint8_t *buffer, size_t length)
{
	uint8_t uid[UID_LENGTH];
	size_t out_len = MIN(length, sizeof(uid));

	if (out_len == 0U) {
		return 0;
	}

	GetUID(uid);
	memcpy(buffer, uid, out_len);

	return out_len;
}
