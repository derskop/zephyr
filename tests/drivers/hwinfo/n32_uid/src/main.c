/*
 * Copyright (c) 2026 Nations Technologies
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include <zephyr/drivers/hwinfo.h>
#include <zephyr/ztest.h>

#define N32_UID_SIZE 12

ZTEST(hwinfo_n32_uid, test_uid_length_and_stability)
{
	uint8_t first[16];
	uint8_t second[16];
	ssize_t first_len;
	ssize_t second_len;

	first_len = hwinfo_get_device_id(first, sizeof(first));
	second_len = hwinfo_get_device_id(second, sizeof(second));

	zassert_equal(first_len, N32_UID_SIZE, "unexpected UID length: %zd", first_len);
	zassert_equal(second_len, N32_UID_SIZE, "unexpected UID length: %zd", second_len);
	zassert_mem_equal(first, second, N32_UID_SIZE, "UID changed between reads");
}

ZTEST(hwinfo_n32_uid, test_uid_is_plausible)
{
	uint8_t uid[N32_UID_SIZE];
	uint8_t all_zero[N32_UID_SIZE] = {0};
	uint8_t all_ones[N32_UID_SIZE];

	memset(all_ones, 0xff, sizeof(all_ones));
	zassert_equal(hwinfo_get_device_id(uid, sizeof(uid)), N32_UID_SIZE);
	zassert_not_equal(memcmp(uid, all_zero, sizeof(uid)), 0, "UID is all zero");
	zassert_not_equal(memcmp(uid, all_ones, sizeof(uid)), 0, "UID is all 0xff");
}

ZTEST(hwinfo_n32_uid, test_zero_length_read)
{
	uint8_t canary = 0xa5;

	zassert_equal(hwinfo_get_device_id(&canary, 0), 0);
	zassert_equal(canary, 0xa5, "zero-length read modified the buffer");
}

ZTEST_SUITE(hwinfo_n32_uid, NULL, NULL, NULL, NULL, NULL);
