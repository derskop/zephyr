/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#define RX_TIMEOUT         K_MSEC(100)
#define TX_TIMEOUT         K_MSEC(100)
#define STRESS_FRAME_COUNT 1000

CAN_MSGQ_DEFINE(rx_msgq, 4);

static const struct device *const can1 = DEVICE_DT_GET(DT_NODELABEL(can1));
static const struct device *const can2 = DEVICE_DT_GET(DT_NODELABEL(can2));

static int start_loopback(const struct device *dev)
{
	int ret;

	ret = can_set_mode(dev, CAN_MODE_LOOPBACK | CAN_MODE_LISTENONLY);
	if (ret != 0) {
		return ret;
	}

	return can_start(dev);
}

static void stop_if_started(const struct device *dev)
{
	int ret = can_stop(dev);

	zassert_true((ret == 0) || (ret == -EALREADY), "%s: can_stop failed: %d", dev->name, ret);
}

static int exchange(const struct device *dev, const struct can_frame *tx)
{
	const struct can_filter filter = {
		.flags = (tx->flags & CAN_FRAME_IDE) ? CAN_FILTER_IDE : 0U,
		.id = tx->id,
		.mask = (tx->flags & CAN_FRAME_IDE) ? CAN_EXT_ID_MASK : CAN_STD_ID_MASK,
	};
	struct can_frame rx;
	int filter_id;
	int ret;

	filter_id = can_add_rx_filter_msgq(dev, &rx_msgq, &filter);
	if (filter_id < 0) {
		return filter_id;
	}

	k_msgq_purge(&rx_msgq);
	ret = can_send(dev, tx, TX_TIMEOUT, NULL, NULL);
	if (ret == 0) {
		ret = k_msgq_get(&rx_msgq, &rx, RX_TIMEOUT);
	}

	can_remove_rx_filter(dev, filter_id);
	if (ret != 0) {
		return ret;
	}

	if ((rx.id != tx->id) || (rx.flags != tx->flags) || (rx.dlc != tx->dlc) ||
	    (memcmp(rx.data, tx->data, can_dlc_to_bytes(tx->dlc)) != 0)) {
		return -EBADMSG;
	}

	return 0;
}

static void check_basic_frames(const struct device *dev)
{
	const struct can_frame standard = {
		.id = 0x111,
		.dlc = 4,
		.data = {0xde, 0xad, 0xbe, 0xef},
	};
	const struct can_frame extended = {
		.flags = CAN_FRAME_IDE,
		.id = 0x1234567,
		.dlc = 8,
		.data = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77},
	};
	int ret = start_loopback(dev);

	zassert_ok(ret, "%s: start failed: %d", dev->name, ret);
	zexpect_ok(exchange(dev, &standard), "%s: standard frame failed", dev->name);
	zexpect_ok(exchange(dev, &extended), "%s: extended frame failed", dev->name);
	stop_if_started(dev);
}

ZTEST(can_n32_bxcan, test_can1_basic_frames)
{
	check_basic_frames(can1);
}

ZTEST(can_n32_bxcan, test_can2_basic_frames)
{
	check_basic_frames(can2);
}

ZTEST(can_n32_bxcan, test_dlc_boundaries)
{
	struct can_frame frame = {
		.id = 0x120,
		.data = {0x10, 0x21, 0x32, 0x43, 0x54, 0x65, 0x76, 0x87},
	};
	const uint8_t dlcs[] = {0, 1, 8};
	int ret = start_loopback(can1);

	zassert_ok(ret, "CAN1 start failed: %d", ret);
	for (size_t i = 0; i < ARRAY_SIZE(dlcs); i++) {
		frame.id++;
		frame.dlc = dlcs[i];
		zexpect_ok(exchange(can1, &frame), "DLC %u failed", dlcs[i]);
	}
	stop_if_started(can1);
}

ZTEST(can_n32_bxcan, test_filter_rejection)
{
	const struct can_filter filter = {
		.id = 0x321,
		.mask = CAN_STD_ID_MASK,
	};
	const struct can_frame rejected = {
		.id = 0x322,
		.dlc = 1,
		.data = {0xa5},
	};
	struct can_frame rx;
	int filter_id;
	int ret = start_loopback(can1);

	zassert_ok(ret, "CAN1 start failed: %d", ret);
	filter_id = can_add_rx_filter_msgq(can1, &rx_msgq, &filter);
	zassert_true(filter_id >= 0, "add filter failed: %d", filter_id);
	k_msgq_purge(&rx_msgq);
	zassert_ok(can_send(can1, &rejected, TX_TIMEOUT, NULL, NULL));
	zexpect_equal(k_msgq_get(&rx_msgq, &rx, K_MSEC(20)), -EAGAIN,
		      "non-matching frame passed filter");
	can_remove_rx_filter(can1, filter_id);
	stop_if_started(can1);
}

ZTEST(can_n32_bxcan, test_filter_exhaustion_and_release)
{
	struct can_filter filter = {
		.mask = CAN_STD_ID_MASK,
	};
	int ids[CONFIG_CAN_N32_BXCAN_MAX_STD_ID_FILTERS];
	int ret = start_loopback(can1);

	zassert_ok(ret, "CAN1 start failed: %d", ret);
	for (size_t i = 0; i < ARRAY_SIZE(ids); i++) {
		filter.id = 0x400 + i;
		ids[i] = can_add_rx_filter_msgq(can1, &rx_msgq, &filter);
		zassert_true(ids[i] >= 0, "filter %u allocation failed: %d", i, ids[i]);
	}

	filter.id = 0x500;
	ret = can_add_rx_filter_msgq(can1, &rx_msgq, &filter);
	zexpect_equal(ret, -ENOSPC, "extra filter returned %d", ret);

	for (size_t i = 0; i < ARRAY_SIZE(ids); i++) {
		can_remove_rx_filter(can1, ids[i]);
	}

	ret = can_add_rx_filter_msgq(can1, &rx_msgq, &filter);
	zexpect_true(ret >= 0, "released filter was not reusable: %d", ret);
	if (ret >= 0) {
		can_remove_rx_filter(can1, ret);
	}
	stop_if_started(can1);
}

ZTEST(can_n32_bxcan, test_repeated_start_stop)
{
	for (int i = 0; i < 100; i++) {
		zassert_ok(start_loopback(can1), "start iteration %d failed", i);
		zassert_ok(can_stop(can1), "stop iteration %d failed", i);
	}
}

ZTEST(can_n32_bxcan, test_stress_1000_frames)
{
	struct can_frame frame = {
		.id = 0x600,
		.dlc = 8,
	};
	int ret = start_loopback(can1);

	zassert_ok(ret, "CAN1 start failed: %d", ret);
	for (int i = 0; i < STRESS_FRAME_COUNT; i++) {
		frame.data_32[0] = i;
		frame.data_32[1] = ~i;
		ret = exchange(can1, &frame);
		zassert_ok(ret, "stress frame %d failed: %d", i, ret);
	}
	stop_if_started(can1);
}

static void *can_n32_setup(void)
{
	zassert_true(device_is_ready(can1), "CAN1 is not ready");
	zassert_true(device_is_ready(can2), "CAN2 is not ready");
	return NULL;
}

ZTEST_SUITE(can_n32_bxcan, NULL, can_n32_setup, NULL, NULL, NULL);
