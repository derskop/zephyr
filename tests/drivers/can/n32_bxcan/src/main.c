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

#define RX_TIMEOUT          K_MSEC(100)
#define TX_TIMEOUT          K_MSEC(100)
#define STRESS_FRAME_COUNT  1000
#define NORMAL_STRESS_COUNT 100

CAN_MSGQ_DEFINE(rx_msgq, 4);

static const struct device *const can1 = DEVICE_DT_GET(DT_NODELABEL(can1));
static const struct device *const can2 = DEVICE_DT_GET(DT_NODELABEL(can2));

static void stop_if_started(const struct device *dev)
{
	int ret = can_stop(dev);

	zassert_true((ret == 0) || (ret == -EALREADY), "%s: can_stop failed: %d", dev->name, ret);
}

static int start_loopback(const struct device *dev)
{
	int ret;

	ret = can_set_mode(dev, CAN_MODE_LOOPBACK | CAN_MODE_LISTENONLY);
	if (ret != 0) {
		return ret;
	}

	return can_start(dev);
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
	    (((tx->flags & CAN_FRAME_RTR) == 0U) &&
	     (memcmp(rx.data, tx->data, can_dlc_to_bytes(tx->dlc)) != 0))) {
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
	Z_TEST_SKIP_IFDEF(CONFIG_TEST_CAN_N32_NORMAL_MODE);
	check_basic_frames(can1);
}

ZTEST(can_n32_bxcan, test_can2_basic_frames)
{
	Z_TEST_SKIP_IFDEF(CONFIG_TEST_CAN_N32_NORMAL_MODE);
	check_basic_frames(can2);
}

ZTEST(can_n32_bxcan, test_dlc_boundaries)
{
	Z_TEST_SKIP_IFDEF(CONFIG_TEST_CAN_N32_NORMAL_MODE);
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

ZTEST(can_n32_bxcan, test_remote_frame)
{
	Z_TEST_SKIP_IFDEF(CONFIG_TEST_CAN_N32_NORMAL_MODE);
	const struct can_frame frame = {
		.flags = CAN_FRAME_RTR,
		.id = 0x123,
		.dlc = 8,
	};
	int ret = start_loopback(can1);

	zassert_ok(ret, "CAN1 start failed: %d", ret);
	zexpect_ok(exchange(can1, &frame), "standard remote frame failed");
	stop_if_started(can1);
}

ZTEST(can_n32_bxcan, test_state_and_stopped_send)
{
	Z_TEST_SKIP_IFDEF(CONFIG_TEST_CAN_N32_NORMAL_MODE);
	const struct can_frame frame = {
		.id = 0x124,
		.dlc = 0,
	};
	struct can_bus_err_cnt err_cnt;
	enum can_state state;
	int ret;

	stop_if_started(can1);
	zassert_ok(can_get_state(can1, &state, &err_cnt));
	zexpect_equal(state, CAN_STATE_STOPPED, "unexpected stopped state: %d", state);
	zexpect_equal(can_send(can1, &frame, K_NO_WAIT, NULL, NULL), -ENETDOWN,
		      "send while stopped was accepted");

	ret = start_loopback(can1);
	zassert_ok(ret, "CAN1 start failed: %d", ret);
	zassert_ok(can_get_state(can1, &state, &err_cnt));
	zexpect_equal(state, CAN_STATE_ERROR_ACTIVE, "unexpected started state: %d", state);
	stop_if_started(can1);
}

ZTEST(can_n32_bxcan, test_filter_rejection)
{
	Z_TEST_SKIP_IFDEF(CONFIG_TEST_CAN_N32_NORMAL_MODE);
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
	Z_TEST_SKIP_IFDEF(CONFIG_TEST_CAN_N32_NORMAL_MODE);
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
	Z_TEST_SKIP_IFDEF(CONFIG_TEST_CAN_N32_NORMAL_MODE);
	for (int i = 0; i < 100; i++) {
		zassert_ok(start_loopback(can1), "start iteration %d failed", i);
		zassert_ok(can_stop(can1), "stop iteration %d failed", i);
	}
}

ZTEST(can_n32_bxcan, test_stress_1000_frames)
{
	Z_TEST_SKIP_IFDEF(CONFIG_TEST_CAN_N32_NORMAL_MODE);
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

static int start_normal(const struct device *dev)
{
	int ret = can_set_mode(dev, CAN_MODE_NORMAL);

	if (ret != 0) {
		return ret;
	}

	return can_start(dev);
}

static int cross_bus_exchange(const struct device *sender, const struct device *receiver,
			      const struct can_frame *tx)
{
	const struct can_filter filter = {
		.flags = (tx->flags & CAN_FRAME_IDE) ? CAN_FILTER_IDE : 0U,
		.id = tx->id,
		.mask = (tx->flags & CAN_FRAME_IDE) ? CAN_EXT_ID_MASK : CAN_STD_ID_MASK,
	};
	struct can_frame rx;
	int filter_id;
	int ret;

	filter_id = can_add_rx_filter_msgq(receiver, &rx_msgq, &filter);
	if (filter_id < 0) {
		return filter_id;
	}

	k_msgq_purge(&rx_msgq);
	ret = can_send(sender, tx, TX_TIMEOUT, NULL, NULL);
	if (ret == 0) {
		ret = k_msgq_get(&rx_msgq, &rx, RX_TIMEOUT);
	}

	can_remove_rx_filter(receiver, filter_id);
	if (ret != 0) {
		return ret;
	}

	if ((rx.id != tx->id) || (rx.flags != tx->flags) || (rx.dlc != tx->dlc) ||
	    (memcmp(rx.data, tx->data, can_dlc_to_bytes(tx->dlc)) != 0)) {
		return -EBADMSG;
	}

	return 0;
}

static void start_normal_bus(void)
{
	int ret = start_normal(can1);

	zassert_ok(ret, "CAN1 normal-mode start failed: %d", ret);
	ret = start_normal(can2);
	if (ret != 0) {
		stop_if_started(can1);
	}
	zassert_ok(ret, "CAN2 normal-mode start failed: %d", ret);
}

static void stop_normal_bus(void)
{
	stop_if_started(can2);
	stop_if_started(can1);
}

ZTEST(can_n32_bxcan, test_normal_bidirectional_frames)
{
	Z_TEST_SKIP_IFNDEF(CONFIG_TEST_CAN_N32_NORMAL_MODE);
	const struct can_frame can1_to_can2 = {
		.id = 0x100,
		.dlc = 8,
		.data = {0x10, 0x21, 0x32, 0x43, 0x54, 0x65, 0x76, 0x87},
	};
	const struct can_frame can2_to_can1 = {
		.flags = CAN_FRAME_IDE,
		.id = 0x1234567,
		.dlc = 4,
		.data = {0xde, 0xad, 0xbe, 0xef},
	};

	start_normal_bus();
	zexpect_ok(cross_bus_exchange(can1, can2, &can1_to_can2),
		   "CAN1 to CAN2 standard frame failed");
	zexpect_ok(cross_bus_exchange(can2, can1, &can2_to_can1),
		   "CAN2 to CAN1 extended frame failed");
	stop_normal_bus();
}

ZTEST(can_n32_bxcan, test_normal_stress)
{
	Z_TEST_SKIP_IFNDEF(CONFIG_TEST_CAN_N32_NORMAL_MODE);
	struct can_frame frame = {
		.id = 0x601,
		.dlc = 8,
	};
	int ret;

	start_normal_bus();
	for (int i = 0; i < NORMAL_STRESS_COUNT; i++) {
		frame.data_32[0] = i;
		frame.data_32[1] = ~i;
		ret = cross_bus_exchange(can1, can2, &frame);
		zassert_ok(ret, "normal-mode stress frame %d failed: %d", i, ret);
	}
	stop_normal_bus();
}

static void *can_n32_setup(void)
{
	zassert_true(device_is_ready(can1), "CAN1 is not ready");
	zassert_true(device_is_ready(can2), "CAN2 is not ready");
	return NULL;
}

ZTEST_SUITE(can_n32_bxcan, NULL, can_n32_setup, NULL, NULL, NULL);
