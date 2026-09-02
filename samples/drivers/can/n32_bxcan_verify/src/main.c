/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Driver verification application for the N32 bxCAN driver.
 *
 * Target: n32g45xml_stb (N32G457ME). CAN1 = PA11/PA12, CAN2 = PB12/PB13.
 *
 * Two test phases:
 *
 *  1. Internal loopback (default):
 *     Both CAN1 and CAN2 run independently in CAN_MODE_LOOPBACK. Each channel
 *     transmits standard and extended frames and verifies the frames received
 *     back through its own RX filters. Exercises set_mode/start/send/RX
 *     filters/ISRs per channel. Requires no external hardware.
 *
 *  2. Cross-bus (CONFIG_CAN_SAMPLE_CROSS_BUS=y):
 *     Both channels run in normal mode on a shared physical bus. CAN1 sends a
 *     standard frame to CAN2 and CAN2 sends a standard frame to CAN1.
 *     Requires two transceivers wired onto the same bus, terminated 120 ohm.
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/sys/printk.h>

#define MSGQ_COUNT       4
#define RX_WAIT_TIMEOUT  K_MSEC(1000)

/* Internal loopback test frame IDs */
#define STD_TX_ID        0x111
#define EXT_TX_ID        0x1234567

/* Cross-bus test frame IDs */
#define CAN1_RX_STD_ID   0x200   /* sent by CAN2, received by CAN1 */
#define CAN2_RX_STD_ID   0x100   /* sent by CAN1, received by CAN2 */

CAN_MSGQ_DEFINE(rx_msgq_can1, MSGQ_COUNT);
CAN_MSGQ_DEFINE(rx_msgq_can2, MSGQ_COUNT);

struct can_verify_ctx {
	const char *name;
	const struct device *dev;
	struct k_msgq *rx_msgq;
	unsigned int pass;
	unsigned int fail;
};

static struct can_verify_ctx can1_ctx = {
	.name = "can1",
	.dev = DEVICE_DT_GET(DT_NODELABEL(can1)),
	.rx_msgq = &rx_msgq_can1,
};

static struct can_verify_ctx can2_ctx = {
	.name = "can2",
	.dev = DEVICE_DT_GET(DT_NODELABEL(can2)),
	.rx_msgq = &rx_msgq_can2,
};

static void tx_callback(const struct device *dev, int error, void *user_data)
{
	struct can_verify_ctx *ctx = user_data;

	ARG_UNUSED(dev);

	if (error != 0) {
		printk("%s: TX callback error: %d\n", ctx->name, error);
	}
}

static void record(struct can_verify_ctx *ctx, const char *test, bool ok)
{
	if (ok) {
		ctx->pass++;
		printk("%s: [PASS] %s\n", ctx->name, test);
	} else {
		ctx->fail++;
		printk("%s: [FAIL] %s\n", ctx->name, test);
	}
}

/*
 * Add a filter, purge stale frames, send @p tx, then wait for the echoed frame
 * and verify its ID/DLC/payload. Used for the internal loopback test.
 */
static bool loopback_exchange(struct can_verify_ctx *ctx,
			      const struct can_filter *filter,
			      const struct can_frame *tx)
{
	struct can_frame rx;
	int filter_id;
	int ret;

	filter_id = can_add_rx_filter_msgq(ctx->dev, ctx->rx_msgq, filter);
	if (filter_id < 0) {
		printk("%s: failed to add RX filter (err %d)\n", ctx->name, filter_id);
		return false;
	}

	k_msgq_purge(ctx->rx_msgq);

	ret = can_send(ctx->dev, tx, K_MSEC(500), tx_callback, ctx);
	if (ret != 0) {
		printk("%s: can_send failed (err %d)\n", ctx->name, ret);
		can_remove_rx_filter(ctx->dev, filter_id);
		return false;
	}

	ret = k_msgq_get(ctx->rx_msgq, &rx, RX_WAIT_TIMEOUT);
	can_remove_rx_filter(ctx->dev, filter_id);

	if (ret != 0) {
		printk("%s: timeout waiting for RX frame\n", ctx->name);
		return false;
	}

	if (rx.id != tx->id) {
		printk("%s: ID mismatch: sent 0x%x, received 0x%x\n",
		       ctx->name, tx->id, rx.id);
		return false;
	}

	if (rx.dlc != tx->dlc) {
		printk("%s: DLC mismatch: sent %u, received %u\n",
		       ctx->name, tx->dlc, rx.dlc);
		return false;
	}

	if (memcmp(rx.data, tx->data, tx->dlc) != 0) {
		printk("%s: data mismatch\n", ctx->name);
		return false;
	}

	return true;
}

static int run_loopback_test(struct can_verify_ctx *ctx)
{
	const struct can_filter std_filter = {
		.flags = 0U,
		.id = STD_TX_ID,
		.mask = CAN_STD_ID_MASK,
	};
	const struct can_filter ext_filter = {
		.flags = CAN_FILTER_IDE,
		.id = EXT_TX_ID,
		.mask = CAN_EXT_ID_MASK,
	};
	struct can_frame tx = { 0 };
	int ret;

	if (!device_is_ready(ctx->dev)) {
		printk("%s: device not ready\n", ctx->name);
		return -ENODEV;
	}

	printk("%s: max filters: std=%d ext=%d\n", ctx->name,
	       can_get_max_filters(ctx->dev, false),
	       can_get_max_filters(ctx->dev, true));

	ret = can_set_mode(ctx->dev, CAN_MODE_LOOPBACK);
	if (ret != 0) {
		printk("%s: can_set_mode(CAN_MODE_LOOPBACK) failed (err %d)\n",
		       ctx->name, ret);
		return ret;
	}

	ret = can_start(ctx->dev);
	if (ret != 0) {
		printk("%s: can_start failed (err %d)\n", ctx->name, ret);
		return ret;
	}

	/* Standard frame */
	tx.id = STD_TX_ID;
	tx.dlc = 4U;
	tx.data[0] = 0xde;
	tx.data[1] = 0xad;
	tx.data[2] = 0xbe;
	tx.data[3] = 0xef;
	record(ctx, "loopback standard frame",
	       loopback_exchange(ctx, &std_filter, &tx));

	/* Extended frame */
	tx.flags = CAN_FRAME_IDE;
	tx.id = EXT_TX_ID;
	tx.dlc = 2U;
	tx.data[0] = 0x12;
	tx.data[1] = 0x34;
	record(ctx, "loopback extended frame",
	       loopback_exchange(ctx, &ext_filter, &tx));

	(void)can_stop(ctx->dev);

	return 0;
}

/*
 * Send @p tx from @p sender and wait for it on @p receiver's RX msgq,
 * verifying ID/DLC/payload. Used for the cross-bus test.
 */
static bool cross_bus_exchange(struct can_verify_ctx *sender,
			       struct can_verify_ctx *receiver,
			       const struct can_frame *tx)
{
	struct can_frame rx;
	int ret;

	k_msgq_purge(receiver->rx_msgq);

	ret = can_send(sender->dev, tx, K_MSEC(500), tx_callback, sender);
	if (ret != 0) {
		printk("%s: can_send failed (err %d)\n", sender->name, ret);
		return false;
	}

	ret = k_msgq_get(receiver->rx_msgq, &rx, RX_WAIT_TIMEOUT);
	if (ret != 0) {
		printk("%s: timeout waiting for frame from %s\n",
		       receiver->name, sender->name);
		return false;
	}

	if (rx.id != tx->id || rx.dlc != tx->dlc ||
	    memcmp(rx.data, tx->data, tx->dlc) != 0) {
		printk("%s: frame from %s mismatch\n", receiver->name, sender->name);
		return false;
	}

	return true;
}

static int run_cross_bus_test(struct can_verify_ctx *ctx1, struct can_verify_ctx *ctx2)
{
	const struct can_filter ctx1_rx_filter = {
		.flags = 0U,
		.id = CAN1_RX_STD_ID,
		.mask = CAN_STD_ID_MASK,
	};
	const struct can_filter ctx2_rx_filter = {
		.flags = 0U,
		.id = CAN2_RX_STD_ID,
		.mask = CAN_STD_ID_MASK,
	};
	struct can_frame tx = { 0 };
	int f1, f2, ret;

	if (!device_is_ready(ctx1->dev) || !device_is_ready(ctx2->dev)) {
		printk("cross-bus: can1/can2 device not ready\n");
		return -ENODEV;
	}

	ret = can_start(ctx1->dev);
	if (ret != 0) {
		printk("%s: can_start failed (err %d)\n", ctx1->name, ret);
		return ret;
	}

	ret = can_start(ctx2->dev);
	if (ret != 0) {
		printk("%s: can_start failed (err %d)\n", ctx2->name, ret);
		(void)can_stop(ctx1->dev);
		return ret;
	}

	f1 = can_add_rx_filter_msgq(ctx1->dev, ctx1->rx_msgq, &ctx1_rx_filter);
	f2 = can_add_rx_filter_msgq(ctx2->dev, ctx2->rx_msgq, &ctx2_rx_filter);

	if (f1 < 0 || f2 < 0) {
		printk("cross-bus: failed to add RX filters (f1=%d, f2=%d)\n", f1, f2);
		if (f1 >= 0) {
			can_remove_rx_filter(ctx1->dev, f1);
		}
		if (f2 >= 0) {
			can_remove_rx_filter(ctx2->dev, f2);
		}
		(void)can_stop(ctx2->dev);
		(void)can_stop(ctx1->dev);
		return -ENOSPC;
	}

	/* CAN1 -> CAN2 */
	tx.id = CAN2_RX_STD_ID;
	tx.dlc = 3U;
	tx.data[0] = 0x01;
	tx.data[1] = 0x02;
	tx.data[2] = 0x03;
	record(ctx2, "cross-bus frame CAN1->CAN2",
	       cross_bus_exchange(ctx1, ctx2, &tx));

	/* CAN2 -> CAN1 */
	tx.id = CAN1_RX_STD_ID;
	tx.dlc = 3U;
	tx.data[0] = 0x11;
	tx.data[1] = 0x22;
	tx.data[2] = 0x33;
	record(ctx1, "cross-bus frame CAN2->CAN1",
	       cross_bus_exchange(ctx2, ctx1, &tx));

	can_remove_rx_filter(ctx1->dev, f1);
	can_remove_rx_filter(ctx2->dev, f2);

	(void)can_stop(ctx2->dev);
	(void)can_stop(ctx1->dev);

	return 0;
}

int main(void)
{
	printk("N32 bxCAN driver verification\n");

	if (IS_ENABLED(CONFIG_CAN_SAMPLE_CROSS_BUS)) {
		(void)run_cross_bus_test(&can1_ctx, &can2_ctx);
	} else {
		(void)run_loopback_test(&can1_ctx);
		(void)run_loopback_test(&can2_ctx);
	}

	printk("can1: %u pass, %u fail\n", can1_ctx.pass, can1_ctx.fail);
	printk("can2: %u pass, %u fail\n", can2_ctx.pass, can2_ctx.fail);

	if (can1_ctx.fail == 0U && can2_ctx.fail == 0U) {
		printk("*** ALL TESTS PASSED ***\n");
	} else {
		printk("*** TESTS FAILED ***\n");
	}

	return 0;
}
