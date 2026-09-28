/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * N32G45x I2S loopback demo.
 *
 * Two I2S instances of the same SoC are cabled to each other: i2s3 drives the
 * frame and bit clocks as controller and transmits a sine wave, i2s2 follows
 * them as target and receives it. Each round the block that came back is
 * compared sample by sample against the block that went out.
 *
 * The sequencing is the one the upstream i2s_api suite uses, because it is the
 * one that survives this hardware: the TX queue is given blocks before START
 * (the trigger has nothing to hand the DMA otherwise and fails with -ENOMEM),
 * the target is armed before the controller so it is already listening when
 * the bit clock starts, and each round writes before it reads so the queue
 * never runs dry.
 *
 * Nothing in the transfer loop prints. That is not tidiness. At 8 kHz a
 * BLOCK_SIZE block lasts about 4 ms, and one line of console output on this
 * board's USART1 at 115200 baud blocks for about 5 ms -- longer than the block
 * it is racing. Print inside the loop and the TX queue drains while the CPU is
 * still waiting on the UART, the completion callback finds nothing to re-arm
 * with, and the stream goes to ERROR: the next i2s_buf_write() then fails with
 * -EIO on a round that did nothing wrong. Results are collected and printed
 * after the transfer instead.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2s.h>

#define SAMPLE_NO	32
#define BLOCK_SIZE	(2 * SAMPLE_NO * sizeof(int16_t))	/* stereo 16-bit */
#define FRAME_CLK_FREQ	8000
#define ROUNDS		10
#define TIMEOUT_MS	2000

/*
 * The receiver may hand back a block that starts a little way into the
 * transmitted stream, so the comparison is allowed to slide by this many
 * sample pairs before it gives up. Upstream's own suite allows two.
 */
#define MAX_OFFSET	2

static const struct device *const dev_tx = DEVICE_DT_GET(DT_ALIAS(i2s_tx));
static const struct device *const dev_rx = DEVICE_DT_GET(DT_ALIAS(i2s_rx));

K_MEM_SLAB_DEFINE(tx_slab, BLOCK_SIZE, 4, 32);
K_MEM_SLAB_DEFINE(rx_slab, BLOCK_SIZE, 4, 32);

/*
 * One block is not enough to keep a receiver going, which is the point: the
 * second completion is guaranteed to find nothing free and take the overrun
 * path. What is measured afterwards is whether the block came back.
 */
#define OVR_BLOCKS 1
K_MEM_SLAB_DEFINE(ovr_slab, BLOCK_SIZE, OVR_BLOCKS, 32);

/*
 * One period of a sine wave for the left channel. The right channel carries
 * the same wave a quarter period later, so a block is not palindromic and
 * cannot still match itself after a stereo swap.
 */
static const int16_t sine[SAMPLE_NO] = {
	  6392,  12539,  18204,  23169,  27244,  30272,  32137,  32767,  32137,
	 30272,  27244,  23169,  18204,  12539,   6392,      0,  -6393, -12540,
	-18205, -23170, -27245, -30273, -32138, -32767, -32138, -30273, -27245,
	-23170, -18205, -12540,  -6393,     -1,
};

static int16_t tx_data[SAMPLE_NO * 2];
static int16_t rx_data[SAMPLE_NO * 2];

/* Filled in during the transfer, printed once it is over. */
struct round_result {
	int ret;
	int offset;
	int16_t first_l;
	int16_t first_r;
};

static struct round_result results[ROUNDS];

static void fill_tx(void)
{
	for (int i = 0; i < SAMPLE_NO; i++) {
		tx_data[2 * i] = sine[i];
		tx_data[2 * i + 1] = sine[(i + SAMPLE_NO / 4) % SAMPLE_NO];
	}
}

/*
 * Returns the offset, in sample pairs, at which the received block lines up
 * with the transmitted one, or -1 if no allowed offset lines them up. The
 * comparison starts at that offset, so it covers SAMPLE_NO - offset pairs.
 */
static int find_offset(void)
{
	for (int off = 0; off <= MAX_OFFSET; off++) {
		int i;

		for (i = off; i < SAMPLE_NO; i++) {
			if (rx_data[2 * i] != tx_data[2 * (i - off)] ||
			    rx_data[2 * i + 1] != tx_data[2 * (i - off) + 1]) {
				break;
			}
		}

		if (i == SAMPLE_NO) {
			return off;
		}
	}

	return -1;
}

int main(void)
{
	struct i2s_config cfg = {0};
	size_t rx_size;
	int ret;
	int passed = 0;

	printk("\n=== N32G45x I2S loopback ===\n");
	printk("TX (controller) : %s\n", dev_tx->name);
	printk("RX (target)     : %s\n", dev_rx->name);
	printk("16-bit stereo, I2S format, %u Hz, %u-byte blocks, %u rounds\n\n",
	       (unsigned int)FRAME_CLK_FREQ, (unsigned int)BLOCK_SIZE,
	       (unsigned int)ROUNDS);

	if (!device_is_ready(dev_tx) || !device_is_ready(dev_rx)) {
		printk("FAIL: I2S device not ready\n");
		return -ENODEV;
	}

	cfg.word_size = 16U;
	cfg.channels = 2U;
	cfg.format = I2S_FMT_DATA_FORMAT_I2S;
	cfg.frame_clk_freq = FRAME_CLK_FREQ;
	cfg.block_size = BLOCK_SIZE;
	cfg.timeout = TIMEOUT_MS;

	cfg.options = I2S_OPT_FRAME_CLK_CONTROLLER | I2S_OPT_BIT_CLK_CONTROLLER;
	cfg.mem_slab = &tx_slab;
	ret = i2s_configure(dev_tx, I2S_DIR_TX, &cfg);
	if (ret < 0) {
		printk("FAIL: TX configure: %d\n", ret);
		return ret;
	}

	cfg.options = I2S_OPT_FRAME_CLK_TARGET | I2S_OPT_BIT_CLK_TARGET;
	cfg.mem_slab = &rx_slab;
	ret = i2s_configure(dev_rx, I2S_DIR_RX, &cfg);
	if (ret < 0) {
		printk("FAIL: RX configure: %d\n", ret);
		return ret;
	}

	fill_tx();

	/*
	 * Two blocks of slack before the clock starts: one is what START hands
	 * to the DMA, the other is what the first completion callback re-arms
	 * with. That callback then does not depend on this loop having run.
	 */
	for (int i = 0; i < 2; i++) {
		ret = i2s_buf_write(dev_tx, tx_data, BLOCK_SIZE);
		if (ret < 0) {
			printk("FAIL: TX write: %d\n", ret);
			return ret;
		}
	}

	/* Target first: the controller is about to turn the bit clock on. */
	ret = i2s_trigger(dev_rx, I2S_DIR_RX, I2S_TRIGGER_START);
	if (ret < 0) {
		printk("FAIL: RX START: %d\n", ret);
		return ret;
	}

	ret = i2s_trigger(dev_tx, I2S_DIR_TX, I2S_TRIGGER_START);
	if (ret < 0) {
		printk("FAIL: TX START: %d\n", ret);
		return ret;
	}

	/* No console output in here -- see the note at the top of the file. */
	for (int round = 0; round < ROUNDS; round++) {
		results[round].ret = 0;
		results[round].offset = -1;

		ret = i2s_buf_write(dev_tx, tx_data, BLOCK_SIZE);
		if (ret < 0) {
			results[round].ret = ret;
			break;
		}

		rx_size = 0U;
		ret = i2s_buf_read(dev_rx, rx_data, &rx_size);
		if (ret < 0 || rx_size != BLOCK_SIZE) {
			results[round].ret = (ret < 0) ? ret : -EIO;
			break;
		}

		results[round].offset = find_offset();
		if (results[round].offset < 0) {
			/* Keep the block around for the mismatch dump below. */
			break;
		}

		results[round].first_l = rx_data[2 * results[round].offset];
		results[round].first_r = rx_data[2 * results[round].offset + 1];
		passed++;
	}

	/*
	 * DROP unwinds both streams and returns every block to its slab, so
	 * the demo can be started again without a reset.
	 */
	(void)i2s_trigger(dev_tx, I2S_DIR_TX, I2S_TRIGGER_DROP);
	(void)i2s_trigger(dev_rx, I2S_DIR_RX, I2S_TRIGGER_DROP);

	/* The transfer is over, so blocking console output is free now. */
	for (int round = 0; round < ROUNDS; round++) {
		if (results[round].ret < 0) {
			printk("round %2d: failed (%d)\n", round,
			       results[round].ret);
			break;
		}

		if (results[round].offset < 0) {
			printk("round %2d: MISMATCH\n", round);
			break;
		}

		printk("round %2d: OK   %2d samples, offset %d, first pair %6d %6d\n",
		       round, SAMPLE_NO - results[round].offset,
		       results[round].offset, results[round].first_l,
		       results[round].first_r);
	}

	if (passed != ROUNDS) {
		for (int i = 0; i < 4; i++) {
			printk("  sent %6d %6d\n", tx_data[2 * i],
			       tx_data[2 * i + 1]);
			printk("  got  %6d %6d\n", rx_data[2 * i],
			       rx_data[2 * i + 1]);
		}
	}

	printk("\n%u/%u rounds verified -> %s\n", (unsigned int)passed,
	       (unsigned int)ROUNDS, (passed == ROUNDS) ? "PASS" : "FAIL");

	/*
	 * Overrun recovery. With a one-block slab the receiver cannot keep up by
	 * construction, so the completion after the first block finds nothing to
	 * re-arm with and the stream goes to ERROR. Every block must still be
	 * accounted for when the dust settles, or an application that hits one
	 * overrun can never restart: the slab shrinks by a block each time.
	 */
	struct i2s_config ovr_cfg = {0};
	unsigned int free_blocks;

	ovr_cfg.word_size = 16U;
	ovr_cfg.channels = 2U;
	ovr_cfg.format = I2S_FMT_DATA_FORMAT_I2S;
	ovr_cfg.frame_clk_freq = FRAME_CLK_FREQ;
	ovr_cfg.block_size = BLOCK_SIZE;
	ovr_cfg.timeout = TIMEOUT_MS;
	ovr_cfg.options = I2S_OPT_FRAME_CLK_TARGET | I2S_OPT_BIT_CLK_TARGET;
	ovr_cfg.mem_slab = &ovr_slab;

	printk("\n--- RX overrun recovery ---\n");

	ret = i2s_configure(dev_rx, I2S_DIR_RX, &ovr_cfg);
	if (ret < 0) {
		printk("RX reconfigure failed: %d\n", ret);
		return ret;
	}

	/* Two blocks of TX so the clocks outlive the overrun. */
	for (int i = 0; i < 2; i++) {
		ret = i2s_buf_write(dev_tx, tx_data, BLOCK_SIZE);
		if (ret < 0) {
			printk("TX write failed: %d\n", ret);
			return ret;
		}
	}

	ret = i2s_trigger(dev_rx, I2S_DIR_RX, I2S_TRIGGER_START);
	if (ret < 0) {
		printk("RX START failed: %d\n", ret);
		return ret;
	}

	ret = i2s_trigger(dev_tx, I2S_DIR_TX, I2S_TRIGGER_START);
	if (ret < 0) {
		printk("TX START failed: %d\n", ret);
		return ret;
	}

	/* Deliberately never read: the single block fills and stays filled. */
	k_msleep(20);

	(void)i2s_trigger(dev_tx, I2S_DIR_TX, I2S_TRIGGER_DROP);
	(void)i2s_trigger(dev_rx, I2S_DIR_RX, I2S_TRIGGER_DROP);

	free_blocks = k_mem_slab_num_free_get(&ovr_slab);
	printk("slab free after one forced overrun: %u/%u -> %s\n", free_blocks,
	       (unsigned int)OVR_BLOCKS,
	       (free_blocks == OVR_BLOCKS) ? "PASS" : "FAIL (a block was lost)");

	return (passed == ROUNDS && free_blocks == OVR_BLOCKS) ? 0 : -EIO;
}
