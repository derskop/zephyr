/*
 * Copyright (c) 2026 Nations Technologies Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#define UART_DEV DEVICE_DT_GET(DT_CHOSEN(zephyr_uart_test))
#define UART_PEER DEVICE_DT_GET(DT_CHOSEN(zephyr_uart_peer))

static struct uart_config boot_config;
K_SEM_DEFINE(tx_irq_done, 0, 1);
static volatile int fifo_bytes_written;
static volatile int fifo_bytes_read;
static uint8_t fifo_rx_byte;

/* A one-byte N32 data register needs one TX-ready interrupt per byte.  Using
 * 1 KiB in both directions exercises repeated TXDE/RXDNE handling instead of
 * only proving that the first interrupt arrives. */
#define IRQ_STRESS_LEN 1024U

struct irq_stream {
	const uint8_t *tx;
	uint8_t *rx;
	size_t tx_pos;
	size_t rx_pos;
	int errors;
	bool rx_done;
	struct k_sem done;
};

static uint8_t stress_tx_1[IRQ_STRESS_LEN];
static uint8_t stress_tx_2[IRQ_STRESS_LEN];
static uint8_t stress_rx_1[IRQ_STRESS_LEN];
static uint8_t stress_rx_2[IRQ_STRESS_LEN];

static void tx_irq_callback(const struct device *dev, void *user_data)
{
	static const uint8_t byte = 0U;

	ARG_UNUSED(user_data);

	uart_irq_update(dev);
	if (uart_irq_tx_ready(dev) && (fifo_bytes_written == 0)) {
		fifo_bytes_written = uart_fifo_fill(dev, &byte, 1);
		uart_irq_tx_disable(dev);
		k_sem_give(&tx_irq_done);
	}
}

static void rx_irq_callback(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	uart_irq_update(dev);
	if (uart_irq_rx_ready(dev)) {
		fifo_bytes_read = uart_fifo_read(dev, &fifo_rx_byte, 1);
		uart_irq_rx_disable(dev);
		k_sem_give(&tx_irq_done);
	}
}

static void irq_stream_callback(const struct device *dev, void *user_data)
{
	struct irq_stream *stream = user_data;
	uint8_t byte;
	int count;

	while (true) {
		uart_irq_update(dev);
		if (uart_irq_is_pending(dev) == 0) {
			break;
		}

		if (uart_irq_rx_ready(dev)) {
			do {
				count = uart_fifo_read(dev, &byte, 1);
				if ((count == 1) && (stream->rx_pos < IRQ_STRESS_LEN)) {
					stream->rx[stream->rx_pos++] = byte;
				}
			} while (count > 0);

			if ((stream->rx_pos == IRQ_STRESS_LEN) && !stream->rx_done) {
				stream->rx_done = true;
				k_sem_give(&stream->done);
			}
		}

		if (uart_irq_tx_ready(dev)) {
			if (stream->tx_pos < IRQ_STRESS_LEN) {
				count = uart_fifo_fill(dev, &stream->tx[stream->tx_pos],
						       IRQ_STRESS_LEN - stream->tx_pos);
				if (count > 0) {
					stream->tx_pos += count;
				}
			}
			if (stream->tx_pos == IRQ_STRESS_LEN) {
				uart_irq_tx_disable(dev);
			}
		}

		if (uart_err_check(dev) != 0) {
			stream->errors++;
		}
	}
}

static void discard_rx_data(const struct device *dev)
{
	unsigned char byte;

	while (uart_poll_in(dev, &byte) == 0) {
	}
}

static bool poll_transfer_byte(const struct device *tx_dev,
			       const struct device *rx_dev, uint8_t tx, uint8_t *rx)
{
	uart_poll_out(tx_dev, tx);
	for (int i = 0; i < 1000; i++) {
		if (uart_poll_in(rx_dev, rx) == 0) {
			return true;
		}
		k_busy_wait(10);
	}
	return false;
}

ZTEST(uart_nsing, test_device_and_boot_configuration)
{
	TC_PRINT("N32 UART API test revision 12\n");
	zassert_true(device_is_ready(UART_DEV), "UART device is not ready");
	zassert_true(device_is_ready(UART_PEER), "peer UART device is not ready");
	zassert_ok(uart_config_get(UART_DEV, &boot_config));
	zassert_equal(boot_config.baudrate, 115200U);
	zassert_equal(boot_config.data_bits, UART_CFG_DATA_BITS_8);
	zassert_equal(boot_config.parity, UART_CFG_PARITY_NONE);
	zassert_equal(boot_config.stop_bits, UART_CFG_STOP_BITS_1);
	zassert_equal(boot_config.flow_ctrl, UART_CFG_FLOW_CTRL_NONE);
}

ZTEST(uart_nsing, test_polling_loopback)
{
	uint8_t received;

	discard_rx_data(UART_PEER);
	if (!poll_transfer_byte(UART_DEV, UART_PEER, 0x5a, &received)) {
		TC_PRINT("SKIP: connect USART1 PA9 (TX) to USART2 PA3 (RX)\n");
		ztest_test_skip();
		return;
	}
	zassert_equal(received, 0x5a, "polling loopback data mismatch");
}

ZTEST(uart_nsing, test_interrupt_fifo_rx_api)
{
	uint8_t probe;
	int ret;

	/* Detect the USART2-PA2 to USART1-PA10 connection without failing an
	 * unattended regression run that does not have the test wiring installed. */
	discard_rx_data(UART_DEV);
	if (!poll_transfer_byte(UART_PEER, UART_DEV, 0xa5, &probe)) {
		TC_PRINT("SKIP: connect USART2 PA2 (TX) to USART1 PA10 (RX)\n");
		ztest_test_skip();
		return;
	}
	zassert_equal(probe, 0xa5, "loopback probe data mismatch");

	fifo_bytes_read = 0;
	fifo_rx_byte = 0;
	k_sem_reset(&tx_irq_done);
	uart_irq_callback_user_data_set(UART_DEV, rx_irq_callback, NULL);
	uart_irq_tx_disable(UART_DEV);
	uart_irq_err_disable(UART_DEV);
	uart_irq_rx_enable(UART_DEV);
	uart_poll_out(UART_PEER, 0x3c);

	ret = k_sem_take(&tx_irq_done, K_MSEC(100));
	uart_irq_rx_disable(UART_DEV);
	uart_irq_callback_user_data_set(UART_DEV, NULL, NULL);

	zassert_ok(ret, "RX-ready interrupt did not arrive");
	zassert_equal(fifo_bytes_read, 1, "uart_fifo_read() did not read one byte");
	zassert_equal(fifo_rx_byte, 0x3c, "RX interrupt loopback data mismatch");
}

ZTEST(uart_nsing, test_runtime_configuration)
{
	struct uart_config requested = boot_config;
	struct uart_config read_back;

	/* This checks the N32 9-bit frame mode: 8 data bits plus even parity. */
	requested.parity = UART_CFG_PARITY_EVEN;
	requested.data_bits = UART_CFG_DATA_BITS_8;
	requested.stop_bits = UART_CFG_STOP_BITS_1;
	requested.flow_ctrl = UART_CFG_FLOW_CTRL_NONE;

	zassert_ok(uart_configure(UART_DEV, &requested));
	zassert_ok(uart_config_get(UART_DEV, &read_back));
	zassert_mem_equal(&read_back, &requested, sizeof(requested));

	/* Restore 8N1 before ztest emits further console output. */
	zassert_ok(uart_configure(UART_DEV, &boot_config));
}

ZTEST(uart_nsing, test_unsupported_configuration)
{
	struct uart_config invalid = boot_config;

	/* N32 supports 7 data bits only when parity occupies the eighth bit. */
	invalid.parity = UART_CFG_PARITY_NONE;
	invalid.data_bits = UART_CFG_DATA_BITS_7;
	zassert_equal(uart_configure(UART_DEV, &invalid), -ENOTSUP);
}

ZTEST(uart_nsing, test_rts_cts_configuration)
{
	struct uart_config requested = boot_config;
	struct uart_config peer_boot;
	struct uart_config peer_requested;
	struct uart_config read_back;
	static const uint8_t tx1[] = "USART1 -> USART2 using CTS and RTS";
	static const uint8_t tx2[] = "USART2 -> USART1 using CTS and RTS";
	uint8_t received;

	zassert_ok(uart_config_get(UART_PEER, &peer_boot));
	requested.flow_ctrl = UART_CFG_FLOW_CTRL_RTS_CTS;
	peer_requested = peer_boot;
	peer_requested.flow_ctrl = UART_CFG_FLOW_CTRL_RTS_CTS;
	zassert_ok(uart_configure(UART_DEV, &requested));
	zassert_ok(uart_configure(UART_PEER, &peer_requested));
	zassert_ok(uart_config_get(UART_DEV, &read_back));
	zassert_equal(read_back.flow_ctrl, UART_CFG_FLOW_CTRL_RTS_CTS);

	/* Follow the vendor HardwareFlowCtrl example: transfer and compare a
	 * buffer in each direction while both USARTs have CTS and RTS enabled. */
	discard_rx_data(UART_DEV);
	discard_rx_data(UART_PEER);
	for (size_t i = 0; i < sizeof(tx1); i++) {
		zassert_true(poll_transfer_byte(UART_DEV, UART_PEER, tx1[i], &received),
			     "USART1 to USART2 timed out at byte %u", (unsigned int)i);
		zassert_equal(received, tx1[i], "USART1 to USART2 mismatch at byte %u",
			      (unsigned int)i);
	}
	for (size_t i = 0; i < sizeof(tx2); i++) {
		zassert_true(poll_transfer_byte(UART_PEER, UART_DEV, tx2[i], &received),
			     "USART2 to USART1 timed out at byte %u", (unsigned int)i);
		zassert_equal(received, tx2[i], "USART2 to USART1 mismatch at byte %u",
			      (unsigned int)i);
	}

	zassert_ok(uart_configure(UART_DEV, &boot_config));
	zassert_ok(uart_configure(UART_PEER, &peer_boot));
}

ZTEST(uart_nsing, test_no_spurious_error)
{
	discard_rx_data(UART_DEV);
	zassert_equal(uart_err_check(UART_DEV), 0,
		      "UART reported an error without an injected bad frame");
}

ZTEST(uart_nsing, test_interrupt_fifo_tx_api)
{
	int ret;
	bool tx_complete = false;

	fifo_bytes_written = 0;
	k_sem_reset(&tx_irq_done);
	uart_irq_callback_user_data_set(UART_DEV, tx_irq_callback, NULL);
	uart_irq_rx_disable(UART_DEV);
	uart_irq_err_disable(UART_DEV);
	uart_irq_tx_enable(UART_DEV);

	ret = k_sem_take(&tx_irq_done, K_MSEC(100));
	uart_irq_tx_disable(UART_DEV);
	uart_irq_callback_user_data_set(UART_DEV, NULL, NULL);

	zassert_ok(ret, "TX-ready interrupt did not arrive");
	zassert_equal(fifo_bytes_written, 1, "uart_fifo_fill() did not write one byte");

	/* fifo_fill() only writes DAT. TXC becomes set after the complete frame,
	 * including its stop bit, has left the pin. */
	for (int i = 0; i < 1000; i++) {
		if (uart_irq_tx_complete(UART_DEV)) {
			tx_complete = true;
			break;
		}
		k_busy_wait(10);
	}
	zassert_true(tx_complete, "TX did not complete within 10 ms");
}

ZTEST(uart_nsing, test_interrupt_fifo_stress)
{
	struct uart_config cfg1;
	struct uart_config cfg2;
	struct irq_stream stream1 = {
		.tx = stress_tx_1,
		.rx = stress_rx_1,
	};
	struct irq_stream stream2 = {
		.tx = stress_tx_2,
		.rx = stress_rx_2,
	};
	bool tx1_complete = false;
	bool tx2_complete = false;

	/* The stress test isolates IRQ/FIFO behavior. RTS/CTS is independently
	 * exercised by test_rts_cts_configuration(). */
	zassert_ok(uart_config_get(UART_DEV, &cfg1));
	zassert_ok(uart_config_get(UART_PEER, &cfg2));
	cfg1.flow_ctrl = UART_CFG_FLOW_CTRL_NONE;
	cfg2.flow_ctrl = UART_CFG_FLOW_CTRL_NONE;
	zassert_ok(uart_configure(UART_DEV, &cfg1));
	zassert_ok(uart_configure(UART_PEER, &cfg2));

	for (size_t i = 0; i < IRQ_STRESS_LEN; i++) {
		stress_tx_1[i] = (uint8_t)i;
		stress_tx_2[i] = (uint8_t)(0xffU - i);
	}
	memset(stress_rx_1, 0, sizeof(stress_rx_1));
	memset(stress_rx_2, 0, sizeof(stress_rx_2));
	k_sem_init(&stream1.done, 0, 1);
	k_sem_init(&stream2.done, 0, 1);
	discard_rx_data(UART_DEV);
	discard_rx_data(UART_PEER);

	zassert_ok(uart_irq_callback_user_data_set(UART_DEV, irq_stream_callback, &stream1));
	zassert_ok(uart_irq_callback_user_data_set(UART_PEER, irq_stream_callback, &stream2));
	uart_irq_err_enable(UART_DEV);
	uart_irq_err_enable(UART_PEER);
	uart_irq_rx_enable(UART_DEV);
	uart_irq_rx_enable(UART_PEER);
	uart_irq_tx_enable(UART_DEV);
	uart_irq_tx_enable(UART_PEER);

	zassert_ok(k_sem_take(&stream1.done, K_SECONDS(2)),
		   "USART1 did not receive 1 KiB within 2 seconds");
	zassert_ok(k_sem_take(&stream2.done, K_SECONDS(2)),
		   "USART2 did not receive 1 KiB within 2 seconds");

	uart_irq_tx_disable(UART_DEV);
	uart_irq_tx_disable(UART_PEER);
	uart_irq_rx_disable(UART_DEV);
	uart_irq_rx_disable(UART_PEER);
	uart_irq_err_disable(UART_DEV);
	uart_irq_err_disable(UART_PEER);
	uart_irq_callback_user_data_set(UART_DEV, NULL, NULL);
	uart_irq_callback_user_data_set(UART_PEER, NULL, NULL);

	for (int i = 0; i < 1000; i++) {
		if (uart_irq_tx_complete(UART_DEV) && uart_irq_tx_complete(UART_PEER)) {
			tx1_complete = true;
			tx2_complete = true;
			break;
		}
		k_busy_wait(10);
	}
	zassert_true(tx1_complete && tx2_complete, "a stress-test TX frame did not complete");
	zassert_equal(stream1.tx_pos, IRQ_STRESS_LEN, "USART1 TX byte count mismatch");
	zassert_equal(stream2.tx_pos, IRQ_STRESS_LEN, "USART2 TX byte count mismatch");
	zassert_equal(stream1.rx_pos, IRQ_STRESS_LEN, "USART1 RX byte count mismatch");
	zassert_equal(stream2.rx_pos, IRQ_STRESS_LEN, "USART2 RX byte count mismatch");
	zassert_equal(stream1.errors, 0, "USART1 reported %d RX errors", stream1.errors);
	zassert_equal(stream2.errors, 0, "USART2 reported %d RX errors", stream2.errors);
	zassert_mem_equal(stress_rx_1, stress_tx_2, IRQ_STRESS_LEN,
			  "USART1 received data differs from USART2 stream");
	zassert_mem_equal(stress_rx_2, stress_tx_1, IRQ_STRESS_LEN,
			  "USART2 received data differs from USART1 stream");
}

ZTEST_SUITE(uart_nsing, NULL, NULL, NULL, NULL, NULL);
