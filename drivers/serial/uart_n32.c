/* Copyright (c) 2024 Nations Technologies
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nsing_n32_uart

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/irq.h>
#include <soc.h>

struct uart_n32_config {
	uintptr_t reg_base;
	uint32_t clk_cfg;
	const struct pinctrl_dev_config *pinctrl;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	void (*irq_config_func)(const struct device *dev);
#endif
};

struct uart_n32_data {
	struct uart_config uart_cfg;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t cb;
	void *cb_data;
#endif
};

static int uart_n32_apply_config(const struct device *dev, const struct uart_config *cfg)
{
	const struct uart_n32_config *config = dev->config;
	USART_InitType init = { 0 };

	if (cfg->baudrate == 0U) {
		return -EINVAL;
	}
	switch (cfg->parity) {
	case UART_CFG_PARITY_NONE: init.Parity = USART_PE_NO; break;
	case UART_CFG_PARITY_ODD: init.Parity = USART_PE_ODD; break;
	case UART_CFG_PARITY_EVEN: init.Parity = USART_PE_EVEN; break;
	default: return -ENOTSUP;
	}

	/*
	 * The N32 word-length bit selects the complete frame width. With
	 * parity enabled its MSB is the parity bit, not a data bit.
	 */
	if (cfg->parity == UART_CFG_PARITY_NONE) {
		switch (cfg->data_bits) {
		case UART_CFG_DATA_BITS_8: init.WordLength = USART_WL_8B; break;
		case UART_CFG_DATA_BITS_9: init.WordLength = USART_WL_9B; break;
		default: return -ENOTSUP;
		}
	} else {
		switch (cfg->data_bits) {
		case UART_CFG_DATA_BITS_7: init.WordLength = USART_WL_8B; break;
		case UART_CFG_DATA_BITS_8: init.WordLength = USART_WL_9B; break;
		default: return -ENOTSUP;
		}
	}
	switch (cfg->stop_bits) {
	case UART_CFG_STOP_BITS_0_5: init.StopBits = USART_STPB_0_5; break;
	case UART_CFG_STOP_BITS_1: init.StopBits = USART_STPB_1; break;
	case UART_CFG_STOP_BITS_1_5: init.StopBits = USART_STPB_1_5; break;
	case UART_CFG_STOP_BITS_2: init.StopBits = USART_STPB_2; break;
	default: return -EINVAL;
	}
	switch (cfg->flow_ctrl) {
	case UART_CFG_FLOW_CTRL_NONE: init.HardwareFlowControl = USART_HFCTRL_NONE; break;
	case UART_CFG_FLOW_CTRL_RTS_CTS: init.HardwareFlowControl = USART_HFCTRL_RTS_CTS; break;
	default: return -ENOTSUP;
	}

	init.BaudRate = cfg->baudrate;
	init.Mode = USART_MODE_RX | USART_MODE_TX;
	USART_Enable((USART_Module *)config->reg_base, DISABLE);
	USART_Init((USART_Module *)config->reg_base, &init);
	USART_Enable((USART_Module *)config->reg_base, ENABLE);
	return 0;
}

static int uart_n32_init(const struct device *dev)
{
	const struct uart_n32_config *config = dev->config;
	struct uart_n32_data *data = dev->data;
	int ret;

	ret = pinctrl_apply_state(config->pinctrl, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}
	ret = clock_control_on(DEVICE_DT_GET(DT_NODELABEL(rcc)),
			       (clock_control_subsys_t)&config->clk_cfg);
	if (ret != 0) {
		return ret;
	}
	ret = uart_n32_apply_config(dev, &data->uart_cfg);
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	if (ret == 0) {
		config->irq_config_func(dev);
	}
#endif
	return ret;
}

static int uart_n32_poll_in(const struct device *dev, unsigned char *c)
{
	const struct uart_n32_config *config = dev->config;
	USART_Module *uart = (USART_Module *)config->reg_base;

	if (USART_GetFlagStatus(uart, USART_FLAG_RXDNE) == RESET) {
		return -1;
	}
	*c = (uint8_t)USART_ReceiveData(uart);
	return 0;
}

static void uart_n32_poll_out(const struct device *dev, unsigned char c)
{
	const struct uart_n32_config *config = dev->config;
	USART_Module *uart = (USART_Module *)config->reg_base;

	while (USART_GetFlagStatus(uart, USART_FLAG_TXDE) == RESET) {
	}
	USART_SendData(uart, c);
}

static int uart_n32_err_check(const struct device *dev)
{
	const struct uart_n32_config *config = dev->config;
	USART_Module *uart = (USART_Module *)config->reg_base;
	uint32_t status = uart->STS;
	int errors = 0;

	if ((status & USART_FLAG_OREF) != 0U) { errors |= UART_ERROR_OVERRUN; }
	if ((status & USART_FLAG_NEF) != 0U) { errors |= UART_ERROR_NOISE; }
	if ((status & USART_FLAG_PEF) != 0U) { errors |= UART_ERROR_PARITY; }
	if ((status & USART_FLAG_FEF) != 0U) { errors |= UART_ERROR_FRAMING; }

	/*
	 * N32 clears PE, FE, NE and ORE only after an STS read followed by a
	 * DAT read.  The STS access above is deliberately kept before this
	 * volatile DAT access.  Reading DAT also consumes the erroneous frame,
	 * which must not be returned to the caller as valid input.
	 */
	if (errors != 0) {
		(void)uart->DAT;
	}

	return errors;
}

#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
static int uart_n32_configure(const struct device *dev, const struct uart_config *cfg)
{
	struct uart_n32_data *data = dev->data;
	int ret = uart_n32_apply_config(dev, cfg);

	if (ret == 0) { data->uart_cfg = *cfg; }
	return ret;
}

static int uart_n32_config_get(const struct device *dev, struct uart_config *cfg)
{
	*cfg = ((const struct uart_n32_data *)dev->data)->uart_cfg;
	return 0;
}
#endif

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void uart_n32_isr(const struct device *dev)
{
	struct uart_n32_data *data = dev->data;
	if (data->cb != NULL) { data->cb(dev, data->cb_data); }
}

static int uart_n32_fifo_fill(const struct device *dev, const uint8_t *tx, int size)
{
	const struct uart_n32_config *config = dev->config;
	USART_Module *uart = (USART_Module *)config->reg_base;
	if ((size <= 0) || (USART_GetFlagStatus(uart, USART_FLAG_TXDE) == RESET)) { return 0; }
	USART_SendData(uart, tx[0]);
	return 1; /* N32 USART has a one-byte data register. */
}

static int uart_n32_fifo_read(const struct device *dev, uint8_t *rx, const int size)
{
	const struct uart_n32_config *config = dev->config;
	USART_Module *uart = (USART_Module *)config->reg_base;
	if ((size <= 0) || (USART_GetFlagStatus(uart, USART_FLAG_RXDNE) == RESET)) { return 0; }
	rx[0] = (uint8_t)USART_ReceiveData(uart);
	return 1;
}

static void uart_n32_irq_tx_enable(const struct device *dev)
{
	USART_ConfigInt((USART_Module *)((const struct uart_n32_config *)dev->config)->reg_base,
			USART_INT_TXDE, ENABLE);
}
static void uart_n32_irq_tx_disable(const struct device *dev)
{
	USART_ConfigInt((USART_Module *)((const struct uart_n32_config *)dev->config)->reg_base,
			USART_INT_TXDE, DISABLE);
}
static int uart_n32_irq_tx_ready(const struct device *dev)
{
	USART_Module *uart = (USART_Module *)((const struct uart_n32_config *)dev->config)->reg_base;
	return USART_GetIntStatus(uart, USART_INT_TXDE) == SET;
}
static void uart_n32_irq_rx_enable(const struct device *dev)
{
	USART_ConfigInt((USART_Module *)((const struct uart_n32_config *)dev->config)->reg_base,
			USART_INT_RXDNE, ENABLE);
}
static void uart_n32_irq_rx_disable(const struct device *dev)
{
	USART_ConfigInt((USART_Module *)((const struct uart_n32_config *)dev->config)->reg_base,
			USART_INT_RXDNE, DISABLE);
}
static int uart_n32_irq_tx_complete(const struct device *dev)
{
	return USART_GetFlagStatus((USART_Module *)((const struct uart_n32_config *)dev->config)->reg_base,
			USART_FLAG_TXC) == SET;
}
static int uart_n32_irq_rx_ready(const struct device *dev)
{
	USART_Module *uart = (USART_Module *)((const struct uart_n32_config *)dev->config)->reg_base;
	return USART_GetIntStatus(uart, USART_INT_RXDNE) == SET;
}
static void uart_n32_irq_err_enable(const struct device *dev)
{
	USART_Module *uart = (USART_Module *)((const struct uart_n32_config *)dev->config)->reg_base;
	USART_ConfigInt(uart, USART_INT_PEF, ENABLE);
	USART_ConfigInt(uart, USART_INT_ERRF, ENABLE);
}
static void uart_n32_irq_err_disable(const struct device *dev)
{
	USART_Module *uart = (USART_Module *)((const struct uart_n32_config *)dev->config)->reg_base;
	USART_ConfigInt(uart, USART_INT_PEF, DISABLE);
	USART_ConfigInt(uart, USART_INT_ERRF, DISABLE);
}
static int uart_n32_irq_is_pending(const struct device *dev)
{
	USART_Module *uart = (USART_Module *)((const struct uart_n32_config *)dev->config)->reg_base;
	return (USART_GetIntStatus(uart, USART_INT_TXDE) == SET) ||
	       (USART_GetIntStatus(uart, USART_INT_RXDNE) == SET) ||
	       (USART_GetIntStatus(uart, USART_INT_PEF) == SET) ||
	       (USART_GetIntStatus(uart, USART_INT_ERRF) == SET);
}
static void uart_n32_irq_update(const struct device *dev) { ARG_UNUSED(dev); }
static void uart_n32_irq_callback_set(const struct device *dev,
				      uart_irq_callback_user_data_t cb, void *cb_data)
{
	struct uart_n32_data *data = dev->data;
	data->cb = cb;
	data->cb_data = cb_data;
}
#endif

static DEVICE_API(uart, uart_n32_driver_api) = {
	.poll_in = uart_n32_poll_in,
	.poll_out = uart_n32_poll_out,
	.err_check = uart_n32_err_check,
#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
	.configure = uart_n32_configure,
	.config_get = uart_n32_config_get,
#endif
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = uart_n32_fifo_fill,
	.fifo_read = uart_n32_fifo_read,
	.irq_tx_enable = uart_n32_irq_tx_enable,
	.irq_tx_disable = uart_n32_irq_tx_disable,
	.irq_tx_ready = uart_n32_irq_tx_ready,
	.irq_rx_enable = uart_n32_irq_rx_enable,
	.irq_rx_disable = uart_n32_irq_rx_disable,
	.irq_tx_complete = uart_n32_irq_tx_complete,
	.irq_rx_ready = uart_n32_irq_rx_ready,
	.irq_err_enable = uart_n32_irq_err_enable,
	.irq_err_disable = uart_n32_irq_err_disable,
	.irq_is_pending = uart_n32_irq_is_pending,
	.irq_update = uart_n32_irq_update,
	.irq_callback_set = uart_n32_irq_callback_set,
#endif
};

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
#define UART_N32_IRQ_DECL(n) static void uart_n32_irq_config_##n(const struct device *dev);
#define UART_N32_IRQ_CONFIG(n) .irq_config_func = uart_n32_irq_config_##n,
#define UART_N32_IRQ_CONNECT(n)                                                     \
	static void uart_n32_irq_config_##n(const struct device *dev)                 \
	{                                                                                \
		ARG_UNUSED(dev);                                                         \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), uart_n32_isr,    \
			    DEVICE_DT_INST_GET(n), 0);                                        \
		irq_enable(DT_INST_IRQN(n));                                             \
	}
#else
#define UART_N32_IRQ_DECL(n)
#define UART_N32_IRQ_CONFIG(n)
#define UART_N32_IRQ_CONNECT(n)
#endif

#define UART_N32_DEVICE(n)                                                        \
	PINCTRL_DT_INST_DEFINE(n);                                                   \
	UART_N32_IRQ_DECL(n)                                                         \
	static struct uart_n32_data uart_n32_data_##n = {                            \
		.uart_cfg = {                                                          \
			.baudrate = DT_INST_PROP_OR(n, current_speed, 115200),          \
			.parity = DT_INST_ENUM_IDX_OR(n, parity, UART_CFG_PARITY_NONE),  \
			.stop_bits = DT_INST_ENUM_IDX_OR(n, stop_bits, UART_CFG_STOP_BITS_1), \
			.data_bits = DT_INST_PROP_OR(n, data_bits, 8) - 5,                \
			.flow_ctrl = DT_INST_PROP_OR(n, hw_flow_control, 0) ?             \
				UART_CFG_FLOW_CTRL_RTS_CTS : UART_CFG_FLOW_CTRL_NONE,        \
		},                                                                      \
	};                                                                              \
	static const struct uart_n32_config uart_n32_config_##n = {                   \
		.reg_base = DT_INST_REG_ADDR(n),                                       \
		.clk_cfg = DT_INST_CLOCKS_CELL(n, bits),                               \
		.pinctrl = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                          \
		UART_N32_IRQ_CONFIG(n)                                                 \
	};                                                                              \
	DEVICE_DT_INST_DEFINE(n, uart_n32_init, NULL, &uart_n32_data_##n,             \
			      &uart_n32_config_##n, PRE_KERNEL_1,                         \
			      CONFIG_SERIAL_INIT_PRIORITY, &uart_n32_driver_api);         \
	UART_N32_IRQ_CONNECT(n)

DT_INST_FOREACH_STATUS_OKAY(UART_N32_DEVICE)
