/*
 * Copyright (c) 2024
 * SPDX-License-Identifier: Apache-2.0
 */



#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <soc.h>

#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control/n32_clock_control.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#include <zephyr/arch/cpu.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/init.h>
#include <zephyr/drivers/clock_control.h>

#include <zephyr/linker/sections.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#define DT_DRV_COMPAT nsing_n32_uart

struct uart_n32_config
{
	uint32_t reg_base;
	uint32_t clk_cfg;
	const struct pinctrl_dev_config *pinctrl_config;

};

struct uart_n32_data
{
    uint32_t data_bits;
    uint32_t parity;
    uint32_t stop_bits;
	uint32_t baud_rate;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t cb; /**< Callback function pointer */
	void *cb_data;					  /**< Callback function arg */
#endif								  /* CONFIG_UART_INTERRUPT_DRIVEN */
};


static int uart_n32_init(const struct device *dev)
{
    int ret = 0;
    
    USART_InitType USART_InitStructure;
    const struct uart_n32_config *config = dev->config;
    const struct uart_n32_data   *data   = dev->data;
    
    ret = pinctrl_apply_state(config->pinctrl_config, PINCTRL_STATE_DEFAULT);
    
    if (ret < 0) {
		return ret;
    }

    ret = clock_control_on(DEVICE_DT_GET(DT_NODELABEL(rcc)), (clock_control_subsys_t *)(&config->clk_cfg));
    
    if (ret != 0) {
        return ret;
    }
	
    USART_InitStructure.BaudRate   = data->baud_rate;
    USART_InitStructure.WordLength = USART_WL_8B;
    USART_InitStructure.StopBits   = USART_STPB_1;
    USART_InitStructure.Parity     = USART_PE_NO;
    USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
    USART_InitStructure.Mode = USART_MODE_RX | USART_MODE_TX;

    USART_Init((USART_Module *)config->reg_base, &USART_InitStructure);
    USART_Enable((USART_Module *)config->reg_base, ENABLE);
 
	return 0;
}

static int uart_n32_poll_in(const struct device *dev, unsigned char *c)
{
    const struct uart_n32_config *config = dev->config;
	USART_Module *uart_port = (USART_Module*)config->reg_base;
   
    if (USART_GetFlagStatus(uart_port, USART_FLAG_RXDNE) == RESET){
		return -1;
	}
    else {
        *((unsigned char *)c) = (unsigned char)USART_ReceiveData(uart_port);
    }
    
	return 0;
}

static void uart_n32_poll_out(const struct device *dev, unsigned char c)
{
    const struct uart_n32_config *config = dev->config;
	USART_Module *uart_port = (USART_Module*)config->reg_base;

    
    while (USART_GetFlagStatus(uart_port, USART_FLAG_TXDE) == RESET)
        ;
    USART_SendData(uart_port, c);
}

static int uart_n32_err_check(const struct device *dev)
{
    const struct uart_n32_config *config = dev->config;
	USART_Module *uart = (USART_Module *)config->reg_base;
    uint32_t reg_val = uart->STS;
    
    
    int z_err = ((reg_val & USART_FLAG_OREF)? UART_ERROR_OVERRUN : 0) |
			    ((reg_val & USART_FLAG_PEF) ? UART_ERROR_PARITY : 0) |
		        ((reg_val & USART_FLAG_FEF) ? UART_ERROR_FRAMING : 0);

	USART_ClrFlag(uart, reg_val & (USART_FLAG_OREF | USART_FLAG_NEF | USART_FLAG_PEF | USART_FLAG_FEF));
    
    return (int)z_err;
}



static DEVICE_API(uart, uart_n32_driver_api) = {
	.poll_in = uart_n32_poll_in,
	.poll_out = uart_n32_poll_out,
	.err_check = uart_n32_err_check,

};


#define UART_N32_DEVICE(n)                \
    PINCTRL_DT_INST_DEFINE(n);            \
    const static struct uart_n32_config uart_n32_config_##n = {     \
		.reg_base = DT_INST_REG_ADDR(n),           \
		.clk_cfg  = DT_INST_CLOCKS_CELL(n, bits), \
		.pinctrl_config = PINCTRL_DT_INST_DEV_CONFIG_GET(n),  \
    };  \
    static struct uart_n32_data uart_n32_data_##n = {              \
        .baud_rate = DT_INST_PROP_OR(n, current_speed, 115200), \
		.data_bits = DT_INST_ENUM_IDX_OR(n, data_bits, UART_CFG_DATA_BITS_8), \
		.stop_bits = DT_INST_ENUM_IDX_OR(n, stop_bits, UART_CFG_STOP_BITS_1), \
		.parity	   = DT_INST_ENUM_IDX_OR(n, parity,	UART_CFG_PARITY_NONE), \
	};          \
    DEVICE_DT_INST_DEFINE(n,                            \
                          &uart_n32_init,                \
						  NULL,                           \
                          &uart_n32_data_##n,       \
						  &uart_n32_config_##n,        \
						  PRE_KERNEL_1,                   \
                          CONFIG_SERIAL_INIT_PRIORITY,    \
						  &uart_n32_driver_api);
                          
DT_INST_FOREACH_STATUS_OKAY(UART_N32_DEVICE);

