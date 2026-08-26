/*
 * Copyright (c) 2023, Quincy.W <wangqyfm@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nsing_n32_gpio
#include <errno.h>

//#include <zephyr/pm/device.h>


#include <zephyr/drivers/clock_control/n32_clock_control.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/sys_io.h>
#include <soc.h>

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/drivers/gpio.h>

struct gpio_n32_config
{
	uint32_t base;
	uint32_t clkid;
	void (*irq_connect)(void);
};

struct gpio_n32_data
{
	struct gpio_driver_data common;
	sys_slist_t callbacks;
};

#define DEV_CFG(dev) ((const struct gpio_n32_config *)(dev)->config)
#define DEV_DATA(dev) ((struct gpio_n32_data *)(dev)->data)

static inline void n32_exti_isr(const void *arg)
{
	;
}

static int gpio_n32_init(const struct device *dev)
{
	const struct gpio_n32_config *cfg = DEV_CFG(dev);
	const struct device *clk = DEVICE_DT_GET(DT_NODELABEL(rcc));
	int ret;

	ret = clock_control_on(clk, (clock_control_subsys_t *)&cfg->clkid);

	if (cfg->irq_connect)
	{
		cfg->irq_connect();
	}

	return ret;
	
}

#define _GPIO_MODE_INPUT 0
#define _GPIO_MODE_OUTPUT_2MHZ 1
#define _GPIO_MODE_OUTPUT_10MHZ 2
#define _GPIO_MODE_OUTPUT_50MHZ 3

#define _GPIO_CFG_INPUT_AN (0 << 4)
#define _GPIO_CFG_INPUT_FLOAT (1 << 4)
#define _GPIO_CFG_INPUT_PD_PU (2 << 4)

#define _GPIO_CFG_OUTPUT_OD (1 << 4)

static int gpio_n32_pin_configure(const struct device *dev,
								  gpio_pin_t pin,
								  gpio_flags_t flags)
{
	GPIO_InitType init_structure;
	const struct gpio_n32_config *cfg = DEV_CFG(dev);
	GPIO_Module *gpio = (GPIO_Module *)cfg->base;

	GPIO_InitStruct(&init_structure);

	if ((flags & GPIO_OUTPUT) && (flags & GPIO_INPUT))
	{
		return -ENOTSUP;
	}

	if ((flags & GPIO_OUTPUT_INIT_HIGH))
	{
		gpio->PBSC |= BIT(pin);
	}
	else if ((flags & GPIO_OUTPUT_INIT_LOW))
	{
		gpio->PBC |= BIT(pin);
	}

	if (flags & GPIO_OUTPUT)
	{
		init_structure.GPIO_Mode = (flags & GPIO_SINGLE_ENDED) ? GPIO_Mode_Out_OD : GPIO_Mode_Out_PP;
	}
	else if (flags & GPIO_INPUT)
	{
		if (flags & (GPIO_PULL_UP | GPIO_PULL_DOWN))
		{
			init_structure.GPIO_Mode = (flags & GPIO_PULL_UP) ? GPIO_Mode_IPU : GPIO_Mode_IPD;
		}
		else
		{
			init_structure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		}
	}

	init_structure.Pin = 1 << pin;

	GPIO_InitPeripheral(gpio, &init_structure);

	return 0;
}

static int gpio_n32_port_get_raw(const struct device *dev,
								 gpio_port_value_t *value)
{
	const struct gpio_n32_config *cfg = DEV_CFG(dev);
	GPIO_Module *gpio = (GPIO_Module *)cfg->base;

	*value = gpio->PID;

	return 0;
}

/* API implementation: port_set_masked_raw */
static int gpio_n32_port_set_masked_raw(const struct device *dev,
										gpio_port_pins_t mask,
										gpio_port_value_t value)
{
	const struct gpio_n32_config *cfg = DEV_CFG(dev);
	GPIO_Module *gpio = (GPIO_Module *)cfg->base;

	gpio->POD = (gpio->POD & ~mask) | (value & mask);

	return 0;
}

static int gpio_n32_port_set_bits_raw(const struct device *dev,
									  gpio_port_pins_t mask)
{
	const struct gpio_n32_config *cfg = DEV_CFG(dev);
	GPIO_Module *gpio = (GPIO_Module *)cfg->base;

	gpio->PBSC = mask;

	return 0;
}

static int gpio_n32_port_clear_bits_raw(const struct device *dev,
										gpio_port_pins_t mask)
{
	const struct gpio_n32_config *cfg = DEV_CFG(dev);
	GPIO_Module *gpio = (GPIO_Module *)cfg->base;

	gpio->PBC = mask;

	return 0;
}

static int gpio_n32_port_toggle_bits(const struct device *dev,
									 gpio_port_pins_t mask)
{
	const struct gpio_n32_config *cfg = DEV_CFG(dev);
	GPIO_Module *gpio = (GPIO_Module *)cfg->base;

	gpio->POD = (gpio->POD ^ mask);

	return 0;
}

// static void gpio_n32_irq_handler(const struct device *dev)
// {
// 	struct gpio_n32_data *data = dev->data;

// 	gpio_fire_callbacks(&data->callbacks, dev, 0); // <TODO:
// }

static int gpio_n32_pin_interrupt_configure(const struct device *dev,
											gpio_pin_t pin,
											enum gpio_int_mode mode,
											enum gpio_int_trig trig)
{
	int rc = -ENOTSUP;

	switch (mode)
	{
	case GPIO_INT_MODE_DISABLED:
		break;

	case GPIO_INT_MODE_LEVEL:

		break;

	case GPIO_INT_MODE_EDGE:

		break;

	default:
		rc = -ENOTSUP;
		break;
	}

	return rc;
}

static int gpio_n32_manage_callback(const struct device *dev,
									struct gpio_callback *callback,
									bool set)
{
	struct gpio_n32_data *data = dev->data;

	return gpio_manage_callback(&data->callbacks, callback, set);
}

static const struct gpio_driver_api gpio_n32_api = {
	.pin_configure = gpio_n32_pin_configure,
	.port_get_raw = gpio_n32_port_get_raw,
	.port_set_masked_raw = gpio_n32_port_set_masked_raw,
	.port_set_bits_raw = gpio_n32_port_set_bits_raw,
	.port_clear_bits_raw = gpio_n32_port_clear_bits_raw,
	.port_toggle_bits = gpio_n32_port_toggle_bits,
	.pin_interrupt_configure = gpio_n32_pin_interrupt_configure,
	.manage_callback = gpio_n32_manage_callback};

#define GPIO_N32_INIT(n)                                           \
	static const struct gpio_n32_config gpio_n32_config_##n = {    \
		.base = DT_INST_REG_ADDR(n),                               \
		.clkid = DT_INST_CLOCKS_CELL(n, bits),			           \
		.irq_connect = NULL,                                       \
	};                                                             \
	static struct gpio_n32_data gpio_n32_data_##n;                 \
                                                                   \
	DEVICE_DT_INST_DEFINE(n, gpio_n32_init,                        \
						  NULL,                                    \
						  &gpio_n32_data_##n,                      \
						  &gpio_n32_config_##n,                    \
						  PRE_KERNEL_1, CONFIG_GPIO_INIT_PRIORITY, \
						  &gpio_n32_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_N32_INIT)

static int afio_n32_init(void)
{
	const struct device *clk = DEVICE_DT_GET(DT_NODELABEL(rcc));
	
	return clock_control_on(clk, (clock_control_subsys_t *)N32_CLOCK_AFIO);
}





SYS_INIT(afio_n32_init, PRE_KERNEL_1, CONFIG_GPIO_INIT_PRIORITY);