/*
 * Copyright (c) 2024 Nations Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/n32_clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/drivers/interrupt_controller/gpio_intc_n32.h>
#include <zephyr/sys/util.h>

#include <n32_gpio_shared.h>
#include <pinctrl_soc.h>

#include <n32g45x.h>

#define DT_DRV_COMPAT nsing_n32_gpio

struct gpio_n32_config {
	GPIO_Module *base;
	uint32_t clkid;
	uint8_t port;
	struct gpio_driver_config common;
};

struct gpio_n32_data {
	struct gpio_driver_data common;
	sys_slist_t callbacks;
};

#define DEV_CFG(dev) ((const struct gpio_n32_config *)(dev)->config)
#define DEV_DATA(dev) ((struct gpio_n32_data *)(dev)->data)

static int gpio_n32_init(const struct device *dev)
{
	const struct gpio_n32_config *cfg = DEV_CFG(dev);

	/* Enable the port clock through the clock control driver */
	return clock_control_on(DEVICE_DT_GET(DT_NODELABEL(rcc)),
				(clock_control_subsys_t *)&cfg->clkid);
}

/* Convert Zephyr GPIO flags to a pin configuration (pinctrl_soc_pin_t),
 * reusing the same bit field encoding as the pinctrl driver.
 */
static int gpio_n32_flags_to_conf(gpio_flags_t flags, pinctrl_soc_pin_t *pincfg)
{
	*pincfg = 0;

	if ((flags & GPIO_INPUT) && (flags & GPIO_OUTPUT)) {
		return -ENOTSUP;
	}
	if ((flags & GPIO_PULL_UP) && (flags & GPIO_PULL_DOWN)) {
		return -ENOTSUP;
	}
	if (flags == GPIO_DISCONNECTED) {
		/* High impedance */
		*pincfg = N32_CNFMODE_ANALOG;
		return 0;
	}
	if (flags & GPIO_OUTPUT) {
		*pincfg = N32_PCFG_GP_PP | N32_PMODE_50MHZ;
		if (flags & GPIO_SINGLE_ENDED) {
			if (!(flags & GPIO_LINE_OPEN_DRAIN)) {
				/* Open-source not supported by the hardware */
				return -ENOTSUP;
			}
			*pincfg = N32_PCFG_GP_OD | N32_PMODE_50MHZ;
		}
		if (flags & GPIO_OUTPUT_INIT_HIGH) {
			*pincfg |= N32_POD_1;
		}
	} else if (flags & GPIO_INPUT) {
		if (flags & GPIO_PULL_UP) {
			*pincfg = N32_CNFMODE_INPUT_PUPD | N32_POD_1;
		} else if (flags & GPIO_PULL_DOWN) {
			*pincfg = N32_CNFMODE_INPUT_PUPD;
		} else {
			*pincfg = N32_CNFMODE_INPUT_FLOAT;
		}
	} else {
		/* Direction required */
		return -ENOTSUP;
	}

	if (flags & ~(GPIO_DIR_MASK | GPIO_PULL_UP | GPIO_PULL_DOWN |
		      GPIO_SINGLE_ENDED | GPIO_LINE_OPEN_DRAIN |
		      GPIO_OUTPUT_INIT_LOW | GPIO_OUTPUT_INIT_HIGH |
		      GPIO_ACTIVE_LOW)) {
		return -ENOTSUP;
	}

	return 0;
}

static int gpio_n32_pin_configure(const struct device *dev, gpio_pin_t pin,
				  gpio_flags_t flags)
{
	const struct gpio_n32_config *cfg = DEV_CFG(dev);
	pinctrl_soc_pin_t pincfg;
	int ret;

	ret = gpio_n32_flags_to_conf(flags, &pincfg);
	if (ret < 0) {
		return ret;
	}

	ret = clock_control_on(DEVICE_DT_GET(DT_NODELABEL(rcc)),
			       (clock_control_subsys_t *)&cfg->clkid);
	if (ret < 0) {
		return ret;
	}

	return n32_gpioport_configure_pin(cfg->base, pin, pincfg, true);
}

static int gpio_n32_port_get_raw(const struct device *dev,
				 gpio_port_value_t *value)
{
	const struct gpio_n32_config *cfg = DEV_CFG(dev);

	*value = cfg->base->PID;

	return 0;
}

static int gpio_n32_port_set_masked_raw(const struct device *dev,
					gpio_port_pins_t mask,
					gpio_port_value_t value)
{
	const struct gpio_n32_config *cfg = DEV_CFG(dev);

	cfg->base->POD = (cfg->base->POD & ~mask) | (value & mask);

	return 0;
}

static int gpio_n32_port_set_bits_raw(const struct device *dev,
				      gpio_port_pins_t mask)
{
	const struct gpio_n32_config *cfg = DEV_CFG(dev);

	cfg->base->PBSC = mask;

	return 0;
}

static int gpio_n32_port_clear_bits_raw(const struct device *dev,
					gpio_port_pins_t mask)
{
	const struct gpio_n32_config *cfg = DEV_CFG(dev);

	cfg->base->PBC = mask;

	return 0;
}

static int gpio_n32_port_toggle_bits(const struct device *dev,
				     gpio_port_pins_t mask)
{
	const struct gpio_n32_config *cfg = DEV_CFG(dev);

	cfg->base->POD = cfg->base->POD ^ mask;

	return 0;
}

static void gpio_n32_isr(gpio_port_pins_t pin, void *arg)
{
	const struct device *dev = arg;
	struct gpio_n32_data *data = DEV_DATA(dev);

	gpio_fire_callbacks(&data->callbacks, dev, pin);
}

static int gpio_n32_pin_interrupt_configure(const struct device *dev,
					    gpio_pin_t pin,
					    enum gpio_int_mode mode,
					    enum gpio_int_trig trig)
{
	const struct gpio_n32_config *cfg = DEV_CFG(dev);
	uint8_t irq_trig;
	int ret;

	if (mode == GPIO_INT_MODE_DISABLED) {
		n32_gpio_intc_disable_line(pin);
		n32_gpio_intc_remove_irq_callback(pin);
		return 0;
	}

	if (mode != GPIO_INT_MODE_EDGE) {
		/* Level-triggered interrupts are not supported by the EXTI */
		return -ENOTSUP;
	}

	switch (trig) {
	case GPIO_INT_TRIG_LOW:
		irq_trig = N32_GPIO_IRQ_TRIG_FALLING;
		break;
	case GPIO_INT_TRIG_HIGH:
		irq_trig = N32_GPIO_IRQ_TRIG_RISING;
		break;
	case GPIO_INT_TRIG_BOTH:
		irq_trig = N32_GPIO_IRQ_TRIG_BOTH;
		break;
	default:
		return -EINVAL;
	}

	ret = n32_gpio_intc_set_line_src_port(pin, cfg->port);
	if (ret < 0) {
		return ret;
	}

	ret = n32_gpio_intc_set_irq_callback(pin, gpio_n32_isr, (void *)dev);
	if (ret < 0) {
		return ret;
	}

	n32_gpio_intc_select_line_trigger(pin, irq_trig);
	n32_gpio_intc_enable_line(pin);

	return 0;
}

static int gpio_n32_manage_callback(const struct device *dev,
				    struct gpio_callback *callback,
				    bool set)
{
	struct gpio_n32_data *data = DEV_DATA(dev);

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
	.manage_callback = gpio_n32_manage_callback,
};

#define GPIO_N32_INIT(n)						\
	static const struct gpio_n32_config gpio_n32_config_##n = {	\
		.base = (GPIO_Module *)DT_INST_REG_ADDR(n),		\
		.clkid = DT_INST_CLOCKS_CELL(n, bits),			\
		/* N32_CLOCK_GPIOA..G are consecutive (APB2PCLKEN bit 2..8) */ \
		.port = DT_INST_CLOCKS_CELL(n, bits) - N32_CLOCK_GPIOA,	\
		.common = {						\
			.port_pin_mask =				\
				GPIO_PORT_PIN_MASK_FROM_DT_NODE(DT_DRV_INST(n)), \
		},							\
	};								\
	static struct gpio_n32_data gpio_n32_data_##n;			\
									\
	DEVICE_DT_INST_DEFINE(n, gpio_n32_init, NULL,			\
			      &gpio_n32_data_##n, &gpio_n32_config_##n,	\
			      PRE_KERNEL_1, CONFIG_GPIO_INIT_PRIORITY,	\
			      &gpio_n32_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_N32_INIT)
