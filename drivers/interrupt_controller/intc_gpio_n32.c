/*
 * Copyright (c) 2024 Nations Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/n32_clock_control.h>
#include <zephyr/drivers/interrupt_controller/gpio_intc_n32.h>
#include <zephyr/init.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/util_macro.h>

/*
 * The EXTI controller is F1-compatible on N32G45x. Other series with a
 * different EXTI layout provide their own implementation section.
 */
#ifdef CONFIG_SOC_SERIES_N32G45X
#include <n32g45x.h>
#endif /* CONFIG_SOC_SERIES_N32G45X */

#ifdef CONFIG_SOC_SERIES_N32G45X

#define EXTI_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(nsing_n32_exti)

struct n32_intc_gpio_data {
	n32_gpio_irq_cb_t cb[16];
	void *cb_data[16];
};

static struct n32_intc_gpio_data intc_gpio_data;

/* EXTI line -> NVIC IRQ mapping, filled at init from the devicetree node */
static uint8_t n32_exti_irq_table[16];

static void n32_fill_irq_table(uint32_t start, uint32_t len, uint32_t irq)
{
	for (uint32_t i = 0U; i < len; i++) {
		n32_exti_irq_table[start + i] = irq;
	}
}

static void n32_afio_clock_enable(void)
{
	/* AFIO_EXTI_CFG line source selection requires the AFIO clock */
	uint32_t clkid = N32_CLOCK_AFIO;

	(void)clock_control_on(DEVICE_DT_GET(DT_NODELABEL(rcc)),
			       (clock_control_subsys_t)&clkid);
}

int n32_gpio_intc_set_line_src_port(n32_gpio_irq_line_t line, uint8_t port)
{
	/* AFIO_EXTI_CFG1..4, 4 bits per line, port select 0-6 (PA..PG) */
	const uint32_t shift = (line % 4U) * 4U;

	n32_afio_clock_enable();

	AFIO->EXTI_CFG[line / 4U] &= ~(0xFU << shift);
	AFIO->EXTI_CFG[line / 4U] |= (uint32_t)(port & 0x7U) << shift;

	return 0;
}

void n32_gpio_intc_enable_line(n32_gpio_irq_line_t line)
{
	EXTI->IMASK |= BIT(line);

	/* IRQ_CONNECT only registers the vector; the NVIC IRQ must be
	 * enabled explicitly.
	 */
	irq_enable(n32_exti_irq_table[line]);
}

void n32_gpio_intc_disable_line(n32_gpio_irq_line_t line)
{
	EXTI->IMASK &= ~BIT(line);
}

void n32_gpio_intc_select_line_trigger(n32_gpio_irq_line_t line, uint8_t trig)
{
	switch (trig) {
	case N32_GPIO_IRQ_TRIG_RISING:
		EXTI->RT_CFG |= BIT(line);
		EXTI->FT_CFG &= ~BIT(line);
		break;
	case N32_GPIO_IRQ_TRIG_FALLING:
		EXTI->RT_CFG &= ~BIT(line);
		EXTI->FT_CFG |= BIT(line);
		break;
	case N32_GPIO_IRQ_TRIG_BOTH:
		EXTI->RT_CFG |= BIT(line);
		EXTI->FT_CFG |= BIT(line);
		break;
	default:
		break;
	}
}

int n32_gpio_intc_set_irq_callback(n32_gpio_irq_line_t line,
				   n32_gpio_irq_cb_t cb, void *user)
{
	if (intc_gpio_data.cb[line] != NULL) {
		return -EBUSY;
	}

	intc_gpio_data.cb[line] = cb;
	intc_gpio_data.cb_data[line] = user;

	return 0;
}

void n32_gpio_intc_remove_irq_callback(n32_gpio_irq_line_t line)
{
	intc_gpio_data.cb[line] = NULL;
	intc_gpio_data.cb_data[line] = NULL;
}

/* One ISR per interrupt vector; dispatch to the per-line callbacks. */
static void n32_intc_gpio_isr(const void *exti_range)
{
	const uint32_t *range = exti_range;
	uint32_t line;

	for (uint32_t i = 0U; i < range[1]; i++) {
		line = range[0] + i;

		if ((EXTI->PEND & BIT(line)) != 0U) {
			/* Clear the pending bit (write 1 to clear) */
			EXTI->PEND = BIT(line);

			if (intc_gpio_data.cb[line] != NULL) {
				intc_gpio_data.cb[line](BIT(line),
							intc_gpio_data.cb_data[line]);
			}
		}
	}
}

#define N32_EXTI_INIT_LINE_RANGE(node_id, interrupts, idx)			\
	static const uint32_t line_range_##idx[2] = {				\
		DT_PROP_BY_IDX(node_id, line_ranges, UTIL_X2(idx)),		\
		DT_PROP_BY_IDX(node_id, line_ranges, UTIL_INC(UTIL_X2(idx)))	\
	};									\
	n32_fill_irq_table(line_range_##idx[0], line_range_##idx[1],		\
			   DT_IRQ_BY_IDX(node_id, idx, irq));			\
	IRQ_CONNECT(DT_IRQ_BY_IDX(node_id, idx, irq),				\
		    DT_IRQ_BY_IDX(node_id, idx, priority),			\
		    n32_intc_gpio_isr, line_range_##idx, 0);

static int n32_exti_gpio_intc_init(void)
{
	DT_FOREACH_PROP_ELEM(EXTI_NODE, interrupt_names, N32_EXTI_INIT_LINE_RANGE);

	return 0;
}

SYS_INIT(n32_exti_gpio_intc_init, PRE_KERNEL_1, CONFIG_INTC_INIT_PRIORITY);

#endif /* CONFIG_SOC_SERIES_N32G45X */
