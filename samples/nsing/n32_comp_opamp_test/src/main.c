/*
 * Copyright (c) 2026 Nations Technologies
 * SPDX-License-Identifier: Apache-2.0
 *
 * N32G45x COMP / OPAMP driver hardware test.
 *
 * The test enables (through the board overlay):
 *   - comp1 : +PA1 vs VREF1          (Zephyr comparator device, IRQ 82)
 *   - opamp1: non-inverting PGA, PA1 (Zephyr opamp device)
 *   - opamp4: follower, PC3          (Zephyr opamp device)
 *
 * The application drives the Zephyr APIs, then reads the peripheral
 * registers back and records PASS/FAIL in the "autotest_results" RAM
 * structure.  A J-Link script then halts the core and reads both the
 * result structure and the COMP/OPAMP registers to check the hardware
 * actually received the expected configuration.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/comparator.h>
#include <zephyr/drivers/opamp.h>
#include <zephyr/sys/printk.h>

#include <soc.h>
#include <n32g45x_comp.h>
#include <n32g45x_opamp.h>

/* Peripheral bases (see dts/arm/nsing/n32g45x/n32g45x.dtsi / HAL n32g45x.h) */
#define COMP_BASE_ADDR	 0x40002400U
#define OPAMP_BASE_ADDR	 0x40002000U

#define TEST_MAGIC	 0x4E333254U /* "N32T" */

struct autotest_results {
	uint32_t magic;
	uint32_t comp1_cfg_ok;	 /* driver register configuration matches DT */
	uint32_t comp1_out;	 /* last comparator_get_output() value         */
	uint32_t comp1_api_ok;	 /* get_output/set_trigger API return codes   */
	uint32_t opamp1_cfg_ok;	 /* EN/MOD/VMSEL after init                    */
	uint32_t opamp1_gain_ok; /* set_gain(OPAMP_GAIN_4) applied to PGAGAN */
	uint32_t opamp4_cfg_ok;	 /* follower EN/MOD/VMSEL                      */
	uint32_t all_ok;	 /* AND of all *_ok                            */
} __attribute__((used));

static volatile struct autotest_results autotest_results;

static int test_comp1(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(comp1));
	volatile COMP_Module *comp = (COMP_Module *)COMP_BASE_ADDR;
	uint32_t ctrl, vrefscl;
	int out, ret;

	autotest_results.comp1_cfg_ok = 0U;
	autotest_results.comp1_api_ok = 0U;

	if (!device_is_ready(dev)) {
		printk("comp1: device not ready\n");
		return -ENODEV;
	}

	/* --- register configuration check ----------------------------- */
	ctrl = comp->Cmp[0].CTRL; /* COMP1_CTRL @ 0x40002410 */
	vrefscl = comp->VREFSCL;

	printk("comp1: COMP1_CTRL=0x%08x VREFSCL=0x%08x\n", ctrl, vrefscl);

	if ((ctrl & COMP_CTRL_EN_MASK) == 0U) {
		printk("comp1: FAIL EN not set\n");
		return -EIO;
	}
	if (((ctrl & COMP_CTRL_INPSEL_MASK) >> 4) != 0U) {
		printk("comp1: FAIL INPSEL != PA1\n");
		return -EIO;
	}
	if (((ctrl & COMP_CTRL_INMSEL_MASK) >> 1) != 3U) {
		printk("comp1: FAIL INMSEL != VREF1\n");
		return -EIO;
	}
	if ((vrefscl & COMP_VREFSCL_VV1EN_MSK) == 0U) {
		printk("comp1: FAIL VREF1 scaler not enabled\n");
		return -EIO;
	}
	autotest_results.comp1_cfg_ok = 1U;

	/* --- API smoke test -------------------------------------------- */
	out = comparator_get_output(dev);
	if (out < 0) {
		printk("comp1: get_output failed (%d)\n", out);
		return out;
	}
	autotest_results.comp1_out = (uint32_t)out;

	ret = comparator_set_trigger(dev, COMPARATOR_TRIGGER_NONE);
	if (ret != 0) {
		printk("comp1: set_trigger(NONE) failed (%d)\n", ret);
		return ret;
	}
	ret = comparator_set_trigger(dev, COMPARATOR_TRIGGER_RISING_EDGE);
	if (ret != 0) {
		printk("comp1: set_trigger(RISING) failed (%d)\n", ret);
		return ret;
	}
	(void)comparator_trigger_is_pending(dev);
	ret = comparator_set_trigger(dev, COMPARATOR_TRIGGER_NONE);
	if (ret != 0) {
		printk("comp1: set_trigger(NONE) failed (%d)\n", ret);
		return ret;
	}
	autotest_results.comp1_api_ok = 1U;
	printk("comp1: cfg ok, out=%d, api ok\n", out);

	return 0;
}

static int test_opamp1(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(opamp1));
	volatile OPAMP_Module *opamp = (OPAMP_Module *)OPAMP_BASE_ADDR;
	uint32_t cs1;

	autotest_results.opamp1_cfg_ok = 0U;
	autotest_results.opamp1_gain_ok = 0U;

	if (!device_is_ready(dev)) {
		printk("opamp1: device not ready\n");
		return -ENODEV;
	}

	cs1 = opamp->CS1;
	printk("opamp1: CS1=0x%08x\n", cs1);

	/* configuration written by n32_opamp_init (non_inverting/PGA) */
	if ((cs1 & OPAMP_CS_EN_MASK) == 0U) {
		printk("opamp1: FAIL EN not set\n");
		return -EIO;
	}
	if ((cs1 & OPAMP_CS_MOD_MASK) != OPAMP_CS_PGA_EN) {
		printk("opamp1: FAIL MOD != PGA\n");
		return -EIO;
	}
	if ((cs1 & OPAMP_CS_VMSEL_MASK) != OPAMPx_CS_VMSEL_FLOAT) {
		printk("opamp1: FAIL VMSEL not floating\n");
		return -EIO;
	}
	autotest_results.opamp1_cfg_ok = 1U;

	/* runtime gain change via the Zephyr API must reach PGAGAN */
	if (opamp_set_gain(dev, OPAMP_GAIN_4) != 0) {
		printk("opamp1: set_gain(x4) failed\n");
		return -EIO;
	}
	cs1 = opamp->CS1;
	if ((cs1 & OPAMP_CS_PGA_GAIN_MASK) != OPAMP_CS_PGA_GAIN_4) {
		printk("opamp1: FAIL PGAGAN != x4 (CS1=0x%08x)\n", cs1);
		return -EIO;
	}
	autotest_results.opamp1_gain_ok = 1U;
	printk("opamp1: cfg ok, gain x4 applied (CS1=0x%08x)\n", cs1);

	return 0;
}

static int test_opamp4(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(opamp4));
	volatile OPAMP_Module *opamp = (OPAMP_Module *)OPAMP_BASE_ADDR;
	uint32_t cs4;

	autotest_results.opamp4_cfg_ok = 0U;

	if (!device_is_ready(dev)) {
		printk("opamp4: device not ready\n");
		return -ENODEV;
	}

	cs4 = opamp->CS4;
	printk("opamp4: CS4=0x%08x\n", cs4);

	/* follower mode set by n32_opamp_init */
	if ((cs4 & OPAMP_CS_EN_MASK) == 0U) {
		printk("opamp4: FAIL EN not set\n");
		return -EIO;
	}
	if ((cs4 & OPAMP_CS_MOD_MASK) != OPAMP_CS_FOLLOW) {
		printk("opamp4: FAIL MOD != FOLLOW (CS4=0x%08x)\n", cs4);
		return -EIO;
	}
	if ((cs4 & OPAMP_CS_VMSEL_MASK) != OPAMPx_CS_VMSEL_FLOAT) {
		printk("opamp4: FAIL VMSEL not floating\n");
		return -EIO;
	}
	autotest_results.opamp4_cfg_ok = 1U;
	printk("opamp4: follower cfg ok (CS4=0x%08x)\n", cs4);

	return 0;
}

void main(void)
{
	int err = 0;

	autotest_results.magic = 0U;
	err |= test_comp1();
	err |= test_opamp1();
	err |= test_opamp4();

	autotest_results.all_ok =
		autotest_results.comp1_cfg_ok &
		autotest_results.comp1_api_ok &
		autotest_results.opamp1_cfg_ok &
		autotest_results.opamp1_gain_ok &
		autotest_results.opamp4_cfg_ok;

	autotest_results.magic = TEST_MAGIC;

	printk("autotest: all_ok=%u\n", autotest_results.all_ok);
	printk("autotest: results @ 0x%p\n", (void *)&autotest_results);
	printk("autotest: %s\n", autotest_results.all_ok ? "PASS" : "FAIL");

	k_sleep(K_FOREVER);
}
