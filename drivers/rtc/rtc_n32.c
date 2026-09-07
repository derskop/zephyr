/*
 * Copyright (c) 2026 Nations Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nsing_n32_rtc

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/dt-bindings/clock/n32_clock.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "rtc_utils.h"

LOG_MODULE_REGISTER(rtc_n32, CONFIG_RTC_LOG_LEVEL);

/*
 * The N32 RTC is a BCD calendar RTC (TSH/DATE/PRE/CTRL/INITSTS registers):
 * the RTC clock source selection lives in RCC->BDCTRL (RTCSEL can only be
 * changed after a backup domain reset), and alarm events reach the NVIC
 * through EXTI line 17 -> RTCAlarm_IRQn.
 */
#ifdef CONFIG_SOC_SERIES_N32G45X

#include <n32g45x.h>

#define RTC_N32_CLK_SRC_LSE        0
#define RTC_N32_CLK_SRC_LSI        1
#define RTC_N32_CLK_SRC_HSE_DIV128 2

#define RTC_N32_CLK_SRC_IDX DT_ENUM_IDX(DT_DRV_INST(0), nsing_rtc_clock_source)

BUILD_ASSERT(DT_NODE_HAS_PROP(DT_DRV_INST(0), nsing_rtc_clock_source),
	     "rtc node enabled without nsing,rtc-clock-source: declare the board "
	     "clock source (lse/lsi/hse-div128) in the board devicetree");

/* The node declares the clocks it needs so the driver holds
 * no hard coded clock id: the APB1 gates for backup-domain access, BKP and
 * PWR.
 */
BUILD_ASSERT(DT_INST_CLOCKS_HAS_IDX(0, 0) && DT_INST_CLOCKS_HAS_IDX(0, 1),
	     "rtc node enabled without clocks entries: declare "
	     "clocks = <&rcc N32_CLOCK_BKP>, <&rcc N32_CLOCK_PWR>");

/* RTCSEL[1:0] in RCC->BDCTRL */
#define RTC_N32_BDCTRL_RTCSEL_MASK 0x00000300U

/* Poll bound for the init-mode and shadow-sync flag waits (INITF/RSYF). */
#define RTC_N32_INITM_TIMEOUT 100000U

/*
 * The prescalers are computed from the frequency of the declared clock
 * source, never hard coded: the frequency is read from the same DT
 * fixed-clock node the clock controller uses, so a board overriding
 * clk_lse/clk_lsi/clk_hse gets a consistent RTC divider:
 *
 *  - lse:        32.768 kHz crystal                   -> clk_lse  clock-frequency
 *  - lsi:        internal RC, 40 kHz typical per DS_N32G457 (the RC drifts
 *                with device and temperature, so an LSI calendar can still
 *                be off by tens of seconds per day; accuracy demanding
 *                applications must use LSE)           -> clk_lsi  clock-frequency
 *  - hse-div128: RTCCLK = HSE / 128 (the /128 divider is fixed by the RCC
 *                block)                               -> clk_hse  clock-frequency / 128
 *
 * rtc_n32_derive_prescalers() turns that frequency into the (PREDIV_A,
 * PREDIV_S) pair behind the 1 Hz calendar.
 */
#define RTC_N32_CLK_FREQ_HZ                                                                        \
	((RTC_N32_CLK_SRC_IDX == RTC_N32_CLK_SRC_LSE)                                              \
		 ? (uint32_t)DT_PROP(DT_NODELABEL(clk_lse), clock_frequency)                       \
	 : (RTC_N32_CLK_SRC_IDX == RTC_N32_CLK_SRC_LSI)                                            \
		 ? (uint32_t)DT_PROP(DT_NODELABEL(clk_lsi), clock_frequency)                       \
		 : (uint32_t)(DT_PROP(DT_NODELABEL(clk_hse), clock_frequency) / 128))

/* The hse-div128 RTC clock only exists when the HSE declaration is a multiple of 128. */
BUILD_ASSERT((RTC_N32_CLK_SRC_IDX != RTC_N32_CLK_SRC_HSE_DIV128) ||
		     ((DT_PROP(DT_NODELABEL(clk_hse), clock_frequency) % 128) == 0),
	     "clk_hse clock-frequency must be a multiple of 128 for hse-div128");

/* Prescaler register widths (silicon limits). */
#define RTC_N32_PREDIV_A_MAX 0x7FU   /* PREDIV_A + 1 in [1, 128] */
#define RTC_N32_PREDIV_S_MAX 0x7FFFU /* apre = PREDIV_S + 1 <= 32768 */

/* N32 stores the year 2000 + Y in BCD year field [0, 99]; tm_year is year - 1900. */
#define RTC_N32_TM_YEAR_MIN 100
#define RTC_N32_TM_YEAR_MAX 199

/* tm_wday: 0 = Sunday .. 6 = Saturday. N32: 1 = Monday .. 7 = Sunday. */
#define RTC_N32_WDAY_TO_TM(wday)   ((wday) % 7)
#define RTC_N32_WDAY_FROM_TM(wday) (((wday) + 6) % 7 + 1)

#ifdef CONFIG_RTC_ALARM

/* Number of alarms is a DT property: 1 exposes alarm A only, 2 adds alarm B. */
#define RTC_N32_ALARMS_COUNT DT_INST_PROP(0, alarms_count)

#define RTC_N32_ALRM_A 0U
#define RTC_N32_ALRM_B 1U

/* Zephyr alarm time fields supported by the hardware. */
#define RTC_N32_SUPPORTED_ALARM_FIELDS                                                             \
	(RTC_ALARM_TIME_MASK_SECOND | RTC_ALARM_TIME_MASK_MINUTE | RTC_ALARM_TIME_MASK_HOUR |      \
	 RTC_ALARM_TIME_MASK_WEEKDAY | RTC_ALARM_TIME_MASK_MONTHDAY)

/* The RTC alarm event is routed to the NVIC through the EXTI line declared
 * as alrm-exti-line (17 on N32G45x): the line is a DT property, not a hard
 * coded literal.
 */
#define RTC_N32_EXTI_ALARM_LINE DT_INST_PROP(0, alrm_exti_line)
/* EXTI->IMASK / RT_CFG / PEND are plain per-line bit maps. */
#define RTC_N32_EXTI_ALARM_BIT  BIT(RTC_N32_EXTI_ALARM_LINE)

#endif /* CONFIG_RTC_ALARM */

#ifdef CONFIG_RTC_CALIBRATION
/*
 * Smooth calibration: the CALIB register (bit 15 CP, bits 8:0 CM[8:0]) adds
 * or removes (512 * CP) - CM[8:0] RTCCLK pulses per calibration window, the
 * default window being 2^20 RTCCLK cycles (~32 s at 32768 Hz). Values are
 * exchanged with the user in ppb, through a fixed frequency-independent
 * scaling:
 *
 * nb_pulses = ppb * 2^20 / 10^9 = ppb * 2^11 / 5^9 = ppb * 2048 / 1953125
 */
#define RTC_N32_CALIB_PPB_TO_NB_PULSES(ppb)    DIV_ROUND_CLOSEST((ppb) * 2048, 1953125)
#define RTC_N32_CALIB_NB_PULSES_TO_PPB(pulses) DIV_ROUND_CLOSEST((pulses) * 1953125, 2048)

/* CP is a single bit representing 512 pulses per window; CM is 9 bits (0-511). */
#define RTC_N32_CALIB_MAX_CALP 512
#define RTC_N32_CALIB_MAX_CALM 511

#define RTC_N32_CALIB_MAX_PPB RTC_N32_CALIB_NB_PULSES_TO_PPB(RTC_N32_CALIB_MAX_CALP)
#define RTC_N32_CALIB_MIN_PPB (-RTC_N32_CALIB_NB_PULSES_TO_PPB(RTC_N32_CALIB_MAX_CALM))
#endif /* CONFIG_RTC_CALIBRATION */

#ifdef CONFIG_RTC_ALARM
struct rtc_n32_alarm {
	rtc_alarm_callback user_callback;
	void *user_data;
	bool is_pending;
};
#endif /* CONFIG_RTC_ALARM */

struct rtc_n32_data {
	struct k_spinlock lock;
	/* DIVS value actually programmed, read back from RTC->PRE. */
	uint16_t divs;
#ifdef CONFIG_RTC_ALARM
	/* Index 0 is alarm A, index 1 alarm B (only when alarms-count > 1). */
	struct rtc_n32_alarm alarm[2];
#endif /* CONFIG_RTC_ALARM */
};

#define RTC_N32_DATA(dev) ((struct rtc_n32_data *)(dev)->data)

/* True when the oscillator behind a RDY flag is already up: the RTC clock
 * sources keep running across resets, so a previous boot may have left the
 * source ready before this driver runs.
 */
static bool rtc_n32_osc_ready(uint8_t rdy_flag)
{
	return RCC_GetFlagStatus(rdy_flag) == SET;
}

/*
 * Poll a clock flag instead of busy-waiting on a time base: this driver
 * initializes at PRE_KERNEL_1, but the kernel timer (SysTick) only starts
 * with the system clock driver at PRE_KERNEL_2, so the cycle counter is
 * still frozen while we run and k_busy_wait() would never return (seen as
 * a cold-boot hang in the LSE start-up wait, where the oscillator is not
 * ready yet). Raw cycle polling is required in this window.
 *
 * Each loop iteration is one RCC_GetFlagStatus() call of about
 * RTC_N32_OSC_POLL_COST_CYCLES core cycles; the iteration budget below is
 * derived from timeout_us at the core clock SystemInit() recorded in
 * SystemCoreClock. The flag is re-polled every iteration and the loop
 * exits as soon as it sets, so the budget only bounds the failure path
 * (e.g. a crystal that never starts); the success path lasts exactly as
 * long as the oscillator start-up.
 */
#define RTC_N32_OSC_POLL_COST_CYCLES 32U

static int rtc_n32_wait_flag(uint8_t flag, uint32_t timeout_us)
{
	uint64_t poll_cycles = (uint64_t)timeout_us * SystemCoreClock / USEC_PER_SEC;
	uint32_t polls = poll_cycles / RTC_N32_OSC_POLL_COST_CYCLES + 1U;

	while (polls-- > 0U) {
		if (RCC_GetFlagStatus(flag) == SET) {
			return 0;
		}
	}

	return -EIO;
}

/*
 * Enable the oscillator backing the declared source and make sure it is
 * ready. The RTCSEL field of BDCTRL is only written when no source was
 * selected yet: once latched it cannot be changed without a backup domain
 * reset. If a previous boot (e.g. vendor firmware) already latched a source,
 * we trust it and log a warning when it differs from the board declaration.
 */
static int rtc_n32_clock_init(void)
{
	/*
	 * Enable the APB1 gates the node declares: N32 needs both the BKP
	 * and the PWR clock for backup-domain access, so the node carries
	 * two clocks cells.
	 */
	const struct device *rcc = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_IDX(0, 0));
	uint32_t clkids[2] = {
		DT_INST_CLOCKS_CELL_BY_IDX(0, 0, bits),
		DT_INST_CLOCKS_CELL_BY_IDX(0, 1, bits),
	};
	uint32_t bdctrl_sel;
	int ret;

	for (int i = 0; i < ARRAY_SIZE(clkids); i++) {
		ret = clock_control_on(rcc, (clock_control_subsys_t *)&clkids[i]);
		if (ret < 0) {
			LOG_ERR("Failed to enable clock id 0x%x", clkids[i]);
			return ret;
		}
	}

	/* Disable backup domain write protection (PWR->CTRL.DBKP). */
	PWR->CTRL |= PWR_CTRL_DBKP;

	bdctrl_sel = RCC->BDCTRL & RTC_N32_BDCTRL_RTCSEL_MASK;

	switch (RTC_N32_CLK_SRC_IDX) {
	case RTC_N32_CLK_SRC_LSE:
		/* Start the crystal only when it is not already up: the backup
		 * domain keeps LSE running across resets, so a previous boot may
		 * have left it ready. A cold crystal (first power up or backup
		 * domain reset) takes up to ~2 s to oscillate per the reference
		 * manual, so allow that before giving up.
		 */
		if (!rtc_n32_osc_ready(RCC_FLAG_LSERD)) {
			RCC_ConfigLse(RCC_LSE_ENABLE);
			ret = rtc_n32_wait_flag(RCC_FLAG_LSERD, 3000000);
			if (ret < 0) {
				LOG_ERR("LSE did not start: check that a 32.768 kHz crystal is "
					"present and the nsing,rtc-clock-source declaration "
					"matches "
					"the hardware");
				return ret;
			}
		}
		if (bdctrl_sel == 0) {
			RCC_ConfigRtcClk(RCC_RTCCLK_SRC_LSE);
		}
		break;
	case RTC_N32_CLK_SRC_LSI:
		/* Start the RC oscillator only when it is not already running
		 * (same reasoning as LSE above).
		 */
		if (!rtc_n32_osc_ready(RCC_FLAG_LSIRD)) {
			RCC_EnableLsi(ENABLE);
			ret = rtc_n32_wait_flag(RCC_FLAG_LSIRD, 1000000);
			if (ret < 0) {
				LOG_ERR("LSI did not start");
				return ret;
			}
		}
		if (bdctrl_sel == 0) {
			RCC_ConfigRtcClk(RCC_RTCCLK_SRC_LSI);
		}
		break;
	case RTC_N32_CLK_SRC_HSE_DIV128:
		/* The system clock path usually already runs HSE; just verify it. */
		ret = rtc_n32_wait_flag(RCC_FLAG_HSERD, 1000000);
		if (ret < 0) {
			LOG_ERR("HSE not ready: hse-div128 declared but no HSE running - "
				"check the board clock configuration");
			return ret;
		}
		if (bdctrl_sel == 0) {
			RCC_ConfigRtcClk(RCC_RTCCLK_SRC_HSE_DIV128);
		}
		break;
	default:
		return -EINVAL;
	}

	if (bdctrl_sel != 0) {
		uint32_t declared = 0;

		switch (RTC_N32_CLK_SRC_IDX) {
		case RTC_N32_CLK_SRC_LSE:
			declared = RCC_RTCCLK_SRC_LSE;
			break;
		case RTC_N32_CLK_SRC_LSI:
			declared = RCC_RTCCLK_SRC_LSI;
			break;
		case RTC_N32_CLK_SRC_HSE_DIV128:
			declared = RCC_RTCCLK_SRC_HSE_DIV128;
			break;
		}

		if (bdctrl_sel != declared) {
			LOG_WRN("BDCTRL.RTCSEL retains 0x%x, board declares 0x%x: source "
				"cannot be changed without a backup domain reset",
				bdctrl_sel, declared);
		}
	}

	RCC_EnableRtcClk(ENABLE);

	return 0;
}

/*
 * Turn the RTC clock frequency into the prescaler pair that produces
 * exactly 1 Hz at the calendar input. Both prescalers are integer dividers
 * (f_ck_apre = freq / (PREDIV_A + 1), f_ck_spre = f_ck_apre / (PREDIV_S + 1)),
 * so ck_spre = freq / ((PREDIV_A + 1) * (PREDIV_S + 1)) is exactly 1 Hz only
 * when freq has a divisor d = PREDIV_A + 1 in [1, 128] with apre = freq / d
 * <= 32768 (the PREDIV_S range); the largest such d is preferred, which
 * keeps the classic 256 Hz asynchronous rate at 32.768 kHz.
 *
 * Every RTC behavior (calendar seconds, sub-second counter, alarm
 * comparisons) is anchored on that 1 Hz clock, so a divider pair that can
 * only approximate it is refused: silently seeding a calendar that runs off
 * rate would be worse than failing init. Boards must declare a clock
 * frequency that the divider chain can reach exactly (all documented N32
 * sources do: 32768, 40000 and HSE/128 divide cleanly).
 *
 * Returns 0 on success, a negative error code when the frequency cannot be
 * divided into exactly 1 Hz.
 */
static int rtc_n32_derive_prescalers(uint32_t freq, uint16_t *diva, uint16_t *divs)
{
	uint32_t apre;
	uint32_t d;

	if ((freq < 2U) || (freq > (uint32_t)(RTC_N32_PREDIV_A_MAX + 1U) *
					   (uint32_t)(RTC_N32_PREDIV_S_MAX + 1U))) {
		return -EINVAL;
	}

	for (d = RTC_N32_PREDIV_A_MAX + 1U; d >= 1U; d--) {
		if ((freq % d) == 0U) {
			apre = freq / d;

			if (apre <= (uint32_t)(RTC_N32_PREDIV_S_MAX + 1U)) {
				*diva = d - 1U;
				*divs = apre - 1U;

				return 0;
			}
		}
	}

	/* No exact divider pair exists (freq has no usable divisor). */
	return -EINVAL;
}

/*
 * Seed the calendar on first initialization (INITSF = 0, e.g. first power up
 * or after a backup domain reset): prescalers + 2000-01-01 00:00:00 Monday.
 * Once INITSF is set the calendar is never touched again by init - it
 * persists across system resets in the backup domain.
 */
static int rtc_n32_seed_calendar(struct rtc_n32_data *data)
{
	uint16_t diva;
	uint16_t divs;
	int ret;

	ret = rtc_n32_derive_prescalers(RTC_N32_CLK_FREQ_HZ, &diva, &divs);
	if (ret < 0) {
		LOG_ERR("cannot divide %u Hz to exactly 1 Hz: check the clock "
			"frequency declared for the selected source",
			(uint32_t)RTC_N32_CLK_FREQ_HZ);
		return ret;
	}

	/*
	 * A cold seed must program the prescaler, time and date inside a
	 * single init-mode session. Splitting it across the vendor RTC_Init,
	 * RTC_SetDate and RTC_ConfigTime calls (three init-mode entries, each
	 * of which shuts the calendar down and restarts it) leaves the alarm
	 * compare engine dead on this part: ALAF/ALBF never latch on a
	 * freshly seeded domain even though the register file ends up byte
	 * identical to a live one. Writing PRE, TSH and DATE together before
	 * exiting init mode (as the user manual's init sequence describes)
	 * keeps the comparators alive across a cold-seeded boot.
	 */
	RTC->WRP = 0xCA;
	RTC->WRP = 0x53;

	RTC->INITSTS |= RTC_INITSTS_INITM;
	for (uint32_t timeout = 0;
	     (RTC->INITSTS & RTC_INITSTS_INITF) == 0U && timeout < RTC_N32_INITM_TIMEOUT;
	     timeout++) {
	}
	if ((RTC->INITSTS & RTC_INITSTS_INITF) == 0U) {
		RTC->WRP = 0xFF;
		LOG_ERR("RTC failed to enter init mode (INITF timeout)");
		return -EIO;
	}

	RTC->PRE = ((uint32_t)diva << 16) | divs;
	RTC->TSH = 0x00000000UL;  /* 00:00:00, 24 h format */
	RTC->DATE = 0x00002101UL; /* 2000-01-01, Monday */

	RTC->INITSTS &= ~RTC_INITSTS_INITM;
	RTC->WRP = 0xFF;

	/* Calendar restarted at the seeded value: wait for the shadow
	 * registers to resynchronize before they are read.
	 */
	if (RTC_WaitForSynchro() == ERROR) {
		LOG_ERR("RTC shadow sync timeout after seeding");
		return -EIO;
	}

	LOG_INF("RTC calendar seeded at 1 Hz: PREDIV_A = %u, PREDIV_S = %u", diva, divs);

	/* Remember the value actually programmed for the sub-second math. */
	data->divs = RTC->PRE & RTC_N32_PREDIV_S_MAX;

	return 0;
}

#ifdef CONFIG_RTC_ALARM

/* Vendor handles for the two hardware alarms (CTRL enable bits). */
static inline uint32_t rtc_n32_alarm_hw_id(uint16_t id)
{
	return (id == RTC_N32_ALRM_B) ? RTC_B_ALARM : RTC_A_ALARM;
}

/* INITSTS flags and CTRL interrupt enable bits of an alarm. */
static inline uint32_t rtc_n32_alarm_flag(uint16_t id)
{
	return (id == RTC_N32_ALRM_B) ? RTC_FLAG_ALBF : RTC_FLAG_ALAF;
}

static inline uint32_t rtc_n32_alarm_it(uint16_t id)
{
	return (id == RTC_N32_ALRM_B) ? RTC_INT_ALRB : RTC_INT_ALRA;
}

static inline bool rtc_n32_alarm_id_valid(uint16_t id)
{
	return (id == RTC_N32_ALRM_A) ||
	       (RTC_N32_ALARMS_COUNT > RTC_N32_ALRM_B && id == RTC_N32_ALRM_B);
}

/*
 * Program one alarm register from Zephyr time/mask semantics: the mask lists
 * the fields that participate in the comparison, while the hardware MASKx
 * bits mean "ignore" - exactly the opposite, hence the inversion. WEEKDAY and
 * MONTHDAY share a single hardware comparison field (WKDSEL chooses which one
 * is matched), so at most one of them may be active.
 */
static void rtc_n32_alarm_config_hw(const struct rtc_time *timeptr, uint16_t id, uint16_t mask)
{
	RTC_AlarmType alarm = {0};

	alarm.AlarmMask = RTC_ALARMMASK_ALL;

	if (mask & RTC_ALARM_TIME_MASK_SECOND) {
		alarm.AlarmMask &= ~RTC_ALARMMASK_SECONDS;
		alarm.AlarmTime.Seconds = timeptr->tm_sec;
	}

	if (mask & RTC_ALARM_TIME_MASK_MINUTE) {
		alarm.AlarmMask &= ~RTC_ALARMMASK_MINUTES;
		alarm.AlarmTime.Minutes = timeptr->tm_min;
	}

	if (mask & RTC_ALARM_TIME_MASK_HOUR) {
		alarm.AlarmMask &= ~RTC_ALARMMASK_HOURS;
		alarm.AlarmTime.Hours = timeptr->tm_hour;
	}

	if (mask & RTC_ALARM_TIME_MASK_WEEKDAY) {
		alarm.AlarmMask &= ~RTC_ALARMMASK_WEEKDAY;
		alarm.DateWeekMode = RTC_ALARM_SEL_WEEKDAY_WEEKDAY;
		alarm.DateWeekValue = RTC_N32_WDAY_FROM_TM(timeptr->tm_wday);
	} else if (mask & RTC_ALARM_TIME_MASK_MONTHDAY) {
		alarm.AlarmMask &= ~RTC_ALARMMASK_WEEKDAY;
		alarm.DateWeekMode = RTC_ALARM_SEL_WEEKDAY_DATE;
		alarm.DateWeekValue = timeptr->tm_mday;
	}

	RTC_SetAlarm(RTC_FORMAT_BIN, rtc_n32_alarm_hw_id(id), &alarm);
}

static void rtc_n32_isr(const struct device *dev)
{
	struct rtc_n32_data *data = RTC_N32_DATA(dev);
	uint16_t id;

	for (id = 0; id < RTC_N32_ALARMS_COUNT; id++) {
		if (RTC_GetFlagStatus(rtc_n32_alarm_flag(id)) == SET) {
			RTC_ClrFlag(rtc_n32_alarm_flag(id));

			data->alarm[id].is_pending = true;

			if (data->alarm[id].user_callback != NULL) {
				data->alarm[id].user_callback(dev, id, data->alarm[id].user_data);
			}
		}
	}

	/* Acknowledge the EXTI alarm event (write 1 clears). */
	EXTI->PEND = RTC_N32_EXTI_ALARM_BIT;
}

static void rtc_n32_irq_setup(const struct device *dev)
{
	/*
	 * Route the RTC alarm event (the declared alrm-exti-line) to the NVIC:
	 * unmask the line and select its rising edge trigger, then drop any
	 * pending bit latched before the alarms were initialized.
	 */
	EXTI->IMASK |= RTC_N32_EXTI_ALARM_BIT;
	EXTI->RT_CFG |= RTC_N32_EXTI_ALARM_BIT;
	EXTI->PEND = RTC_N32_EXTI_ALARM_BIT;

	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority), rtc_n32_isr, DEVICE_DT_INST_GET(0),
		    0);
	irq_enable(DT_INST_IRQN(0));
}

static int rtc_n32_alarm_get_supported_fields(const struct device *dev, uint16_t id, uint16_t *mask)
{
	if (mask == NULL) {
		LOG_ERR("NULL mask pointer");
		return -EINVAL;
	}

	if (!rtc_n32_alarm_id_valid(id)) {
		LOG_ERR("invalid alarm ID %d", id);
		return -EINVAL;
	}

	*mask = (uint16_t)RTC_N32_SUPPORTED_ALARM_FIELDS;

	return 0;
}

static int rtc_n32_alarm_set_time(const struct device *dev, uint16_t id, uint16_t mask,
				  const struct rtc_time *timeptr)
{
	struct rtc_n32_data *data = RTC_N32_DATA(dev);
	struct rtc_n32_alarm *alarm;
	k_spinlock_key_t key;
	int err = 0;

	if (!rtc_n32_alarm_id_valid(id)) {
		LOG_ERR("invalid alarm ID %d", id);
		return -EINVAL;
	}

	key = k_spin_lock(&data->lock);

	alarm = &data->alarm[id];

	if ((mask == 0) && (timeptr == NULL)) {
		/* Disable the alarm and drop its callback and pending state. */
		alarm->user_callback = NULL;
		alarm->user_data = NULL;
		alarm->is_pending = false;

		/* Disabling also waits for the alarm write flag (ALxWF). */
		if (RTC_EnableAlarm(rtc_n32_alarm_hw_id(id), DISABLE) == ERROR) {
			LOG_ERR("alarm %d disable timed out", id);
			err = -EIO;
			goto unlock;
		}
		RTC_ConfigInt(rtc_n32_alarm_it(id), DISABLE);

		LOG_DBG("alarm %d disabled", id);
		goto unlock;
	}

	if ((mask & ~RTC_N32_SUPPORTED_ALARM_FIELDS) != 0) {
		LOG_ERR("unsupported alarm %d field mask 0x%04x", id, mask);
		err = -EINVAL;
		goto unlock;
	}

	if ((mask & RTC_ALARM_TIME_MASK_WEEKDAY) && (mask & RTC_ALARM_TIME_MASK_MONTHDAY)) {
		/* The hardware can match weekday OR monthday, not both. */
		LOG_ERR("alarm %d cannot match weekday and monthday simultaneously", id);
		err = -EINVAL;
		goto unlock;
	}

	if (timeptr == NULL) {
		LOG_ERR("timeptr is invalid");
		err = -EINVAL;
		goto unlock;
	}

	if (!rtc_utils_validate_rtc_time(timeptr, mask)) {
		LOG_ERR("one or multiple time values are invalid");
		err = -EINVAL;
		goto unlock;
	}

	LOG_DBG("set alarm %d: second = %d, min = %d, hour = %d, wday = %d, "
		"mday = %d, mask = 0x%04x",
		id, timeptr->tm_sec, timeptr->tm_min, timeptr->tm_hour, timeptr->tm_wday,
		timeptr->tm_mday, mask);

	/*
	 * The register can only be rewritten while the alarm is off (the
	 * disable call waits for the ALxWF write flag), so: disable -> write
	 * -> enable -> clear any stale flag -> arm the interrupt.
	 */
	if (RTC_EnableAlarm(rtc_n32_alarm_hw_id(id), DISABLE) == ERROR) {
		LOG_ERR("alarm %d disable timed out", id);
		err = -EIO;
		goto unlock;
	}
	RTC_ConfigInt(rtc_n32_alarm_it(id), DISABLE);

	rtc_n32_alarm_config_hw(timeptr, id, mask);

	if (RTC_EnableAlarm(rtc_n32_alarm_hw_id(id), ENABLE) == ERROR) {
		LOG_ERR("alarm %d enable failed", id);
		err = -EIO;
		goto unlock;
	}
	RTC_ClrFlag(rtc_n32_alarm_flag(id));
	RTC_ConfigInt(rtc_n32_alarm_it(id), ENABLE);

unlock:
	k_spin_unlock(&data->lock, key);

	return err;
}

static int rtc_n32_alarm_get_time(const struct device *dev, uint16_t id, uint16_t *mask,
				  struct rtc_time *timeptr)
{
	struct rtc_n32_data *data = RTC_N32_DATA(dev);
	RTC_AlarmType alarm;
	k_spinlock_key_t key;
	int err = 0;

	if ((mask == NULL) || (timeptr == NULL)) {
		LOG_ERR("NULL pointer");
		return -EINVAL;
	}

	if (!rtc_n32_alarm_id_valid(id)) {
		LOG_ERR("invalid alarm ID %d", id);
		return -EINVAL;
	}

	/* Fields that do not participate in the match are reported as -1. */
	memset(timeptr, -1, sizeof(*timeptr));

	key = k_spin_lock(&data->lock);

	RTC_GetAlarm(RTC_FORMAT_BIN, rtc_n32_alarm_hw_id(id), &alarm);

	k_spin_unlock(&data->lock, key);

	*mask = 0;

	if ((alarm.AlarmMask & RTC_ALARMMASK_SECONDS) == 0) {
		*mask |= RTC_ALARM_TIME_MASK_SECOND;
		timeptr->tm_sec = alarm.AlarmTime.Seconds;
	}
	if ((alarm.AlarmMask & RTC_ALARMMASK_MINUTES) == 0) {
		*mask |= RTC_ALARM_TIME_MASK_MINUTE;
		timeptr->tm_min = alarm.AlarmTime.Minutes;
	}
	if ((alarm.AlarmMask & RTC_ALARMMASK_HOURS) == 0) {
		*mask |= RTC_ALARM_TIME_MASK_HOUR;
		timeptr->tm_hour = alarm.AlarmTime.Hours;
	}
	if ((alarm.AlarmMask & RTC_ALARMMASK_WEEKDAY) == 0) {
		/* WKDSEL picks which day comparison field was programmed. */
		if (alarm.DateWeekMode == RTC_ALARM_SEL_WEEKDAY_WEEKDAY) {
			*mask |= RTC_ALARM_TIME_MASK_WEEKDAY;
			timeptr->tm_wday = RTC_N32_WDAY_TO_TM(alarm.DateWeekValue);
		} else {
			*mask |= RTC_ALARM_TIME_MASK_MONTHDAY;
			timeptr->tm_mday = alarm.DateWeekValue;
		}
	}

	LOG_DBG("get alarm %d: mday = %d, wday = %d, hour = %d, min = %d, sec = %d, "
		"mask = 0x%04x",
		id, timeptr->tm_mday, timeptr->tm_wday, timeptr->tm_hour, timeptr->tm_min,
		timeptr->tm_sec, *mask);

	return err;
}

static int rtc_n32_alarm_set_callback(const struct device *dev, uint16_t id,
				      rtc_alarm_callback callback, void *user_data)
{
	struct rtc_n32_data *data = RTC_N32_DATA(dev);
	struct rtc_n32_alarm *alarm;
	k_spinlock_key_t key;

	if (!rtc_n32_alarm_id_valid(id)) {
		LOG_ERR("invalid alarm ID %d", id);
		return -EINVAL;
	}

	key = k_spin_lock(&data->lock);

	alarm = &data->alarm[id];
	alarm->user_callback = callback;
	alarm->user_data = user_data;

	k_spin_unlock(&data->lock, key);

	return 0;
}

static int rtc_n32_alarm_is_pending(const struct device *dev, uint16_t id)
{
	struct rtc_n32_data *data = RTC_N32_DATA(dev);
	struct rtc_n32_alarm *alarm;
	unsigned int irq_key;
	int ret;

	if (!rtc_n32_alarm_id_valid(id)) {
		LOG_ERR("invalid alarm ID %d", id);
		return -EINVAL;
	}

	/*
	 * The flag is set from the ISR, which does not take the lock, so a
	 * plain lock would not serialize with it: mask interrupts instead.
	 */
	irq_key = irq_lock();
	alarm = &data->alarm[id];
	ret = alarm->is_pending ? 1 : 0;
	alarm->is_pending = false;
	irq_unlock(irq_key);

	return ret;
}

#endif /* CONFIG_RTC_ALARM */

static int rtc_n32_init(const struct device *dev)
{
	struct rtc_n32_data *data = RTC_N32_DATA(dev);
	int ret;

	ret = rtc_n32_clock_init();
	if (ret < 0) {
		return ret;
	}

	if (RTC_WaitForSynchro() == ERROR) {
		LOG_ERR("RTC register synchronization failed");
		return -EIO;
	}

	if (RTC_GetFlagStatus(RTC_FLAG_INITSF) == RESET) {
		ret = rtc_n32_seed_calendar(data);
		if (ret < 0) {
			return ret;
		}
	} else {
		/* Calendar already running from a previous boot (backup domain
		 * survives resets); sub-second math needs the actual DIVS.
		 */
		data->divs = RTC->PRE & 0x7FFFU;
	}

	LOG_INF("N32 RTC ready (clock source: %s, %u Hz)",
		RTC_N32_CLK_SRC_IDX == RTC_N32_CLK_SRC_LSE   ? "lse"
		: RTC_N32_CLK_SRC_IDX == RTC_N32_CLK_SRC_LSI ? "lsi"
							     : "hse-div128",
		(uint32_t)RTC_N32_CLK_FREQ_HZ);

#ifdef CONFIG_RTC_ALARM
	rtc_n32_irq_setup(dev);

	LOG_INF("N32 RTC alarms ready (count: %d)", RTC_N32_ALARMS_COUNT);
#endif /* CONFIG_RTC_ALARM */

	return 0;
}

static int rtc_n32_set_time(const struct device *dev, const struct rtc_time *timeptr)
{
	RTC_TimeType time;
	RTC_DateType date;

	if (timeptr->tm_wday == -1) {
		/* Hardware calendar always carries a week day. */
		return -EINVAL;
	}

	if (!rtc_utils_validate_rtc_time(
		    timeptr, RTC_ALARM_TIME_MASK_SECOND | RTC_ALARM_TIME_MASK_MINUTE |
				     RTC_ALARM_TIME_MASK_HOUR | RTC_ALARM_TIME_MASK_MONTHDAY |
				     RTC_ALARM_TIME_MASK_MONTH | RTC_ALARM_TIME_MASK_YEAR |
				     RTC_ALARM_TIME_MASK_WEEKDAY)) {
		return -EINVAL;
	}

	/* The BCD year field covers 2000-2099. */
	if (timeptr->tm_year < RTC_N32_TM_YEAR_MIN || timeptr->tm_year > RTC_N32_TM_YEAR_MAX) {
		return -EINVAL;
	}

	time.Hours = timeptr->tm_hour;
	time.Minutes = timeptr->tm_min;
	time.Seconds = timeptr->tm_sec;
	time.H12 = RTC_AM_H12; /* ignored in 24 hour mode */

	date.WeekDay = RTC_N32_WDAY_FROM_TM(timeptr->tm_wday);
	date.Date = timeptr->tm_mday;
	date.Month = timeptr->tm_mon + 1;
	date.Year = timeptr->tm_year - 100;

	if (RTC_ConfigTime(RTC_FORMAT_BIN, &time) == ERROR) {
		return -EIO;
	}
	if (RTC_SetDate(RTC_FORMAT_BIN, &date) == ERROR) {
		return -EIO;
	}

	return 0;
}

static int rtc_n32_get_time(const struct device *dev, struct rtc_time *timeptr)
{
	struct rtc_n32_data *data = RTC_N32_DATA(dev);
	RTC_TimeType t1, t2;
	RTC_DateType d1, d2;
	uint32_t ss2;
	uint64_t nsec;
	uint8_t attempt;

	if (RTC_GetFlagStatus(RTC_FLAG_INITSF) == RESET) {
		/* No calendar initialized in the backup domain. */
		return -ENODATA;
	}

	/*
	 * Read time + date twice and only accept a consistent snapshot. The
	 * shadow registers latch on the TSH read and are released by the DATE
	 * read, so a pair crossing a second boundary is detected by the
	 * comparison and retried. SUBS is sampled right before each TSH read.
	 */
	for (attempt = 0; attempt < 4; attempt++) {
		ss2 = RTC->SUBS;
		RTC_GetTime(RTC_FORMAT_BIN, &t1);
		RTC_GetDate(RTC_FORMAT_BIN, &d1);
		RTC_GetTime(RTC_FORMAT_BIN, &t2);
		RTC_GetDate(RTC_FORMAT_BIN, &d2);

		if (t1.Seconds == t2.Seconds && t1.Minutes == t2.Minutes && t1.Hours == t2.Hours &&
		    d1.Date == d2.Date && d1.Month == d2.Month && d1.Year == d2.Year &&
		    d1.WeekDay == d2.WeekDay) {
			break;
		}
	}

	if (attempt == 4) {
		return -EIO;
	}

	/*
	 * SUBS counts down from DIVS within each second: at the second
	 * boundary it is reloaded to DIVS and TSH increments. The fraction of
	 * the current second already elapsed is (DIVS - SS) / (DIVS + 1).
	 * (Direction/sign pinned by hardware tests, see test_time_boundary.)
	 */
	nsec = ((uint64_t)(data->divs - (ss2 & 0xFFFFU)) * 1000000000U) /
	       (uint64_t)(data->divs + 1);

	timeptr->tm_sec = t2.Seconds;
	timeptr->tm_min = t2.Minutes;
	timeptr->tm_hour = t2.Hours;
	timeptr->tm_mday = d2.Date;
	timeptr->tm_mon = d2.Month - 1;
	timeptr->tm_year = d2.Year + 100;
	timeptr->tm_wday = RTC_N32_WDAY_TO_TM(d2.WeekDay);
	timeptr->tm_yday = -1;
	timeptr->tm_isdst = -1;
	timeptr->tm_nsec = nsec;

	return 0;
}

#ifdef CONFIG_RTC_CALIBRATION
static int rtc_n32_set_calibration(const struct device *dev, int32_t calibration)
{
	ARG_UNUSED(dev);

	/* ppb applied on the clock period with an opposite sign. */
	if ((calibration > RTC_N32_CALIB_MAX_PPB) || (calibration < RTC_N32_CALIB_MIN_PPB)) {
		LOG_ERR("calibration %d out of supported range [%d, %d]", calibration,
			RTC_N32_CALIB_MIN_PPB, RTC_N32_CALIB_MAX_PPB);
		return -EINVAL;
	}

	int32_t nb_pulses = RTC_N32_CALIB_PPB_TO_NB_PULSES(calibration);

	__ASSERT_NO_MSG(nb_pulses <= RTC_N32_CALIB_MAX_CALP);
	__ASSERT_NO_MSG(nb_pulses >= -RTC_N32_CALIB_MAX_CALM);

	uint32_t plus;
	uint32_t minus;

	if (nb_pulses > 0) {
		plus = RTC_SMOOTH_CALIB_PLUS_PULSES_SET;
		minus = RTC_N32_CALIB_MAX_CALP - nb_pulses;
	} else {
		plus = RTC_SMOOTH_CALIB_PLUS_PULSES__RESET;
		minus = -nb_pulses;
	}

	/*
	 * A smooth calibration is applied over a whole window (the default
	 * 2^20 RTCCLK cycles: 32 s at 32.768 kHz) and the RECPF flag stays set
	 * for its entire duration. A write issued while the previous window is
	 * still running is dropped by the hardware, so wait the flag out
	 * first. 100 s bounds any window a
	 * previous boot may have left pending (2^20 cycles at the slowest
	 * declared RTC clock, ~32 s, with margin).
	 */
	for (uint32_t i = 0; i < 100000U && (RTC->INITSTS & RTC_INITSTS_RECPF) != 0U; i++) {
		k_sleep(K_MSEC(1));
	}

	if ((RTC->INITSTS & RTC_INITSTS_RECPF) != 0U) {
		LOG_ERR("smooth calibration window never completed");
		return -EIO;
	}

	/*
	 * RTC_ConfigSmoothCalib handles the write protection itself before
	 * applying the new value. The default 2^20 RTCCLK cycle window is kept
	 * (its pulse-to-ppb mapping is the one used above).
	 */
	if (RTC_ConfigSmoothCalib(SMOOTH_CALIB_32SEC, plus, minus) == ERROR) {
		LOG_ERR("smooth calibration failed (recalibration pending?)");
		return -EIO;
	}

	LOG_DBG("set calibration: %d ppb (%d pulses per window)", calibration, nb_pulses);

	return 0;
}

static int rtc_n32_get_calibration(const struct device *dev, int32_t *calibration)
{
	ARG_UNUSED(dev);

	if (calibration == NULL) {
		LOG_ERR("NULL calibration pointer");
		return -EINVAL;
	}

	uint32_t calib = RTC->CALIB;

	/* Pulses per window = (512 * CP) - CM[8:0]. */
	int32_t nb_pulses = -((int32_t)(calib & RTC_CALIB_CM));

	if ((calib & RTC_CALIB_CP) != 0) {
		nb_pulses += RTC_N32_CALIB_MAX_CALP;
	}

	*calibration = RTC_N32_CALIB_NB_PULSES_TO_PPB(nb_pulses);

	return 0;
}
#endif /* CONFIG_RTC_CALIBRATION */

static DEVICE_API(rtc, rtc_n32_driver_api) = {
	.set_time = rtc_n32_set_time,
	.get_time = rtc_n32_get_time,
#ifdef CONFIG_RTC_ALARM
	.alarm_get_supported_fields = rtc_n32_alarm_get_supported_fields,
	.alarm_set_time = rtc_n32_alarm_set_time,
	.alarm_get_time = rtc_n32_alarm_get_time,
	.alarm_set_callback = rtc_n32_alarm_set_callback,
	.alarm_is_pending = rtc_n32_alarm_is_pending,
#endif /* CONFIG_RTC_ALARM */
#ifdef CONFIG_RTC_CALIBRATION
	.set_calibration = rtc_n32_set_calibration,
	.get_calibration = rtc_n32_get_calibration,
#endif /* CONFIG_RTC_CALIBRATION */
};

static struct rtc_n32_data rtc_n32_data_0;

DEVICE_DT_INST_DEFINE(0, rtc_n32_init, NULL, &rtc_n32_data_0, NULL, PRE_KERNEL_1,
		      CONFIG_RTC_INIT_PRIORITY, &rtc_n32_driver_api);

#endif /* CONFIG_SOC_SERIES_N32G45X */
