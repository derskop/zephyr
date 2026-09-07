/*
 * L3 functional RTC tests for the Nations N32G45X port (n32g45xml_stb).
 *
 * Exercises the full stm32-parity feature surface of the nsing,n32-rtc
 * driver on the real board:
 *  1. set/get_time round trip (the calendar is re-seeded from the same
 *     fixed value at suite start, so test logs are deterministic)
 *  2. supported alarm fields: SECOND|MINUTE|HOUR|MONTHDAY|WEEKDAY
 *     (the N32 comparator has no month/year compare fields)
 *  3. alarm A at now + 7 s and alarm B at now + 15 s: callback, pending
 *     flag, and get_time read-back with the expected mask
 *  4. disabling alarm B with rtc_alarm_set_time(id, 0, NULL): no stale
 *     pending flag and no late callback may appear
 *  5. calibration round trip 0 -> +100000 ppb -> -100000 ppb -> 0 (the
 *     read-back is quantized to ~0.5 ppm pulses, so it is compared with
 *     tolerance), ending back at 0 so later tests see an unadjusted clock
 *  6. the calendar keeps ticking at 1 Hz after all of the above
 *
 * Every test arms its own comparator from the current time, disarms it
 * again before returning, and checks a per-test callback hit delta, so
 * the cases are self-contained: ztest runs them independently (and a
 * case that failed half-way must not poison the next one).
 *
 * KNOWN ISSUE (n32-rtc-alarm-cold-boot): on a backup domain that had to
 * be cold-seeded by the driver (first power-on of a board without VBAT,
 * or a backup-domain reset), the alarm compare engine never latches
 * ALAF/ALBF and the alarm tests below time out. On warm boots over a
 * long-lived healthy domain they fire normally. The suite-start probe
 * print (COLD/WARM) classifies the boot without touching the calendar.
 *
 * Copyright (c) 2026 Nations Technologies Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/ztest.h>

#define RTC_DEV_NODE DT_NODELABEL(rtc)

/* Number of alarms the board exposes; the driver validates ids against
 * the same DT property. Boards may declare 1 (alarm B calls then return
 * -EINVAL) - the B cases skip themselves in that case.
 */
#define RTC_ALARMS_COUNT DT_PROP_OR(DT_NODELABEL(rtc), alarms_count, 0)

#define ALARM_TIME_MASK_HMS	(RTC_ALARM_TIME_MASK_SECOND |	\
				 RTC_ALARM_TIME_MASK_MINUTE |	\
				 RTC_ALARM_TIME_MASK_HOUR)

static const struct device *rtc_dev;

/* Shared callback counters. Each alarm test snapshots its delta before
 * arming, so a leftover fire from another test cannot fake a pass.
 */
static volatile int alarm_a_hits;
static volatile int alarm_b_hits;

static int user_a = 0xAA;
static int user_b = 0xBB;

static void print_rtc_time(const char *tag, const struct rtc_time *t)
{
	printk("%s: %04u-%02u-%02u (wday=%d) %02u:%02u:%02u.%09u\n",
	       tag, t->tm_year + 1900, t->tm_mon + 1, t->tm_mday, t->tm_wday,
	       t->tm_hour, t->tm_min, t->tm_sec, t->tm_nsec);
}

static void alarm_a_cb(const struct device *dev, uint16_t id, void *user_data)
{
	alarm_a_hits++;
	printk("alarm A callback: id=%u user_data=%d (hits=%d)\n",
	       id, *(int *)user_data, alarm_a_hits);
}

static void alarm_b_cb(const struct device *dev, uint16_t id, void *user_data)
{
	alarm_b_hits++;
	printk("alarm B callback: id=%u user_data=%d (hits=%d)\n",
	       id, *(int *)user_data, alarm_b_hits);
}

/*
 * Suite setup: classify the boot (COLD/WARM) by reading the calendar
 * before touching it, then seed the same fixed date every run so the
 * alarms below arm against a known clock. The driver only seeds a
 * cleared backup domain with 2000-01-01, so a 2000 timestamp proves the
 * driver's cold-seed path just ran (true cold boot / backup-domain
 * reset), while a real date means the previous domain state survived.
 */
static void *rtc_nsing_l3_setup(void)
{
	struct rtc_time boot_t = { 0 };
	struct rtc_time seed = {
		.tm_sec = 56,
		.tm_min = 34,
		.tm_hour = 12,
		.tm_mday = 7,
		.tm_mon = 8,    /* September */
		.tm_year = 126, /* 2026 */
		.tm_wday = 1,   /* Monday */
	};
	int ret;

	rtc_dev = DEVICE_DT_GET(RTC_DEV_NODE);
	printk("rtc nsing_l3: device %s\n", rtc_dev->name);
	printk("rtc nsing_l3: alarms_count=%d (DT)\n", (int)RTC_ALARMS_COUNT);

	ret = rtc_get_time(rtc_dev, &boot_t);
	if (ret < 0) {
		printk("PROBE: pre-set get_time failed: %d (-ENODATA = cleared"
		       " domain, driver cold-seeded)\n", ret);
	} else {
		print_rtc_time("PROBE boot domain time", &boot_t);
		printk("PROBE verdict: %s (2000 timestamp = cleared domain,"
		       " driver cold-seeded)\n",
		       boot_t.tm_year == 100 ? "COLD" : "WARM");
	}

	ret = rtc_set_time(rtc_dev, &seed);
	zassert_ok(ret, "suite seed rtc_set_time failed: %d", ret);

	return NULL;
}

/* Advance t by offset_s, carrying into minute and hour (offsets used here
 * never cross a day boundary: alarms compare seconds/minutes/hours only).
 */
static void time_add_seconds(struct rtc_time *t, int offset_s)
{
	t->tm_sec += offset_s;
	if (t->tm_sec < 60) {
		return;
	}
	t->tm_sec -= 60;
	t->tm_min++;
	if (t->tm_min >= 60) {
		t->tm_min -= 60;
		t->tm_hour = (t->tm_hour + 1) % 24;
	}
}

/*
 * Poll up to timeout_ms for the alarm's pending flag to latch (100 ms
 * cadence). Prints each calendar second that elapses so a stuck calendar
 * (no 1 Hz tick) can be told apart from a broken alarm compare.
 */
static int wait_for_alarm(uint16_t id, int timeout_ms)
{
	struct rtc_time t = { 0 };
	int waited = 0;
	int last_sec = -1;

	while (waited < timeout_ms) {
		int pending = rtc_alarm_is_pending(rtc_dev, id);

		if (pending < 0) {
			printk("alarm %u is_pending failed: %d\n", id, pending);
			return pending;
		}
		if (pending == 1) {
			return 1;
		}

		if (rtc_get_time(rtc_dev, &t) == 0 && t.tm_sec != last_sec) {
			last_sec = t.tm_sec;
			printk("   cal %02u:%02u:%02u\n", t.tm_hour, t.tm_min,
			       t.tm_sec);
		}

		k_sleep(K_MSEC(100));
		waited += 100;
	}

	return 0;
}

/*
 * Shared alarm body: arm alarm `id` at now + offset_s, wait for it to
 * latch, verify the callback ran exactly once and the get_time read-back
 * matches the programmed value, then disarm so no stale compare is left
 * armed for the next test.
 */
static void run_alarm_case(uint16_t id, int offset_s, volatile int *hits,
			   rtc_alarm_callback cb, int *user_val)
{
	struct rtc_time target;
	struct rtc_time readback = { 0 };
	uint16_t mask;
	int fired;

	zassert_ok(rtc_get_time(rtc_dev, &target),
		   "alarm %u: rtc_get_time failed", id);
	time_add_seconds(&target, offset_s);
	printk("arming alarm %u at %02u:%02u:%02u (+%d s)\n", id,
	       target.tm_hour, target.tm_min, target.tm_sec, offset_s);

	zassert_ok(rtc_alarm_set_callback(rtc_dev, id, cb, user_val),
		   "alarm %u: set_callback failed", id);
	zassert_ok(rtc_alarm_set_time(rtc_dev, id, ALARM_TIME_MASK_HMS,
				      &target),
		   "alarm %u: set_time failed", id);

	fired = wait_for_alarm(id, (offset_s + 10) * 1000);
	if (fired < 0) {
		zassert_ok(rtc_alarm_set_time(rtc_dev, id, 0, NULL),
			   "alarm %u: disarm after is_pending error failed", id);
		zassert_ok(rtc_alarm_set_callback(rtc_dev, id, NULL, NULL),
			   "alarm %u: clear callback failed", id);
		zassert_true(false, "alarm %u is_pending error", id);
	}
	zassert_true(fired == 1, "alarm %u did not fire within %d s", id,
		     offset_s + 10);

	zassert_ok(rtc_alarm_get_time(rtc_dev, id, &mask, &readback),
		   "alarm %u: get_time failed", id);
	printk("alarm %u readback: %02u:%02u:%02u mask=0x%04x"
	       " (expect 0x%04x)\n",
	       id, readback.tm_hour, readback.tm_min, readback.tm_sec,
	       (uint32_t)mask, (uint32_t)ALARM_TIME_MASK_HMS);
	zassert_equal(mask, ALARM_TIME_MASK_HMS,
		      "alarm %u readback mask mismatch", id);
	zassert_equal(readback.tm_hour, target.tm_hour,
		      "alarm %u readback hour mismatch", id);
	zassert_equal(readback.tm_min, target.tm_min,
		      "alarm %u readback minute mismatch", id);
	zassert_equal(readback.tm_sec, target.tm_sec,
		      "alarm %u readback second mismatch", id);

	zassert_equal(*hits, 1, "alarm %u callback ran %d times (expect 1)",
		      id, *hits);

	/* Leave no armed comparator behind for the next test. */
	zassert_ok(rtc_alarm_set_time(rtc_dev, id, 0, NULL),
		   "alarm %u: disarm failed", id);
	zassert_ok(rtc_alarm_set_callback(rtc_dev, id, NULL, NULL),
		   "alarm %u: clear callback failed", id);
}

/**
 * @brief set_time/get_time round trip at the suite's seeded base value
 *
 * Writes 2026-09-07 12:34:56 and reads it back; all six calendar fields
 * must match (weekday is stored in the hardware day-of-week field but is
 * not part of the Zephyr read-back contract).
 */
ZTEST(rtc_nsing_l3, test_set_get_roundtrip)
{
	struct rtc_time t = {
		.tm_sec = 56,
		.tm_min = 34,
		.tm_hour = 12,
		.tm_mday = 7,
		.tm_mon = 8,    /* September */
		.tm_year = 126, /* 2026 */
		.tm_wday = 1,   /* Monday */
	};
	struct rtc_time rd = { 0 };

	zassert_ok(rtc_set_time(rtc_dev, &t), "rtc_set_time failed");
	zassert_ok(rtc_get_time(rtc_dev, &rd), "rtc_get_time failed");
	print_rtc_time("set time was ", &t);
	print_rtc_time("read back    ", &rd);
	zassert_equal(rd.tm_sec, t.tm_sec, "seconds mismatch");
	zassert_equal(rd.tm_min, t.tm_min, "minutes mismatch");
	zassert_equal(rd.tm_hour, t.tm_hour, "hours mismatch");
	zassert_equal(rd.tm_mday, t.tm_mday, "day-of-month mismatch");
	zassert_equal(rd.tm_mon, t.tm_mon, "month mismatch");
	zassert_equal(rd.tm_year, t.tm_year, "year mismatch");
}

/**
 * @brief the N32 alarm comparator exposes only these fields
 *
 * SECOND|MINUTE|HOUR|MONTHDAY|WEEKDAY (0x004f): there is no month or
 * year compare hardware, and the day field is a weekday/day-of-month
 * selector (bit30 semantics differ from stm32).
 */
ZTEST(rtc_nsing_l3, test_supported_fields)
{
	uint16_t mask = 0;

	if (RTC_ALARMS_COUNT < 1) {
		ztest_test_skip();
		return;
	}

	zassert_ok(rtc_alarm_get_supported_fields(rtc_dev, 0, &mask),
		   "get_supported_fields failed");
	printk("supported alarm fields mask: 0x%04x (expect 0x004f)\n", mask);
	zassert_equal(mask, ALARM_TIME_MASK_HMS | RTC_ALARM_TIME_MASK_MONTHDAY |
			    RTC_ALARM_TIME_MASK_WEEKDAY,
		      "supported field mask mismatch");
}

/**
 * @brief alarm A (id 0) fires at now + 7 s and its callback runs
 *
 * Verifies the pending flag latches within the window, the callback runs
 * exactly once with its user_data, and the read-back keeps the armed
 * value and mask.
 */
ZTEST(rtc_nsing_l3, test_alarm_a_fires)
{
	if (RTC_ALARMS_COUNT < 1) {
		ztest_test_skip();
		return;
	}

	alarm_a_hits = 0;
	run_alarm_case(0, 7, &alarm_a_hits, alarm_a_cb, &user_a);
}

/**
 * @brief alarm B (id 1) fires at now + 15 s and its callback runs
 *
 * Same checks as alarm A on the second comparator; skipped on boards
 * that declare a single alarm.
 */
ZTEST(rtc_nsing_l3, test_alarm_b_fires)
{
	if (RTC_ALARMS_COUNT < 2) {
		ztest_test_skip();
		return;
	}

	alarm_b_hits = 0;
	run_alarm_case(1, 15, &alarm_b_hits, alarm_b_cb, &user_b);
}

/**
 * @brief rtc_alarm_set_time(id, 0, NULL) disables alarm B
 *
 * An armed alarm disabled before its target must leave no stale pending
 * flag and must not fire when the target second passes.
 */
ZTEST(rtc_nsing_l3, test_alarm_disable)
{
	struct rtc_time target;
	int hits_before;
	int stale = 0;

	if (RTC_ALARMS_COUNT < 2) {
		ztest_test_skip();
		return;
	}

	zassert_ok(rtc_get_time(rtc_dev, &target), "rtc_get_time failed");
	time_add_seconds(&target, 6);
	printk("arming alarm B at %02u:%02u:%02u, then disabling it\n",
	       target.tm_hour, target.tm_min, target.tm_sec);

	hits_before = alarm_b_hits;
	zassert_ok(rtc_alarm_set_callback(rtc_dev, 1, alarm_b_cb, &user_b),
		   "alarm B: set_callback failed");
	zassert_ok(rtc_alarm_set_time(rtc_dev, 1, ALARM_TIME_MASK_HMS,
				      &target),
		   "alarm B: set_time failed");
	zassert_ok(rtc_alarm_set_time(rtc_dev, 1, 0, NULL),
		   "alarm B: disable failed");

	/* Watch through the target second plus margin for a late latch. */
	for (int i = 0; i < 14; i++) {
		int pending = rtc_alarm_is_pending(rtc_dev, 1);

		zassert_true(pending >= 0, "alarm B is_pending failed: %d",
			     pending);
		if (pending == 1) {
			stale = 1;
			break;
		}
		k_sleep(K_SECONDS(1));
	}

	zassert_equal(stale, 0, "stale pending flag after disable");
	zassert_equal(alarm_b_hits, hits_before,
		      "alarm B callback ran after disable");

	zassert_ok(rtc_alarm_set_callback(rtc_dev, 1, NULL, NULL),
		   "alarm B: clear callback failed");
}

/**
 * @brief calibration round trip 0 -> +100000 -> -100000 -> 0 ppb
 *
 * The hardware quantizes the adjustment to ~0.5 ppm pulses, so the
 * read-back sits ~136 ppb away from the requested value; compare with a
 * generous +-500 ppb window. Ends at 0 so the clock is unadjusted for
 * the remaining tests.
 */
ZTEST(rtc_nsing_l3, test_calibration_roundtrip)
{
	int32_t calib;

	zassert_ok(rtc_set_calibration(rtc_dev, 0),
		   "set_calibration(0) failed");
	zassert_ok(rtc_get_calibration(rtc_dev, &calib),
		   "get_calibration failed");
	printk("calibration: reset -> %d ppb\n", calib);

	zassert_ok(rtc_set_calibration(rtc_dev, 100000),
		   "set_calibration(+100000) failed");
	zassert_ok(rtc_get_calibration(rtc_dev, &calib),
		   "get_calibration failed");
	printk("calibration: +100000 ppb -> %d ppb (quantized)\n", calib);
	zassert_true(calib > 99500 && calib < 100500,
		     "+100000 ppb read back as %d", calib);

	zassert_ok(rtc_set_calibration(rtc_dev, -100000),
		   "set_calibration(-100000) failed");
	zassert_ok(rtc_get_calibration(rtc_dev, &calib),
		   "get_calibration failed");
	printk("calibration: -100000 ppb -> %d ppb (quantized)\n", calib);
	zassert_true(calib < -99500 && calib > -100500,
		     "-100000 ppb read back as %d", calib);

	zassert_ok(rtc_set_calibration(rtc_dev, 0),
		   "set_calibration(0) restore failed");
	zassert_ok(rtc_get_calibration(rtc_dev, &calib),
		   "get_calibration failed");
	printk("calibration: restore 0 -> %d ppb\n", calib);
	zassert_true(calib > -500 && calib < 500,
		     "calibration not restored to 0 (got %d)", calib);
}

/**
 * @brief the calendar still ticks 1 Hz after alarms and calibration
 *
 * Three consecutive 1 s sleeps must each advance the calendar by at
 * least one second (proves the prescaler chain still runs and the
 * calibration was really reset to zero).
 */
ZTEST(rtc_nsing_l3, test_calendar_ticks)
{
	struct rtc_time t1 = { 0 };
	struct rtc_time t2 = { 0 };
	uint32_t total1, total2;

	zassert_ok(rtc_get_time(rtc_dev, &t1), "rtc_get_time failed");
	print_rtc_time("tick", &t1);

	for (int i = 0; i < 3; i++) {
		k_sleep(K_SECONDS(1));
		zassert_ok(rtc_get_time(rtc_dev, &t2), "rtc_get_time failed");
		print_rtc_time("tick", &t2);
		total1 = t1.tm_hour * 3600U + t1.tm_min * 60U + t1.tm_sec;
		total2 = t2.tm_hour * 3600U + t2.tm_min * 60U + t2.tm_sec;
		zassert_true(total2 >= total1 + 1,
			     "calendar did not advance (was %u, now %u)",
			     total1, total2);
		t1 = t2;
	}
}

ZTEST_SUITE(rtc_nsing_l3, NULL, rtc_nsing_l3_setup, NULL, NULL, NULL);
