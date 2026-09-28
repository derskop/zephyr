/*
 * Copyright (c) 2026 Nations Technologies
 * SPDX-License-Identifier: Apache-2.0
 *
 * AT24 series I2C EEPROM driver test.
 *
 * Wiring on the N32G45XML-STB:
 *   I2C1_SCL -> PB6
 *   I2C1_SDA -> PB7
 *   A2       -> GND  (7-bit base address 0x50)
 *   WP       -> GND  (writes enabled)
 *
 * The chip is organised as 256-byte pages that are selected by the upper
 * word-address bits, i.e. by the slave address: an AT24C04 (512 B) answers
 * at 0x50..0x51, an AT24C08 (1 KiB) at 0x50..0x53.  The Zephyr at24 driver
 * models either as one device with an 8-bit word address and a 16-byte
 * page, so the only thing the devicetree has to get right is `size`.
 *
 * Every offset used below is derived from that size, and test 1 scans the
 * bus and reports which addresses actually answer, so a mismatch between
 * the devicetree and the fitted chip shows up as a named failing check rather
 * than as a bare I2C error.
 *
 * Every check prints PASS/FAIL and the sample ends with a summary.  The
 * exit status of the program is not observable on the target, so the
 * operator reads the summary off the console (115200 8N1 on USART1).
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/eeprom.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>

#include <string.h>

#define EEPROM_NODE	 DT_ALIAS(eeprom_0)
#define EEPROM_SIZE	 DT_PROP(EEPROM_NODE, size)
#define EEPROM_PAGE_SIZE DT_PROP(EEPROM_NODE, pagesize)
#define EEPROM_BASE_ADDR DT_REG_ADDR(EEPROM_NODE)

/* Number of 256-byte pages the chip spreads over the I2C address space. */
#define EEPROM_PAGE_COUNT (EEPROM_SIZE / 256)

#define BOOT_MAGIC  0xA72408C0U
#define BOOT_OFFSET (EEPROM_SIZE - sizeof(struct boot_record))

struct boot_record {
	uint32_t magic;
	uint32_t count;
};

static const struct device *const eeprom = DEVICE_DT_GET(EEPROM_NODE);
static const struct device *const i2c_bus = DEVICE_DT_GET(DT_BUS(EEPROM_NODE));

/* Whole-chip buffers: kept static so they do not sit on the main stack. */
static uint8_t pattern[EEPROM_SIZE];
static uint8_t readback[EEPROM_SIZE];

static unsigned int checks_run;
static unsigned int checks_failed;

static void report(const char *what, bool ok)
{
	checks_run++;
	if (!ok) {
		checks_failed++;
	}

	printk("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
}

static void report_err(const char *what, int err)
{
	checks_run++;
	if (err != 0) {
		checks_failed++;
	}

	printk("  [%s] %s (err %d)\n", err == 0 ? "PASS" : "FAIL", what, err);
}

/*
 * No sleeps between operations here on purpose.  The AT24 family stops
 * answering its address while an internal write cycle runs (tWR, 5 ms max),
 * and eeprom_at24_read()/eeprom_at24_write() both re-issue the transfer
 * until the slave answers again, bounded by the devicetree `timeout`.  So a
 * read issued straight after a write is already safe, and if it ever is not,
 * that is a driver bug worth seeing rather than papering over.
 */

/*
 * The boot record sits in the last bytes of the chip, so the tests that
 * write the whole chip walk straight over it.  Save and restore it around
 * them, otherwise the counter that test 9 maintains is destroyed on every
 * run and never grows - and, worse, test 3 would report a stale magic.
 */
static bool boot_record_save(struct boot_record *out)
{
	return (eeprom_read(eeprom, BOOT_OFFSET, out, sizeof(*out)) == 0) &&
	       (out->magic == BOOT_MAGIC);
}

static void boot_record_restore(const struct boot_record *rec, const char *what)
{
	report_err(what, eeprom_write(eeprom, BOOT_OFFSET, rec, sizeof(*rec)));
}

/*
 * The four pages live on four different slave addresses.  Probing each
 * of them pinpoints a missing chip, a wrong A2 strap or a bus problem
 * before the higher level read/write checks run.
 */
static void test_bus_presence(void)
{
	uint8_t word_addr = 0;
	unsigned int found = 0;
	unsigned int responding = 0;

	printk("1. I2C presence of the %u chip pages\n", EEPROM_PAGE_COUNT);

	/*
	 * Scan a couple of addresses past the four an AT24C08 would use.
	 * A 1 KiB part answers at 0x50..0x53; a 512-byte AT24C04 answers at
	 * 0x50 and 0x51 only, and 0x54..0x57 stay silent either way.  Seeing
	 * exactly two responses is what identifies the smaller chip.
	 */
	for (uint16_t addr = EEPROM_BASE_ADDR; addr < EEPROM_BASE_ADDR + 8; addr++) {
		uint8_t value;
		int ret;

		ret = i2c_write_read(i2c_bus, addr, &word_addr, sizeof(word_addr),
				     &value, sizeof(value));
		if (ret == 0) {
			responding++;
			printk("  bus scan: 0x%02x ACKs (word 0x00 -> 0x%02x)\n",
			       addr, value);
		}
	}

	printk("  bus scan: %u address(es) answered -> %s\n", responding,
	       responding == 2 ? "512-byte AT24C04"
			       : responding == 4 ? "1 KiB AT24C08"
						 : "unrecognised chip");

	for (uint16_t page = 0; page < EEPROM_PAGE_COUNT; page++) {
		uint16_t addr = EEPROM_BASE_ADDR + page;
		uint8_t value;
		int ret;

		ret = i2c_write_read(i2c_bus, addr, &word_addr, sizeof(word_addr),
				     &value, sizeof(value));
		if (ret == 0) {
			found++;
		}

		printk("  [%s] slave 0x%02x (offset 0x%03x) -> 0x%02x\n",
		       ret == 0 ? "PASS" : "FAIL", addr, page * 256,
		       ret == 0 ? value : 0);
		if (ret != 0) {
			checks_run++;
			checks_failed++;
			printk("       i2c_write_read failed (err %d)\n", ret);
		}
	}

	report("all chip pages answer on the bus",
	       found == EEPROM_PAGE_COUNT);
}

static void test_size(void)
{
	size_t size = eeprom_get_size(eeprom);

	printk("2. Reported geometry\n");
	printk("  size=%zu bytes, page=%u bytes, base address=0x%02x\n",
	       size, (unsigned int)EEPROM_PAGE_SIZE,
	       (unsigned int)EEPROM_BASE_ADDR);
	report("eeprom_get_size() matches the devicetree size",
	       size == EEPROM_SIZE);
}

/*
 * Read the boot counter before anything else clobbers the chip, so the
 * increment observed across resets proves the writes really landed in
 * non-volatile memory.
 */
static void test_boot_counter_read(void)
{
	struct boot_record rec;
	int ret;

	printk("3. Persistent boot counter\n");

	ret = eeprom_read(eeprom, BOOT_OFFSET, &rec, sizeof(rec));
	report_err("read of the boot record", ret);
	if (ret < 0) {
		return;
	}

	if (rec.magic == BOOT_MAGIC) {
		printk("  previous boot count: %u\n", rec.count);
	} else {
		printk("  no valid record (magic 0x%08x), starting from 0\n",
		       rec.magic);
	}
}

/*
 * A write must not cross a page boundary inside a single I2C
 * transaction.  The driver splits it; verify the data survives a write
 * that spans two pages.
 */
static void test_page_boundary(void)
{
	uint8_t payload[2 * EEPROM_PAGE_SIZE];
	uint8_t verify[2 * EEPROM_PAGE_SIZE];
	const off_t offset = EEPROM_PAGE_SIZE; /* last byte of page 0 + page 1 */
	int ret;

	printk("4. Write spanning a page boundary\n");

	for (size_t i = 0; i < sizeof(payload); i++) {
		payload[i] = (uint8_t)(0xC3 + i);
	}
	memset(verify, 0, sizeof(verify));

	ret = eeprom_write(eeprom, offset, payload, sizeof(payload));
	report_err("eeprom_write() of 2 pages", ret);
	if (ret < 0) {
		return;
	}

	ret = eeprom_read(eeprom, offset, verify, sizeof(verify));
	report_err("eeprom_read() of 2 pages", ret);
	if (ret < 0) {
		return;
	}

	report("data around the page boundary matches",
	       memcmp(payload, verify, sizeof(payload)) == 0);

	for (size_t i = 0; i < sizeof(payload); i++) {
		if (payload[i] != verify[i]) {
			printk("  first mismatch at offset %d: wrote 0x%02x, read 0x%02x\n",
			       (int)(offset + i), payload[i], verify[i]);
			break;
		}
	}
}

/*
 * Write the whole chip and read it back.  The pattern mixes the high
 * offset bits in, so a page that aliases another one (wrongly decoded
 * page bits) shows up as a mismatch.
 */
static void test_full_chip(void)
{
	struct boot_record saved;
	bool have_boot_record;
	size_t first_mismatch = 0;
	int ret;

	printk("5. Full %u-byte write / read-back\n", (unsigned int)EEPROM_SIZE);

	have_boot_record = boot_record_save(&saved);

	for (size_t i = 0; i < EEPROM_SIZE; i++) {
		pattern[i] = (uint8_t)(0x5A ^ (i & 0xFF) ^ ((i >> 8) * 0x31));
	}
	memset(readback, 0, sizeof(readback));

	ret = eeprom_write(eeprom, 0, pattern, sizeof(pattern));
	report_err("eeprom_write() of the whole chip", ret);
	if (ret < 0) {
		return;
	}

	ret = eeprom_read(eeprom, 0, readback, sizeof(readback));
	report_err("eeprom_read() of the whole chip", ret);
	if (ret < 0) {
		return;
	}

	report("every byte read back identical",
	       memcmp(pattern, readback, sizeof(pattern)) == 0);

	for (size_t i = 0; i < EEPROM_SIZE; i++) {
		if (pattern[i] != readback[i]) {
			first_mismatch = i;
			printk("  first mismatch at 0x%03zx: wrote 0x%02x, read 0x%02x\n",
			       first_mismatch, pattern[first_mismatch],
			       readback[first_mismatch]);
			break;
		}
	}

	/* Spot check the first byte of every 256-byte page. */
	for (size_t page = 0; page < EEPROM_PAGE_COUNT; page++) {
		size_t off = page * 256;

		report("page start byte is addressable", readback[off] == pattern[off]);
	}

	if (have_boot_record) {
		boot_record_restore(&saved, "boot record restored after the full-chip write");
	}
}

static void test_bounds(void)
{
	uint8_t buf[8];
	int ret;

	printk("6. Out-of-range access is rejected\n");

	memset(buf, 0, sizeof(buf));

	ret = eeprom_read(eeprom, EEPROM_SIZE - 4, buf, sizeof(buf));
	report("read past the end returns -EINVAL", ret == -EINVAL);
	if (ret != -EINVAL) {
		printk("       got %d\n", ret);
	}

	ret = eeprom_write(eeprom, EEPROM_SIZE - 4, buf, sizeof(buf));
	report("write past the end returns -EINVAL", ret == -EINVAL);
	if (ret != -EINVAL) {
		printk("       got %d\n", ret);
	}
}

static void test_boot_counter_write(void)
{
	struct boot_record rec;
	struct boot_record verify;
	int ret;

	printk("9. Boot counter update\n");

	ret = eeprom_read(eeprom, BOOT_OFFSET, &rec, sizeof(rec));
	if (ret < 0 || rec.magic != BOOT_MAGIC) {
		rec.magic = BOOT_MAGIC;
		rec.count = 0;
	}
	rec.count++;

	ret = eeprom_write(eeprom, BOOT_OFFSET, &rec, sizeof(rec));
	report_err("write of the boot record", ret);
	if (ret < 0) {
		return;
	}

	ret = eeprom_read(eeprom, BOOT_OFFSET, &verify, sizeof(verify));
	report_err("read-back of the boot record", ret);
	if (ret < 0) {
		return;
	}

	report("boot record round-trips", memcmp(&rec, &verify, sizeof(rec)) == 0);
	printk("  device booted %u time(s) - reset the board to see it grow\n",
	       verify.count);
}

/*
 * Cross-check the eeprom API against bytes put on the bus by hand.
 *
 * Everything above goes through eeprom_read()/eeprom_write(), which share
 * the driver's own offset translation - so if that translation were wrong
 * in the same way for reads and writes, every check above could still pass
 * while the chip held something else entirely.  Here one byte is written
 * and read back with bare i2c_write()/i2c_write_read(), using the slave
 * address and word address worked out by hand, and the two views of the
 * same physical location are compared.
 */
static void test_raw_i2c_ground_truth(void)
{
	/* One byte in the first page and one past 0xFF, so the page bit
	 * really has to move into the slave address (for a 512-byte part
	 * that is 0x1F0 -> slave 0x51, word address 0xF0).
	 */
	static const off_t offsets[] = {0x010, EEPROM_SIZE - 0x10};
	uint8_t via_driver;
	uint8_t via_bus;
	int ret;

	printk("7. Raw I2C ground truth (driver bypassed)\n");

	for (size_t i = 0; i < ARRAY_SIZE(offsets); i++) {
		const off_t off = offsets[i];
		const uint8_t slave = EEPROM_BASE_ADDR + (off >> 8);
		const uint8_t word = off & 0xFF;
		const uint8_t written = (uint8_t)(0xA5 ^ off);
		uint8_t block[2] = {word, written};
		unsigned int attempts;
		int64_t deadline;

		printk("  offset 0x%03x -> slave 0x%02x, word 0x%02x\n",
		       (unsigned int)off, slave, word);

		ret = i2c_write(i2c_bus, block, sizeof(block), slave);
		report_err("raw i2c_write() of one byte", ret);
		if (ret < 0) {
			continue;
		}

		/* Poll the address like the at24 driver does: the chip is deaf
		 * until its write cycle ends.  The raw path has no retry of its
		 * own - that is the point of this test - so the loop is the
		 * test's, not a delay propping up an assertion.  The attempt
		 * count also shows tWR behaves as the datasheet says.
		 */
		deadline = k_uptime_get() + 20;
		attempts = 0;
		while (i2c_write(i2c_bus, &word, 1, slave) != 0) {
			attempts++;
			if (k_uptime_get() > deadline) {
				break;
			}
			k_sleep(K_MSEC(1));
		}

		if (attempts > 0) {
			printk("  chip was busy for ~%u ms after the write\n", attempts);
		}

		via_bus = 0;
		via_driver = 0;
		ret = i2c_write_read(i2c_bus, slave, &word, sizeof(word),
				     &via_bus, sizeof(via_bus));
		report_err("raw i2c_write_read() of one byte", ret);
		if (ret < 0) {
			continue;
		}

		ret = eeprom_read(eeprom, off, &via_driver, sizeof(via_driver));
		report_err("eeprom_read() of the same byte", ret);
		if (ret < 0) {
			continue;
		}

		report("raw bus view matches the driver's view",
		       via_bus == via_driver);
		report("both match the byte that was written",
		       via_bus == written && via_driver == written);

		if (via_bus != written || via_driver != written) {
			printk("       wrote 0x%02x, bus read 0x%02x, driver read 0x%02x\n",
			       written, via_bus, via_driver);
		}
	}
}

/*
 * Walk every offset on the chip, one byte per transaction, writing and then
 * immediately reading that same byte back.  A whole-chip pattern write only
 * proves the pages are reachable; this proves each individual location is,
 * and that a read issued right after a write returns the new value rather
 * than the old one - which is the operation a caller actually performs.
 */
static void test_bytewise_walk(void)
{
	struct boot_record saved;
	bool have_boot_record = boot_record_save(&saved);
	unsigned int mismatches = 0;
	unsigned int write_errors = 0;
	unsigned int read_errors = 0;
	int64_t started = k_uptime_get();

	printk("8. Byte-wise walk over all %u offsets\n", (unsigned int)EEPROM_SIZE);

	for (off_t off = 0; off < (off_t)EEPROM_SIZE; off++) {
		const uint8_t written = (uint8_t)(0x5A ^ off ^ (off >> 8));
		uint8_t seen = 0;
		int ret;

		ret = eeprom_write(eeprom, off, &written, sizeof(written));
		if (ret != 0) {
			if (write_errors++ == 0) {
				printk("  first write error at 0x%03x (err %d)\n",
				       (unsigned int)off, ret);
			}
			continue;
		}

		ret = eeprom_read(eeprom, off, &seen, sizeof(seen));
		if (ret != 0) {
			if (read_errors++ == 0) {
				printk("  first read error at 0x%03x (err %d)\n",
				       (unsigned int)off, ret);
			}
			continue;
		}

		if (seen != written) {
			if (mismatches++ == 0) {
				printk("  first mismatch at 0x%03x: wrote 0x%02x, read 0x%02x\n",
				       (unsigned int)off, written, seen);
			}
		}
	}

	printk("  %u offsets visited in %lld ms (%u write errors, %u read errors)\n",
	       (unsigned int)EEPROM_SIZE, k_uptime_get() - started, write_errors,
	       read_errors);

	report_err("no write error at any offset", write_errors ? -EIO : 0);
	report_err("no read error at any offset", read_errors ? -EIO : 0);
	report("every offset reads back what was just written", mismatches == 0);

	if (have_boot_record) {
		boot_record_restore(&saved, "boot record restored after the walk");
	}
}

int main(void)
{
	printk("\n=== N32G45x I2C AT24 EEPROM test ===\n");
	printk("I2C1: SCL=PB6, SDA=PB7, base address=0x%02x, %u bytes\n\n",
	       (unsigned int)EEPROM_BASE_ADDR, (unsigned int)EEPROM_SIZE);

	if (!device_is_ready(i2c_bus)) {
		printk("I2C bus \"%s\" is not ready\n", i2c_bus->name);
		return 0;
	}
	printk("I2C bus \"%s\" ready\n", i2c_bus->name);

	if (!device_is_ready(eeprom)) {
		printk("EEPROM \"%s\" is not ready - check the I2C wiring and "
		       "the driver init log\n", eeprom->name);
		return 0;
	}
	printk("EEPROM \"%s\" ready\n\n", eeprom->name);

	test_bus_presence();
	test_size();
	test_boot_counter_read();
	test_page_boundary();
	test_full_chip();
	test_bounds();
	test_raw_i2c_ground_truth();
	test_bytewise_walk();

	/* Last, so the walk above cannot clobber the record it leaves behind. */
	test_boot_counter_write();

	printk("\n=== %u checks, %u failed ===\n", checks_run, checks_failed);
	printk(checks_failed == 0 ? "ALL TESTS PASSED\n" : "TESTS FAILED\n");

	return 0;
}
