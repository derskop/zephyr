/*
 * Copyright (c) 2026 Nations Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Bring-up test for the N32G45x QSPI controller, driving the board's P25Q40HA
 * through the raw MSPI API rather than through the flash device layer. The
 * point is to report on the controller itself, one layer at a time, so that a
 * failure says which layer it came from instead of just "the flash did not
 * work":
 *
 *   1. JEDEC ID over 8-8-8 (command 0x9f). Needs the clock, the chip select and
 *      the four pins to be right, and nothing else. Its answer is a constant,
 *      so it also settles byte ordering: a shift of one bit or one byte in the
 *      frame path changes 85 60 13 into something else.
 *
 *   2. Clearing the block protect bits and setting QE, which quad transfers
 *      need. Done over 8-8-8, so a failure here is not a quad problem.
 *
 *   3. Erasing a sector and programming two 16 byte patterns into it, still
 *      over 8-8-8. This is what gives the reads below something to compare
 *      against that is not simply erased flash.
 *
 *   4. Two consecutive 8-8-8 reads of those patterns, from adjacent addresses.
 *      This is the frame count test, and it needs both reads. If the controller
 *      clocks one frame too many, the first read still returns the right bytes
 *      -- the extra frame stays in the receive FIFO -- and the *second* read
 *      starts one frame late, returning the last byte of the first read as its
 *      own first byte. Of the two ways to get the frame count wrong, one shows
 *      up here and one shows up as a read that is a byte short.
 *
 *   5. The same two regions read back over quad data lines (command 0x6b) and
 *      compared byte for byte against step 4. Both ways of putting the address
 *      on the bus are tried, because the vendor example leaves the transfer type
 *      at its default for a command whose name says otherwise, and the two
 *      differ only in this one field. The output names the variant that worked,
 *      which is what belongs in the devicetree.
 *
 *   6. Erasing a second sector and programming it three ways, reading each back
 *      over 8-8-8: a short run over quad data lines (command 0x32), a whole 256
 *      byte page over quad data lines, and a whole page over one line (0x02).
 *      This is the write half, which steps 4 and 5 do not cover. Only the data
 *      phase of the quad commands goes over four lines: the write enable before
 *      them and the status poll after them are 8-8-8 commands, and the device
 *      has to be put back in the control I/O mode for those. A device left in
 *      quad mode ignores them outright and answers the status register with
 *      whatever the four lines carry, which is a failure that looks like a dead
 *      flash rather than like a mode that was not switched. The two page
 *      programs are then written again at 6 MHz, which is what separates a
 *      write that is wrong from a write whose feed could not keep up with the
 *      bus.
 *
 * The read-backs in step 6 are longer than the controller's FIFO, so the driver
 * runs each of them as several transactions of its own accord -- see
 * N32_QSPI_RX_CHUNK_FRAMES in drivers/mspi/mspi_n32.c. Nothing on this side has
 * to know that: a read of any length is one call.
 *
 * Steps 3 and 6 erase the first two sectors of the flash. Anything stored
 * there is destroyed.
 */

#include <stdio.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/mspi.h>
#include <zephyr/drivers/mspi/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(n32_qspi_flash, LOG_LEVEL_INF);

#define QSPI_NODE	DT_NODELABEL(qspi)
#define FLASH_NODE	DT_NODELABEL(p25q40ha_qspi)

/* The opcodes this test uses, named after what the flash does with them. */
#define NOR_CMD_WRITE_ENABLE	0x06U
#define NOR_CMD_READ_STATUS1	0x05U
#define NOR_CMD_READ_STATUS2	0x35U
#define NOR_CMD_WRITE_STATUS	0x01U
#define NOR_CMD_JEDEC_ID	0x9fU
#define NOR_CMD_RELEASE_POWER_DOWN 0xabU
#define NOR_CMD_READ		0x03U
#define NOR_CMD_FAST_READ_QUAD	0x6bU
#define NOR_CMD_PAGE_PROGRAM	0x02U
#define NOR_CMD_QUAD_PAGE_PROGRAM 0x32U
#define NOR_CMD_SECTOR_ERASE	0x20U

#define NOR_STATUS1_BUSY	BIT(0)
#define NOR_STATUS2_QE		BIT(1)

#define NOR_PAGE_SIZE		256U
#define NOR_SECTOR_SIZE		4096U

/* A 24 bit address is three frames in 8-8-8 mode and 24 bits in the address
 * length field of the enhanced one, which is what the driver takes here.
 */
#define NOR_ADDR_LEN_3BYTE	3U

/* The API refuses a timeout above the tolerance the driver was built with. */
#define NOR_TIMEOUT_MS		CONFIG_MSPI_COMPLETION_TIMEOUT_TOLERANCE

/* Both patterns live in the first sector, 16 bytes apart, so that the second
 * read of step 4 can tell a late start from a correct one by its first byte.
 */
#define NOR_PATTERN_A_OFFSET	0x00U
#define NOR_PATTERN_B_OFFSET	0x10U
#define NOR_PATTERN_LEN		16U

/* Step 6 needs a sector of its own, and writes into it three ways: a page over
 * four data lines, a short run over four lines, and a page over one. Each gets
 * its own page, so that a write which went wrong cannot be read back as if it
 * had gone right. The same two page programs are written a second time at a
 * slower clock, into pages of their own, which is what separates a write that is
 * wrong from a write whose feed could not keep up with the bus.
 */
#define NOR_PAGE_OFFSET		0x00U
#define NOR_SHORT_OFFSET	0x100U
#define NOR_SHORT_LEN		16U
#define NOR_SINGLE_OFFSET	0x200U
#define NOR_SLOW_QUAD_OFFSET	0x400U
#define NOR_SLOW_SINGLE_OFFSET	0x500U

/*
 * The clock the two page programs are written at a second time. A frame of an
 * 8-8-8 data phase lasts eight clocks and one of a four line data phase two, so
 * at 6 MHz a feed has 1.33 us and 333 ns per frame to work with rather than the
 * 333 ns and 83 ns the node's own 24 MHz leaves it.
 */
#define NOR_SLOW_FREQ		6000000U

/*
 * Puya's manufacturer code, the SPI NOR memory type, and the capacity code for
 * 4 Mbit: the P25Q40HA, not the 128 Mbit part the board is sold as.
 */
static const uint8_t nor_jedec_id[3] = {0x85U, 0x60U, 0x13U};

static const struct device *const qspi = DEVICE_DT_GET(QSPI_NODE);
static const struct mspi_dev_id dev_id = MSPI_DEVICE_ID_DT(FLASH_NODE);

static unsigned int checks;
static unsigned int failed;

static void check(bool ok, const char *what)
{
	checks++;

	if (ok) {
		LOG_INF("  PASS  %s", what);
		return;
	}

	failed++;
	LOG_ERR("  FAIL  %s", what);
}

static void dump_hex(const char *label, const uint8_t *buf, size_t len)
{
	for (size_t i = 0U; i < len; i += 16U) {
		char line[3U * 16U];
		size_t chunk = MIN(len - i, 16U);
		size_t pos = 0U;

		for (size_t j = 0U; j < chunk; j++) {
			pos += (size_t)snprintf(&line[pos], sizeof(line) - pos, "%02x ",
						buf[i + j]);
		}

		LOG_INF("    %s +%02x: %s", label, (unsigned int)i, line);
	}
}

/*
 * One packet. `addr_len` is in bytes and is 0 for the commands that carry no
 * address, which is how the driver tells those from an address of zero.
 *
 * There is no I/O mode here: mspi_xfer has no field for one, so every transfer
 * goes out in whatever mode the device was last configured with, through
 * mspi_dev_config(). A caller that wants a command on one line has to put the
 * device back in that mode first -- nor_erase_sector(), nor_program() and
 * nor_program_quad() below do it for themselves for that reason.
 */
static int nor_xfer(uint32_t cmd, uint32_t addr, uint8_t addr_len, uint16_t dummy,
		    enum mspi_xfer_direction dir, uint8_t *buf, size_t len)
{
	struct mspi_xfer_packet packet = {
		.dir = dir,
		.cmd = cmd,
		.address = addr,
		.num_bytes = (uint32_t)len,
		.data_buf = buf,
	};
	struct mspi_xfer xfer = {
		.cmd_length = 1U,
		.addr_length = addr_len,
		.rx_dummy = dummy,
		.packets = &packet,
		.num_packet = 1U,
		.timeout = NOR_TIMEOUT_MS,
	};

	return mspi_transceive(qspi, &dev_id, &xfer);
}

static int nor_read(uint32_t cmd, uint32_t addr, uint8_t addr_len, uint16_t dummy,
		    uint8_t *buf, size_t len)
{
	return nor_xfer(cmd, addr, addr_len, dummy, MSPI_RX, buf, len);
}

static int nor_write(uint32_t cmd, uint32_t addr, uint8_t addr_len, const uint8_t *buf,
		     size_t len)
{
	/* The packet holds a non-const pointer, but a transmit only reads it. */
	return nor_xfer(cmd, addr, addr_len, 0U, MSPI_TX, (uint8_t *)buf, len);
}

static int nor_cmd(uint32_t cmd)
{
	return nor_xfer(cmd, 0U, 0U, 0U, MSPI_TX, NULL, 0U);
}

/* Poll the write in progress bit, which is the only completion signal a NOR
 * flash offers. A sector erase is the slow one at a few hundred milliseconds.
 */
static int nor_wait_ready(void)
{
	uint8_t status = 0U;
	int ret;

	for (unsigned int i = 0U; i < 2000U; i++) {
		ret = nor_read(NOR_CMD_READ_STATUS1, 0U, 0U, 0U, &status, sizeof(status));
		if (ret < 0) {
			return ret;
		}

		if ((status & NOR_STATUS1_BUSY) == 0U) {
			return 0;
		}

		k_msleep(1);
	}

	LOG_ERR("the flash never cleared its busy bit");
	return -ETIMEDOUT;
}

static int nor_set_io_mode(enum mspi_io_mode io_mode)
{
	struct mspi_dev_cfg cfg = {
		.io_mode = io_mode,
	};

	/*
	 * The session the first mspi_dev_config() opened is still held, so this
	 * only replaces the stored I/O mode; the block is reprogrammed on the
	 * next packet either way.
	 */
	return mspi_dev_config(qspi, &dev_id, MSPI_DEVICE_CONFIG_IO_MODE, &cfg);
}

/*
 * The same for the clock. The driver recomputes its divider and the next packet
 * goes out at the new rate; the mode the device was last configured with is not
 * touched, so this and nor_set_io_mode() can be used in either order.
 */
static int nor_set_freq(uint32_t freq)
{
	struct mspi_dev_cfg cfg = {
		.freq = freq,
	};

	return mspi_dev_config(qspi, &dev_id, MSPI_DEVICE_CONFIG_FREQUENCY, &cfg);
}

static void stage1_jedec_id(void)
{
	uint8_t id[3] = {0U};
	int ret;

	LOG_INF("Step 1: JEDEC ID, 8-8-8, command 0x%02x", NOR_CMD_JEDEC_ID);

	ret = nor_read(NOR_CMD_JEDEC_ID, 0U, 0U, 0U, id, sizeof(id));
	if (ret < 0) {
		LOG_ERR("  the JEDEC ID read failed: %d", ret);
		check(false, "JEDEC ID read over 8-8-8");
		return;
	}

	/*
	 * A part that does not answer here may be in deep power down. Command 0xb9
	 * is one of the 256 byte values a page program puts on the bus, so a
	 * transfer that goes wrong badly enough for the flash to read its own data
	 * as a command sequence can leave it in that state -- where it ignores
	 * everything except the release command and only a power cycle clears it,
	 * because an MCU reset does not reach the flash. Saying which of the two
	 * happened is worth a line of output: an ID that needs 0xab means the
	 * previous run left garbage on the bus, not that the wiring is wrong.
	 */
	if (memcmp(id, nor_jedec_id, sizeof(nor_jedec_id)) != 0) {
		dump_hex("id", id, sizeof(id));
		LOG_WRN("  trying %02x in case the flash is powered down",
			NOR_CMD_RELEASE_POWER_DOWN);

		ret = nor_cmd(NOR_CMD_RELEASE_POWER_DOWN);
		if (ret < 0) {
			LOG_ERR("  the release command failed: %d", ret);
		} else {
			/* tRES1 is under 10 us; the flash ignores the bus until then. */
			k_msleep(2);
			ret = nor_read(NOR_CMD_JEDEC_ID, 0U, 0U, 0U, id, sizeof(id));
			if (ret < 0) {
				LOG_ERR("  the JEDEC ID read failed: %d", ret);
			}
		}

		if (memcmp(id, nor_jedec_id, sizeof(nor_jedec_id)) == 0) {
			LOG_WRN("  the flash answered only after 0x%02x",
				NOR_CMD_RELEASE_POWER_DOWN);
		}
	}

	dump_hex("id", id, sizeof(id));
	check(memcmp(id, nor_jedec_id, sizeof(nor_jedec_id)) == 0, "the flash answers 85 60 13");
}

/*
 * The ID read on its own, for the checks that have to know the flash is still
 * answering without carrying a step's worth of output.
 */
static bool nor_id_ok(void)
{
	uint8_t id[3] = {0U};

	if (nor_set_io_mode(MSPI_IO_MODE_SINGLE) < 0) {
		return false;
	}

	if (nor_read(NOR_CMD_JEDEC_ID, 0U, 0U, 0U, id, sizeof(id)) < 0) {
		return false;
	}

	if (memcmp(id, nor_jedec_id, sizeof(nor_jedec_id)) != 0) {
		dump_hex("id", id, sizeof(id));
		return false;
	}

	return true;
}

static void stage2_protection(void)
{
	uint8_t sr2 = 0U;
	const uint8_t sr[2] = {0x00U, NOR_STATUS2_QE};
	int ret;

	LOG_INF("Step 2: clear block protection and set QE");

	ret = nor_read(NOR_CMD_READ_STATUS2, 0U, 0U, 0U, &sr2, sizeof(sr2));
	check(ret == 0, "status register 2 can be read");

	if ((sr2 & NOR_STATUS2_QE) != 0U) {
		LOG_INF("  QE was already set (status register 2 = %02x)", sr2);
		return;
	}

	/*
	 * One write of two bytes: status register 1 with the block protect bits
	 * cleared -- some parts arrive with them set and would refuse the erase --
	 * and status register 2 with QE set.
	 */
	ret = nor_cmd(NOR_CMD_WRITE_ENABLE);
	check(ret == 0, "write enable is accepted");

	ret = nor_write(NOR_CMD_WRITE_STATUS, 0U, 0U, sr, sizeof(sr));
	check(ret == 0, "the status registers are written");

	ret = nor_wait_ready();
	check(ret == 0, "the flash returns from the status write");

	ret = nor_read(NOR_CMD_READ_STATUS2, 0U, 0U, 0U, &sr2, sizeof(sr2));
	check(ret == 0 && (sr2 & NOR_STATUS2_QE) != 0U, "status register 2 reports QE");
}

/*
 * The erase, the write enable before it and the status polls after it are all
 * single-line commands: 0x20 has one form and it is not the multi-line one, so
 * a device left in quad I/O mode ignores it and answers the status register
 * with whatever the four lines happen to carry. The flash layer switches the
 * same way in perform_xfer(), where every command that is not the page program
 * goes back to the control I/O mode. The I/O mode is not a property of a
 * transfer -- mspi_xfer has no field for it -- so it has to be set here.
 */
static int nor_erase_sector(uint32_t addr)
{
	int ret;

	ret = nor_set_io_mode(MSPI_IO_MODE_SINGLE);
	if (ret < 0) {
		return ret;
	}

	ret = nor_cmd(NOR_CMD_WRITE_ENABLE);
	if (ret < 0) {
		return ret;
	}

	ret = nor_write(NOR_CMD_SECTOR_ERASE, addr, NOR_ADDR_LEN_3BYTE, NULL, 0U);
	if (ret < 0) {
		return ret;
	}

	return nor_wait_ready();
}

/* The single-line program path: 0x02, and the write enable and status polls
 * around it, which are 8-8-8 commands like the erase above.
 */
static int nor_program(uint32_t cmd, uint32_t addr, const uint8_t *data, size_t len)
{
	int ret = nor_set_io_mode(MSPI_IO_MODE_SINGLE);

	if (ret < 0) {
		return ret;
	}

	ret = nor_cmd(NOR_CMD_WRITE_ENABLE);
	if (ret < 0) {
		return ret;
	}

	ret = nor_write(cmd, addr, NOR_ADDR_LEN_3BYTE, data, len);
	if (ret < 0) {
		return ret;
	}

	return nor_wait_ready();
}

/*
 * The quad page program, 0x32. Only the data phase goes over four lines: the
 * write enable before it and the status poll after it are 8-8-8 commands, so
 * the device is switched back to the control mode around the one transfer that
 * needs the other one. That is the flash layer's rule in perform_xfer(), where
 * only the page program command takes the write configuration.
 */
static int nor_program_quad(uint32_t addr, const uint8_t *data, size_t len)
{
	int ret;

	ret = nor_set_io_mode(MSPI_IO_MODE_SINGLE);
	if (ret < 0) {
		return ret;
	}

	ret = nor_cmd(NOR_CMD_WRITE_ENABLE);
	if (ret < 0) {
		return ret;
	}

	ret = nor_set_io_mode(MSPI_IO_MODE_QUAD_1_1_4);
	if (ret < 0) {
		return ret;
	}

	ret = nor_write(NOR_CMD_QUAD_PAGE_PROGRAM, addr, NOR_ADDR_LEN_3BYTE, data, len);

	/*
	 * Back to 8-8-8 whether or not the transfer went out: the device stays in
	 * whatever mode it was last configured with, and a transfer that failed
	 * is when the next command is least likely to be looked at.
	 */
	if (nor_set_io_mode(MSPI_IO_MODE_SINGLE) < 0) {
		return -EIO;
	}

	if (ret < 0) {
		return ret;
	}

	return nor_wait_ready();
}

/*
 * Read a region back over 8-8-8 and compare it with what was written. The
 * comparison reports the first byte that differs, because a page program can
 * only clear bits: a byte that reads ff was never programmed, so the transfer
 * stopped before it reached that offset, while a byte that reads anything else
 * was programmed with something other than what was sent. Which of the two it
 * is, and where it starts, is most of the diagnosis.
 */
static bool nor_verify(const char *what, uint32_t addr, const uint8_t *want, size_t len)
{
	uint8_t got[NOR_PAGE_SIZE];
	int ret;

	ret = nor_set_io_mode(MSPI_IO_MODE_SINGLE);
	if (ret < 0) {
		LOG_ERR("  could not go back to 8-8-8: %d", ret);
		return false;
	}

	ret = nor_read(NOR_CMD_READ, addr, NOR_ADDR_LEN_3BYTE, 0U, got, len);
	if (ret < 0) {
		LOG_ERR("  the read-back of the %s failed: %d", what, ret);
		return false;
	}

	for (size_t i = 0U; i < len; i++) {
		if (got[i] != want[i]) {
			LOG_ERR("  the %s: first difference at +%02x: wrote %02x, read %02x",
				what, (unsigned int)i, want[i], got[i]);
			return false;
		}
	}

	return true;
}

static void stage3_program_patterns(uint8_t *pattern_a, uint8_t *pattern_b)
{
	int ret;

	LOG_INF("Step 3: erase the first sector and program two patterns, 8-8-8");

	for (size_t i = 0U; i < NOR_PATTERN_LEN; i++) {
		pattern_a[i] = (uint8_t)(0xa0U + i);
		pattern_b[i] = (uint8_t)(0xe0U + i);
	}

	ret = nor_erase_sector(0U);
	check(ret == 0, "the first sector erases");

	ret = nor_program(NOR_CMD_PAGE_PROGRAM, NOR_PATTERN_A_OFFSET, pattern_a,
			  NOR_PATTERN_LEN);
	check(ret == 0, "the first pattern programs");

	ret = nor_program(NOR_CMD_PAGE_PROGRAM, NOR_PATTERN_B_OFFSET, pattern_b,
			  NOR_PATTERN_LEN);
	check(ret == 0, "the second pattern programs");
}

/*
 * The frame count test. Both reads happen back to back, with nothing in
 * between, because the residue a mis-sized read leaves is only visible to the
 * transfer that follows it.
 */
static void stage4_two_reads(const uint8_t *pattern_a, const uint8_t *pattern_b,
			     uint8_t *read_a, uint8_t *read_b)
{
	int ret_a;
	int ret_b;

	LOG_INF("Step 4: two consecutive 8-8-8 reads of adjacent regions");

	ret_a = nor_read(NOR_CMD_READ, NOR_PATTERN_A_OFFSET, NOR_ADDR_LEN_3BYTE, 0U, read_a,
			 NOR_PATTERN_LEN);
	ret_b = nor_read(NOR_CMD_READ, NOR_PATTERN_B_OFFSET, NOR_ADDR_LEN_3BYTE, 0U, read_b,
			 NOR_PATTERN_LEN);

	if (ret_a < 0) {
		LOG_ERR("  the first read failed: %d", ret_a);
		check(false, "the first read completes");
		return;
	}

	dump_hex("a", read_a, NOR_PATTERN_LEN);
	check(memcmp(read_a, pattern_a, NOR_PATTERN_LEN) == 0, "the first read matches");

	if (ret_b < 0) {
		LOG_ERR("  the second read failed: %d", ret_b);
		check(false, "the second read completes");
		return;
	}

	dump_hex("b", read_b, NOR_PATTERN_LEN);

	/*
	 * Spelled out separately from the comparison because it is the one
	 * failure that has a specific meaning: a first byte equal to the last
	 * byte of the first read means this transfer started one frame late,
	 * which is a frame count one too high.
	 */
	if (read_b[0] == read_a[NOR_PATTERN_LEN - 1U] && pattern_b[0] != pattern_a[0]) {
		LOG_ERR("  the second read starts with the last byte of the first one");
	}

	check(memcmp(read_b, pattern_b, NOR_PATTERN_LEN) == 0,
	      "the second read matches, so the frame count is not one too high");
}

/*
 * Step 5. The same region read over four data lines, once with the address on
 * one line (1-1-4, which is what 0x6b asks for) and once with the address on
 * four (1-4-4). Only one of them can work, and the field that selects
 * between them is the one the vendor example leaves at its default.
 */
static void stage5_quad_read(const uint8_t *pattern_a, const uint8_t *pattern_b)
{
	static const struct {
		const char *name;
		enum mspi_io_mode io_mode;
		uint8_t cmd;
	} variants[] = {
		{"1-1-4 (0x6b), address on one line", MSPI_IO_MODE_QUAD_1_1_4,
		 NOR_CMD_FAST_READ_QUAD},
		{"1-4-4 (0x6b), address on four lines", MSPI_IO_MODE_QUAD_1_4_4,
		 NOR_CMD_FAST_READ_QUAD},
	};
	uint8_t read_a[NOR_PATTERN_LEN];
	uint8_t read_b[NOR_PATTERN_LEN];
	bool any = false;
	int ret;

	LOG_INF("Step 5: quad read, 8 dummy cycles, compared against step 4");

	for (size_t v = 0U; v < ARRAY_SIZE(variants); v++) {
		ret = nor_set_io_mode(variants[v].io_mode);
		if (ret < 0) {
			LOG_ERR("  %s: the mode was refused: %d", variants[v].name, ret);
			continue;
		}

		/* 0x6b has eight dummy cycles between the address and the data. */
		ret = nor_read(variants[v].cmd, NOR_PATTERN_A_OFFSET, NOR_ADDR_LEN_3BYTE, 8U,
			       read_a, sizeof(read_a));
		if (ret == 0) {
			ret = nor_read(variants[v].cmd, NOR_PATTERN_B_OFFSET,
				       NOR_ADDR_LEN_3BYTE, 8U, read_b, sizeof(read_b));
		}

		if (ret < 0) {
			LOG_ERR("  %s: the read failed: %d", variants[v].name, ret);
			continue;
		}

		dump_hex("quad a", read_a, sizeof(read_a));
		dump_hex("quad b", read_b, sizeof(read_b));

		if (memcmp(read_a, pattern_a, sizeof(read_a)) == 0 &&
		    memcmp(read_b, pattern_b, sizeof(read_b)) == 0) {
			LOG_INF("  PASS  %s", variants[v].name);
			LOG_INF("        put mspi-io-mode = \"%s\" in the flash node",
				variants[v].io_mode == MSPI_IO_MODE_QUAD_1_1_4
					? "MSPI_IO_MODE_QUAD_1_1_4"
					: "MSPI_IO_MODE_QUAD_1_4_4");
			any = true;
		} else {
			LOG_WRN("  ..    %s: the bytes do not match", variants[v].name);
		}
	}

	check(any, "one of the two quad variants reads the patterns back");
}

/*
 * Step 6. The write half, which the reads above do not touch. Three writes, to
 * a page of their own inside one erased sector, each read back over one data
 * line:
 *
 *   - a short run over four data lines. Its whole packet fits in the transmit
 *     FIFO in one go, so the CPU pushes it before CE goes down and the driver's
 *     polling loop has nothing left to do.
 *   - a page over four data lines. That packet does not fit, so its payload
 *     goes out on the controller's transmit DMA channel -- see the feed
 *     decision in drivers/mspi/mspi_n32.c. A page program is the shape the
 *     flash needs and the shape the CPU cannot feed.
 *   - a page over one data line, which is what the flash layer programs pages
 *     with, and takes the same channel by a different format.
 *
 * The short run is what keeps the two apart on a failure: if it reads back and a
 * page does not, the frames are right and the feed is not; if the short run
 * fails too, it is the frames.
 *
 * The two long pages are then written again at 6 MHz. A transmit whose feed cannot
 * keep up with the bus fails nothing: the core ends the transfer where its FIFO
 * ran dry, no count and no error flag reports it, and the flash commits whatever
 * did arrive. A write that is correct at one clock and not at another is therefore
 * a write whose problem is the feed, and the control pair is what says which of
 * the two it is.
 */
static void stage6_write_paths(void)
{
	struct mspi_dev_cfg dt_cfg = MSPI_DEVICE_CONFIG_DT(FLASH_NODE);
	uint8_t page[NOR_PAGE_SIZE];
	uint8_t short_page[NOR_SHORT_LEN];
	uint8_t readback[NOR_PAGE_SIZE];
	bool erased = false;
	int ret;

	LOG_INF("Step 6: erase the second sector, program it three ways, read each back");

	for (size_t i = 0U; i < sizeof(page); i++) {
		page[i] = (uint8_t)(i ^ 0x5aU);
	}

	for (size_t i = 0U; i < sizeof(short_page); i++) {
		short_page[i] = (uint8_t)(i ^ 0x5aU);
	}

	/*
	 * The erase runs in the control I/O mode like every other 8-8-8 command;
	 * it is the page program below that puts the device in quad. The quad
	 * program itself only works with QE set, which step 2 established.
	 */
	ret = nor_erase_sector(NOR_SECTOR_SIZE);
	if (ret < 0) {
		LOG_WRN("  the erase did not take: %d", ret);
	}

	/*
	 * Read the page back before anything is written into it. An erase that
	 * quietly does nothing leaves whatever was there before -- including what
	 * a previous run of this sample wrote, which is the same pattern -- and
	 * the comparison at the end of this step would then pass on it. This
	 * read-back is what makes the erase itself a check.
	 */
	if (ret == 0) {
		ret = nor_read(NOR_CMD_READ, NOR_SECTOR_SIZE + NOR_PAGE_OFFSET, NOR_ADDR_LEN_3BYTE,
			       0U, readback, sizeof(readback));
		if (ret < 0) {
			LOG_WRN("  the read after the erase failed: %d", ret);
		} else {
			erased = true;

			for (size_t i = 0U; i < sizeof(readback); i++) {
				if (readback[i] != 0xffU) {
					LOG_WRN("  byte +%02x reads %02x after the erase",
						(unsigned int)i, readback[i]);
					erased = false;
					break;
				}
			}
		}
	}

	check(erased, "the second sector erases to ff over 8-8-8");

	if (!erased) {
		return;
	}

	LOG_INF("  the writes below run at the node's own %u Hz", dt_cfg.freq);

	ret = nor_program_quad(NOR_SECTOR_SIZE + NOR_SHORT_OFFSET, short_page,
			       sizeof(short_page));
	check(ret == 0, "a short run programs over four data lines (0x32)");
	check(nor_verify("short run over four lines", NOR_SECTOR_SIZE + NOR_SHORT_OFFSET,
			 short_page, sizeof(short_page)),
	      "the short quad write reads back as written");

	/*
	 * Ask the flash who it is before the next write goes out. A quad transfer
	 * the controller does not finish cleanly can leave the flash reading what
	 * it is being sent as a command sequence, and one of the 256 byte values
	 * in the page below is 0xb9 -- deep power down, which no MCU reset clears
	 * because the reset does not reach the flash. This read separates "the
	 * quad write was wrong" from "the quad write stopped the flash answering",
	 * and it has to happen while the bus is still in the state that did it.
	 */
	check(nor_id_ok(), "the flash still answers its ID between the quad writes");

	ret = nor_program_quad(NOR_SECTOR_SIZE + NOR_PAGE_OFFSET, page, sizeof(page));
	check(ret == 0, "the page programs over four data lines (0x32)");
	check(nor_verify("page written over four lines", NOR_SECTOR_SIZE + NOR_PAGE_OFFSET,
			 page, sizeof(page)),
	      "the quad page reads back as written");

	ret = nor_program(NOR_CMD_PAGE_PROGRAM, NOR_SECTOR_SIZE + NOR_SINGLE_OFFSET, page,
			  sizeof(page));
	check(ret == 0, "the page programs over one data line (0x02)");
	check(nor_verify("page written over one line", NOR_SECTOR_SIZE + NOR_SINGLE_OFFSET,
			 page, sizeof(page)),
	      "the 8-8-8 page reads back as written");

	/*
	 * The same two page programs again at a clock slow enough that no feed
	 * could fall behind them: eight clocks a frame at 6 MHz leaves an 8-8-8
	 * data phase 1.33 us per frame and a four line one 333 ns, against 333 ns
	 * and 83 ns at the node's own 24 MHz. These two are the control. If they
	 * read back and the ones above did not, then the frames this sample sends
	 * are right at either speed and what the fast pair is missing is a feed
	 * that can keep up with them -- which is a property of the bus rate and
	 * not of the packet, and it is the same packet.
	 */
	if (nor_set_freq(NOR_SLOW_FREQ) < 0) {
		LOG_ERR("  the %u Hz clock was refused", NOR_SLOW_FREQ);
		return;
	}

	LOG_INF("  the same two writes again at %u Hz", NOR_SLOW_FREQ);

	ret = nor_program_quad(NOR_SECTOR_SIZE + NOR_SLOW_QUAD_OFFSET, page, sizeof(page));
	check(ret == 0, "the page programs over four data lines at 6 MHz");
	check(nor_verify("page over four lines at 6 MHz",
			 NOR_SECTOR_SIZE + NOR_SLOW_QUAD_OFFSET, page, sizeof(page)),
	      "the 6 MHz quad page reads back as written");

	ret = nor_program(NOR_CMD_PAGE_PROGRAM, NOR_SECTOR_SIZE + NOR_SLOW_SINGLE_OFFSET, page,
			  sizeof(page));
	check(ret == 0, "the page programs over one data line at 6 MHz");
	check(nor_verify("page over one line at 6 MHz",
			 NOR_SECTOR_SIZE + NOR_SLOW_SINGLE_OFFSET, page, sizeof(page)),
	      "the 6 MHz 8-8-8 page reads back as written");

	/* Leave the bus at the rate the node asked for. */
	if (nor_set_freq(dt_cfg.freq) < 0) {
		LOG_ERR("  could not put the clock back to %u Hz", dt_cfg.freq);
	}
}

int main(void)
{
	struct mspi_dt_spec spec = {
		.bus = qspi,
		.config = {
			.channel_num = 0U,
			.op_mode = MSPI_OP_MODE_CONTROLLER,
			.duplex = MSPI_HALF_DUPLEX,
			.num_periph = 1U,
		},
	};
	struct mspi_dev_cfg dev_cfg = MSPI_DEVICE_CONFIG_DT(FLASH_NODE);
	uint8_t pattern_a[NOR_PATTERN_LEN];
	uint8_t pattern_b[NOR_PATTERN_LEN];
	uint8_t read_a[NOR_PATTERN_LEN];
	uint8_t read_b[NOR_PATTERN_LEN];
	int ret;

	/*
	 * Hold off before the first line of output. This sample is run by hand
	 * on a board whose reset is a button or a debug probe, and the terminal
	 * that reads the log is attached after that reset, so the checks would
	 * otherwise scroll past before anyone is watching. The delay is on the
	 * sample, not the driver: nothing about the controller or the flash
	 * needs it.
	 */
	k_sleep(K_SECONDS(3));

	LOG_INF("N32 QSPI controller test on %s", CONFIG_BOARD_TARGET);

	if (!device_is_ready(qspi)) {
		LOG_ERR("%s is not ready", qspi->name);
		return 0;
	}

	ret = mspi_config(&spec);
	if (ret < 0) {
		LOG_ERR("mspi_config() failed: %d", ret);
		return 0;
	}

	/*
	 * This opens the session: the driver holds the controller from here
	 * until mspi_get_channel_status() below hands it back.
	 */
	ret = mspi_dev_config(qspi, &dev_id, MSPI_DEVICE_CONFIG_ALL, &dev_cfg);
	if (ret < 0) {
		LOG_ERR("mspi_dev_config() failed: %d", ret);
		return 0;
	}

	LOG_INF("flash device index %u at %u Hz", dev_id.dev_idx, dev_cfg.freq);

	stage1_jedec_id();
	if (failed != 0U) {
		LOG_ERR("the controller is not getting an answer from the flash; stopping");
		goto out;
	}

	stage2_protection();
	stage3_program_patterns(pattern_a, pattern_b);
	stage4_two_reads(pattern_a, pattern_b, read_a, read_b);
	stage5_quad_read(pattern_a, pattern_b);
	stage6_write_paths();

	/* Going back to 8-8-8 leaves the bus in the state the board starts in. */
	(void)nor_set_io_mode(MSPI_IO_MODE_SINGLE);

out:
	ret = mspi_get_channel_status(qspi, 0U);
	check(ret == 0, "the session is handed back to the controller");

	LOG_INF("%u of %u checks failed", failed, checks);

	return 0;
}
