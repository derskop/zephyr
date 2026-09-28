/*
 * Copyright (c) 2026 Nations Technologies
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>

/* 7-bit target address of the EEPROM. */
#define AT24_ADDR 0x50
/* Memory address inside the EEPROM to write to and read back. */
#define AT24_MEM 0x00
#define AT24_DATA 0x55

int main(void)
{
	const struct device *i2c = DEVICE_DT_GET(DT_NODELABEL(i2c1));
	uint8_t mem_addr = AT24_MEM;
	uint8_t wr_data = AT24_DATA;
	uint8_t wr_buf[2] = { mem_addr, wr_data };
	uint8_t rd_data = 0x00;
	int ret;

	if (!device_is_ready(i2c)) {
		printf("I2C1 not ready\n");
		return 0;
	}

	/* Configure for 100 kHz standard mode. */
	ret = i2c_configure(i2c, I2C_SPEED_SET(I2C_SPEED_STANDARD));
	if (ret < 0) {
		printf("i2c_configure failed: %d\n", ret);
		return 0;
	}

	/* Write: memory address followed by the data byte. */
	ret = i2c_write(i2c, wr_buf, sizeof(wr_buf), AT24_ADDR);
	if (ret < 0) {
		printf("i2c_write failed: %d\n", ret);
		return 0;
	}

	/* Wait for the EEPROM internal write cycle to finish (~5 ms). */
	k_sleep(K_MSEC(10));

	/* Read: send the memory address, repeated START, then read one byte. */
	ret = i2c_write_read(i2c, AT24_ADDR, &mem_addr, 1, &rd_data, 1);
	if (ret < 0) {
		printf("i2c_read failed: %d\n", ret);
		return 0;
	}

	printf("write=0x%02x read=0x%02x  =>  %s\n",
	       wr_data, rd_data, (rd_data == wr_data) ? "PASS" : "FAIL");

	return 0;
}
