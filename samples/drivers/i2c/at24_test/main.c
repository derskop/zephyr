#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>

#define AT24_ADDR  0xA0        /* 7-bit 地址 */
#define AT24_MEM   0x00        /* 要写的内存地址 */

void main(void)
{
	const struct device *i2c = DEVICE_DT_GET(DT_NODELABEL(i2c1));
	uint8_t mem_addr = AT24_MEM;
	uint8_t wr_data  = 0x55;
	uint8_t rd_data  = 0x00;
	int ret;

    printf("hello world\r\n");

	if (!device_is_ready(i2c)) {
		printf("I2C1 not ready\n");
		return;
	}

	/* 配置成标准模式 100kHz */
	ret = i2c_configure(i2c, I2C_SPEED_SET(I2C_SPEED_STANDARD));
	if (ret < 0) {
		printf("i2c_configure failed: %d\n", ret);
		return;
	}

	/* 写：内存地址 0x00 + 数据 0x5A */
	uint8_t wr_buf[2] = { mem_addr, wr_data };
	ret = i2c_write(i2c, wr_buf, sizeof(wr_buf), AT24_ADDR);
	if (ret < 0) {
		printf("i2c_write failed: %d\n", ret);
		return;
	}

	/* 等 EEPROM 写完（内部写周期 ~5ms） */
	k_sleep(K_MSEC(10));

	/* 读：发内存地址 0x00 → 重复 START → 读 1 字节 */
	ret = i2c_write_read(i2c, AT24_ADDR, &mem_addr, 1, &rd_data, 1);
	if (ret < 0) {
		printf("i2c_read failed: %d\n", ret);
		return;
	}

	printf("write=0x%02x read=0x%02x  =>  %s\n",
	       wr_data, rd_data, (rd_data == wr_data) ? "PASS" : "FAIL");
           
}