# Nationstech N32 firmware library
#
# Copyright (c) 2024 Nations Technologies
# SPDX-License-Identifier: Apache-2.0
#

if(CONFIG_HAS_NSING_HAL)

set(nsing_soc_dir     ${ZEPHYR_HAL_NSING_MODULE_DIR}/${CONFIG_SOC_SERIES})
set(nsing_std_dir     ${nsing_soc_dir}/standard_peripheral)
set(nsing_std_src_dir ${nsing_std_dir}/source)

zephyr_library_named(hal_nsing)

# N32G45X: n32g45x.h 要求的芯片选择宏（#ifdef N32G45X）
# USE_STDPERIPH_DRIVER: 使能标准外设库代码
zephyr_compile_definitions(N32G45X USE_STDPERIPH_DRIVER)

# 头文件搜索路径
zephyr_include_directories(${nsing_soc_dir}/include)
zephyr_include_directories(${nsing_std_dir}/include)

# SystemInit() 所在
zephyr_library_sources(${nsing_soc_dir}/source/system_${CONFIG_SOC_SERIES}.c)

# 标准外设库源文件，按 Kconfig 的 USE_NSING_* 按需编译
zephyr_library_sources_ifdef(CONFIG_USE_NSING_MISC  ${nsing_std_src_dir}/misc.c)
zephyr_library_sources_ifdef(CONFIG_USE_NSING_ADC   ${nsing_std_src_dir}/${CONFIG_SOC_SERIES}_adc.c)
zephyr_library_sources_ifdef(CONFIG_USE_NSING_BKP   ${nsing_std_src_dir}/${CONFIG_SOC_SERIES}_bkp.c)
zephyr_library_sources_ifdef(CONFIG_USE_NSING_CAN   ${nsing_std_src_dir}/${CONFIG_SOC_SERIES}_can.c)
zephyr_library_sources_ifdef(CONFIG_USE_NSING_COMP  ${nsing_std_src_dir}/${CONFIG_SOC_SERIES}_comp.c)
zephyr_library_sources_ifdef(CONFIG_USE_NSING_CRC   ${nsing_std_src_dir}/${CONFIG_SOC_SERIES}_crc.c)
zephyr_library_sources_ifdef(CONFIG_USE_NSING_DAC   ${nsing_std_src_dir}/${CONFIG_SOC_SERIES}_dac.c)
zephyr_library_sources_ifdef(CONFIG_USE_NSING_DBG   ${nsing_std_src_dir}/${CONFIG_SOC_SERIES}_dbg.c)
zephyr_library_sources_ifdef(CONFIG_USE_NSING_DMA   ${nsing_std_src_dir}/${CONFIG_SOC_SERIES}_dma.c)
zephyr_library_sources_ifdef(CONFIG_USE_NSING_DVP   ${nsing_std_src_dir}/${CONFIG_SOC_SERIES}_dvp.c)
zephyr_library_sources_ifdef(CONFIG_USE_NSING_ETH   ${nsing_std_src_dir}/${CONFIG_SOC_SERIES}_eth.c)
zephyr_library_sources_ifdef(CONFIG_USE_NSING_EXTI  ${nsing_std_src_dir}/${CONFIG_SOC_SERIES}_exti.c)
zephyr_library_sources_ifdef(CONFIG_USE_NSING_FLASH ${nsing_std_src_dir}/${CONFIG_SOC_SERIES}_flash.c)
zephyr_library_sources_ifdef(CONFIG_USE_NSING_GPIO  ${nsing_std_src_dir}/${CONFIG_SOC_SERIES}_gpio.c)
zephyr_library_sources_ifdef(CONFIG_USE_NSING_I2C   ${nsing_std_src_dir}/${CONFIG_SOC_SERIES}_i2c.c)
zephyr_library_sources_ifdef(CONFIG_USE_NSING_IWDG  ${nsing_std_src_dir}/${CONFIG_SOC_SERIES}_iwdg.c)
zephyr_library_sources_ifdef(CONFIG_USE_NSING_OPAMP ${nsing_std_src_dir}/${CONFIG_SOC_SERIES}_opamp.c)
zephyr_library_sources_ifdef(CONFIG_USE_NSING_PWR   ${nsing_std_src_dir}/${CONFIG_SOC_SERIES}_pwr.c)
zephyr_library_sources_ifdef(CONFIG_USE_NSING_QSPI  ${nsing_std_src_dir}/${CONFIG_SOC_SERIES}_qspi.c)
zephyr_library_sources_ifdef(CONFIG_USE_NSING_RCC   ${nsing_std_src_dir}/${CONFIG_SOC_SERIES}_rcc.c)
zephyr_library_sources_ifdef(CONFIG_USE_NSING_RTC   ${nsing_std_src_dir}/${CONFIG_SOC_SERIES}_rtc.c)
zephyr_library_sources_ifdef(CONFIG_USE_NSING_SDIO  ${nsing_std_src_dir}/${CONFIG_SOC_SERIES}_sdio.c)
zephyr_library_sources_ifdef(CONFIG_USE_NSING_SPI   ${nsing_std_src_dir}/${CONFIG_SOC_SERIES}_spi.c)
zephyr_library_sources_ifdef(CONFIG_USE_NSING_TIM   ${nsing_std_src_dir}/${CONFIG_SOC_SERIES}_tim.c)
zephyr_library_sources_ifdef(CONFIG_USE_NSING_TSC   ${nsing_std_src_dir}/${CONFIG_SOC_SERIES}_tsc.c)
zephyr_library_sources_ifdef(CONFIG_USE_NSING_USART ${nsing_std_src_dir}/${CONFIG_SOC_SERIES}_usart.c)
zephyr_library_sources_ifdef(CONFIG_USE_NSING_WWDGT ${nsing_std_src_dir}/${CONFIG_SOC_SERIES}_wwdg.c)

endif()
