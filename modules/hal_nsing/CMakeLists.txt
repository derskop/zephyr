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

  # N32G45X: chip selection macro required by n32g45x.h (#ifdef N32G45X)
  # N32G457_ETH: n32g45x_conf.h selects n32g457_eth.h based on this
  # USE_STDPERIPH_DRIVER: enables the standard peripheral library code
  zephyr_compile_definitions(N32G45X N32G457_ETH USE_STDPERIPH_DRIVER)

  # Both frequencies are declared in the devicetree only: the rcc node carries
  # SYSCLK, clk_hse carries the crystal. The vendor files leave a guard open for
  # this (n32g45x.h uses #if !defined HSE_VALUE, system_n32g45x.c uses #ifndef
  # SYSCLK_FREQ), so no vendor source is modified. Feeding both from one place
  # also arms the PLL check in system_n32g45x.c, which turns an unusable
  # (HSE, SYSCLK) pair into a compile error instead of a wrong bus speed.
  zephyr_compile_definitions(
    SYSCLK_FREQ=${CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC}
    HSE_VALUE=${CONFIG_N32_HSE_FREQ})

  # Header search paths. The vendor headers are third-party code, so treat them
  # as system headers: their u8/u16/u32 typedefs would otherwise trip -Wshadow
  # in Zephyr code using the same identifier (e.g. lib/net_buf/buf_simple.c).
  zephyr_system_include_directories(${nsing_soc_dir}/include)
  zephyr_system_include_directories(${nsing_std_dir}/include)

  # Where SystemInit() lives
  zephyr_library_sources(${nsing_soc_dir}/source/system_${CONFIG_SOC_SERIES}.c)

  # Standard peripheral library sources, built on demand via the USE_NSING_* Kconfigs
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
  zephyr_library_sources_ifdef(CONFIG_USE_NSING_ETH   ${nsing_std_src_dir}/n32g457_eth.c)
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
