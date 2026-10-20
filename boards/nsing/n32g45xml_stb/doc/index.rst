.. _n32g45xml_stb:

NATIONS N32G45XML-STB
#####################

Overview
********

The N32G45XML-STB board is a hardware platform that enables design and debug
of the Nations N32G457 Cortex-M4F high performance MCU.

Hardware
********
- 2 user LEDs
- 2 user push buttons
- Reset Button
- USB VBUS or external source (3.3V, 5V)
- vbat
- USB to serial port
- NS-Link interface

For more information about the N32G45x SoC and N32G45XML-STB board:

- `Nationstech Cortex-M4 High Performance SoC Website`_
- `N32G457 Datasheet`_
- `N32G457 Reference Manual`_

Supported Features
==================

The Zephyr n32g45xml_stb board configuration supports the following hardware features:

+-----------+------------+-------------------------------------+
| Interface | Controller | Driver/Component                    |
+===========+============+=====================================+
| NVIC      | on-chip    | nested vector interrupt controller  |
+-----------+------------+-------------------------------------+
| UART      | on-chip    | serial port-polling;                |
|           |            | serial port-interrupt               |
+-----------+------------+-------------------------------------+
| PINCTRL   | on-chip    | pin control                         |
+-----------+------------+-------------------------------------+
| GPIO      | on-chip    | gpio                                |
+-----------+------------+-------------------------------------+
| CLOCK     | on-chip    | reset and clock control             |
+-----------+------------+-------------------------------------+
| FLASH     | on-chip    | flash memory                        |
+-----------+------------+-------------------------------------+
| HWINFO    | on-chip    | 96-bit unique device identifier     |
+-----------+------------+-------------------------------------+
| WATCHDOG  | on-chip    | independent watchdog                |
+-----------+------------+-------------------------------------+
| I2C       | on-chip    | i2c                                 |
+-----------+------------+-------------------------------------+
| ADC       | on-chip    | ADC Controller                      |
+-----------+------------+-------------------------------------+
| DAC       | on-chip    | DAC Controller                      |
+-----------+------------+-------------------------------------+
| PWM       | on-chip    | pwm                                 |
+-----------+------------+-------------------------------------+
| SPI       | on-chip    | spi                                 |
+-----------+------------+-------------------------------------+
| USB       | on-chip    | USB device                          |
+-----------+------------+-------------------------------------+
| COUNTER   | on-chip    | rtc                                 |
+-----------+------------+-------------------------------------+
| RTC       | on-chip    | rtc                                 |
+-----------+------------+-------------------------------------+

The default configuration can be found in the defconfig file:
``boards/nsing/n32g45xml_stb/n32g45xml_stb_defconfig``

Default Zephyr Peripheral Mapping
----------------------------------

- USART_1 TX/RX: PA9/PA10 (NS-Link Virtual COM Port)
- I2C1 SDA/SCL: PB7/PB6
- LED1: PA8
- LED2: PB4
- LED3: PB5
- USB_DC DM/DP: PA11/PA12
- PWM TIM3 CH3/CH4: PB0/PB1 (TIM3 CH1/CH2 are used by SPI1)

ADC and DAC Pin Selection
-------------------------

The N32G457MEL in this board uses the LQFP80 package. The board pinctrl file
declares these external ADC channel/pin pairs available in that package:

- ADC1: 1/PA0, 2/PA1, 3/PA6, 4/PA3, 6/PC0, 7/PC1, 8/PC2, 9/PC3, 11/PA2.
- ADC2: 1/PA4, 2/PA5, 3/PB1, 4/PA7, 5/PC4, 6/PC0, 7/PC1, 8/PC2, 9/PC3,
  11/PA2, 12/PC5, 13/PB2.
- ADC3: 1/PB11, 2/PE9, 3/PE13, 4/PE12, 5/PB13, 6/PE8, 7/PD10, 11/PD14,
  12/PB0, 13/PE7, 14/PE10, 15/PE11.
- ADC4: 3/PB12, 4/PB14, 5/PB15, 6/PE8, 7/PD10, 11/PD14, 12/PD8, 13/PD9.

These are pinctrl choices, not pins reserved by the ADC driver. An application
enables the required ADC instance and references only its chosen pinctrl nodes.
For example, ``&adc1_in7_pc1`` selects ADC1 channel 7 on PC1. DAC channel 1
uses PA4 and channel 2 uses PA5. Both DAC pins overlap the board's SPI1 and
QSPI wiring, so an application using them must release the conflicting devices.
Other ADC pins also overlap digital peripherals; check the selected pin's
board connections before enabling it. See ``samples/nsing/n32_adc_dac`` for
an ADC1 PA0 and DAC1 PA4 loopback configuration.

System Clock
------------

The on-board 8MHz crystal is used to produce a 144MHz system clock with PLL.

Flashing
========

Follow the :ref:`getting_started` instructions for Zephyr application
development.

Debugging
=========

References
**********

.. _Nationstech Cortex-M4 High Performance SoC Website:
	https://www.nationstech.com/N32G457/

.. _N32G457 Datasheet:
    https://www.nationstech.com/down4/

.. _N32G457 Reference Manual:
    https://www.nationstech.com/down4/
