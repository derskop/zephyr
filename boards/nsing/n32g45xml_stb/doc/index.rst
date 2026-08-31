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
| WATCHDOG  | on-chip    | independent watchdog                |
+-----------+------------+-------------------------------------+
| I2C       | on-chip    | i2c                                 |
+-----------+------------+-------------------------------------+
| ADC       | on-chip    | ADC Controller                      |
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
