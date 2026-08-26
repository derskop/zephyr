.. _n32g45xv_stb:

NATIONS-N32G45XV-STB
########################

Overview
********

The N32G45XV-STB board is a hardware platform that enables design and debug
of the Cortex-M4F High Performance MCU.There are multiple version of this board .

.. image:: img/n32g45xv_stb.jpg
     :align: center
     :alt: n32g45xv_stb



Hardware
********
- 2 user LEDs
- 2 user push buttons
- Reset Button
- USB VBUS or external source (3.3V, 5V)
- vbat
- usb to serial port
- NS-Link interface


For more information about the N32G45X SoC and N32G45X-STB board:

- `Nationstech Cortex-M4 High Performance SoC Website`_
- `N32G457 Datasheet`_
- `N32G457 Reference Manual`_





Supported Features
==================

The Zephyr n32g45xv_stb board configuration supports the following hardware features:

+-----------+------------+-------------------------------------+
| Interface | Controller | Driver/Component                    |
+===========+============+=====================================+
| NVIC      | on-chip    | nested vector interrupt controller  |
+-----------+------------+-------------------------------------+
| UART      | on-chip    | serial port-polling;                |
|           |            | serial port-interrupt               |
+-----------+------------+-------------------------------------+
| PINMUX    | on-chip    | pinmux                              |
+-----------+------------+-------------------------------------+
| GPIO      | on-chip    | gpio                                |
+-----------+------------+-------------------------------------+
| CLOCK     | on-chip    | reset and clock control             |
+-----------+------------+-------------------------------------+
| FLASH     | on-chip    | flash memory                        |
+-----------+------------+-------------------------------------+
| WATCHDOG  | on-chip    | independent watchdog                |
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
``boards/arm/nations_n32g45xv_stb/nations_n32g45xv_stb_defconfig``




Default Zephyr Peripheral Mapping:
----------------------------------

- USART_1 TX/RX: PA9/PA10 (NS-Link Virtual COM Port)

- SPI1 NSS/SCK/MISO/MOSI: PA4/PA5/PA6/PA7
- SPI2 NSS/SCK/MISO/MOSI: PB12/PB13/PB14/PB15
- I2C1 SDA/SCL: PB9/PB8
- USER_PB: PC13
- LED1: PA8
- LED2: PB4
- LED3: PB5
- KEY1: PA4
- KEY2: PA5
- KEY3: PA6
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



