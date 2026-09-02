.. _n32_bxcan_verify:

N32 bxCAN driver verification
#############################

Overview
********

Driver verification application for the Zephyr N32 bxCAN driver
(``drivers/can/can_n32_bxcan.c``). It drives both CAN instances of the
N32G457ME on the ``n32g45xml_stb`` board:

* **CAN1** on PA11 (RX) / PA12 (TX)
* **CAN2** on PB12 (RX) / PB13 (TX)

Two test phases are provided:

* **Internal loopback (default, no external hardware)** — both CAN1 and CAN2
  run independently in ``CAN_MODE_LOOPBACK``. Each channel transmits a
  standard-ID frame (0x111) and an extended-ID frame (0x1234567) and verifies
  the frame echoed back through its own RX filters. Exercises the driver's
  ``set_mode``/``start``/``send``/``add_rx_filter``/ISR paths on both channels.
* **Cross-bus (``CONFIG_CAN_SAMPLE_CROSS_BUS=y``)** — both channels run in
  normal mode on a shared physical bus. CAN1 sends 0x100 to CAN2, CAN2 sends
  0x200 to CAN1.

Each test prints ``[PASS]``/``[FAIL]`` per channel and a final summary.
Success looks like::

    N32 bxCAN driver verification
    can1: max filters: std=14 ext=7
    can1: [PASS] loopback standard frame
    can1: [PASS] loopback extended frame
    can2: max filters: std=14 ext=7
    can2: [PASS] loopback standard frame
    can2: [PASS] loopback extended frame
    can1: 2 pass, 0 fail
    can2: 2 pass, 0 fail
    *** ALL TESTS PASSED ***

Requirements
************

* West workspace with the N32 HAL module, Zephyr SDK, and a J-Link debugger.
* For the cross-bus phase: two CAN transceivers (e.g. TJA1050/SN65HVD230)
  and 120 ohm bus termination.

Building and Flashing
*********************

Internal loopback test (no external hardware)::

    west build -b n32g45xml_stb samples/drivers/can/n32_bxcan_verify -d build_verify

Cross-bus test (external transceivers required)::

    west build -b n32g45xml_stb samples/drivers/can/n32_bxcan_verify \
      -d build_verify_cross -DCONFIG_CAN_SAMPLE_CROSS_BUS=y

Flash with J-Link (N32G457ME over SWD)::

    printf 'r\nloadfile build_verify/zephyr/zephyr.hex\nr\ng\nq\n' | \
      JLinkExe -device N32G457ME -if SWD -speed 1000 -AutoConnect 1 -ExitOnError 1

Observe the output on the USART1 console (115200 baud).

Hardware Wiring
***************

CAN transceiver connections (both channels, loopback mode needs no wiring):

.. list-table::
   :header-rows: 1

   * - Signal
     - MCU pin
     - Transceiver
   * - CAN1_RX
     - PA11
     - RXD
   * - CAN1_TX
     - PA12
     - TXD
   * - CAN2_RX
     - PB12
     - RXD
   * - CAN2_TX
     - PB13
     - TXD

For the cross-bus test, wire the two transceivers onto one bus::

    CAN1_TXD <-> TJA1050#1 TXD, CAN1_RXD <-> TJA1050#1 RXD
    CAN2_TXD <-> TJA1050#2 TXD, CAN2_RXD <-> TJA1050#2 RXD
    TJA1050#1 CANH --- CANH TJA1050#2   (120 ohm to CANL at both ends)
    TJA1050#1 CANL --- CANL TJA1050#2   (120 ohm to CANH at both ends)

Notes
*****

* CAN2 default pins (PB12/PB13) are the STM32F105-style defaults; verify them
  against the board schematic. CAN1 can be remapped to PB8/9, PD0/1 or
  PD12/13 via the N32 remap feature if needed.
* The loopback test exercises TX, RX filters, and the driver ISRs entirely
  on-chip, so it is a good first bring-up test before any wiring is done.
