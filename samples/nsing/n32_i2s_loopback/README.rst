.. zephyr:code-sample:: nsing-n32-i2s-loopback
   :name: N32G45x I2S loopback
   :relevant-api: i2s_interface

   Send a sine wave out of one N32 I2S instance and check what another one receives.

Overview
********

The sample cabled two I2S instances of the same SoC to each other: ``i2s3``
drives the frame and bit clocks as controller and transmits a sine wave, while
``i2s2`` follows them as target and receives it. Each round the block that came
back is compared sample by sample against the block that went out, and the
result is printed.

It is meant as a quick end-to-end check of the :dtcompatible:`nsing,n32-i2s`
driver that reports in plain text, where :zephyr:code-sample:`the upstream
i2s_api suite <i2s-api>` reports through ztest and needs a debugger to read.

Requirements
************

Three jumper wires between the two I2S instances:

.. list-table::
   :header-rows: 1

   * - Signal
     - Controller (``i2s3``)
     - Target (``i2s2``)
   * - CK
     - PB3
     - PB13
   * - WS
     - PA15
     - PB12
   * - SD
     - PB5
     - PB15

``PB5`` is also the red LED on this board, and ``PB12``/``PB13`` are also
``can2``; the overlay releases all of them, so the CAN and SPI samples can
still use their pins.

Building and Running
********************

.. zephyr-app-commands::
   :zephyr-app: samples/nsing/n32_i2s_loopback
   :board: n32g45xml_stb
   :goals: build flash
   :compact:

The two I2S nodes, their pin control and the ``i2s-tx``/``i2s-rx`` aliases live
in the board devicetree, so the overlay only has to turn them on and settle the
pin conflicts -- which is the part that really is specific to this application.
That keeps the CAN and SPI samples working: they need the same pins, and the
board file leaves the I2S nodes disabled for them.

Sample Output
=============

.. code-block:: console

   === N32G45x I2S loopback ===
   TX (controller) : i2s@40003c00
   RX (target)     : i2s@40003800
   16-bit stereo, I2S format, 8000 Hz, 128-byte blocks, 10 rounds

   round  0: OK   32 samples, offset 0, first pair   6392  32137
   round  1: OK   32 samples, offset 0, first pair   6392  32137
   round  2: OK   32 samples, offset 0, first pair   6392  32137
   round  3: OK   32 samples, offset 0, first pair   6392  32137
   round  4: OK   32 samples, offset 0, first pair   6392  32137
   round  5: OK   32 samples, offset 0, first pair   6392  32137
   round  6: OK   32 samples, offset 0, first pair   6392  32137
   round  7: OK   32 samples, offset 0, first pair   6392  32137
   round  8: OK   32 samples, offset 0, first pair   6392  32137
   round  9: OK   32 samples, offset 0, first pair   6392  32137

   10/10 rounds verified -> PASS

   --- RX overrun recovery ---
   slab free after one forced overrun: 1/1 -> PASS

The device names come from the devicetree node names, which is the unit
address here rather than the label: ``i2s@40003c00`` is ``i2s3`` and
``i2s@40003800`` is ``i2s2``.

The last section is a separate check. It reconfigures the receiver onto a
one-block slab, which cannot possibly keep up, so the completion after the
first block finds nothing to re-arm with and the stream goes to
``I2S_STATE_ERROR`` -- the overrun the API asks to be reported. What it
measures is what is left in the slab afterwards: a block that is neither
handed to the application nor returned to the slab is gone for good, and an
application that hits one overrun could then never restart, losing a block
every time. Upstream's own suite cannot see this: with four blocks, losing one
still leaves enough for every test to pass.

``offset`` is how far into the transmitted stream the received block started.
A steady 0 means the target picked the frame up on its first pair; upstream's
own suite tolerates up to two.

All ten lines appear together because the sample prints only once the transfer
is over. That is deliberate, and the reason is worth carrying into application
code:

.. warning::

   Do not print between ``i2s_buf_write()`` and the ``i2s_buf_read()`` that
   consumes the matching block. At 8 kHz a ``BLOCK_SIZE`` block lasts about
   4 ms, while one line of console output on this board's USART1 at 115200 baud
   blocks for about 5 ms -- longer than the block it is racing. The TX queue
   drains while the CPU waits on the UART, the completion callback finds
   nothing to re-arm with, and the stream goes to ``I2S_STATE_ERROR``. The
   next write then fails with ``-EIO`` on a round that did nothing wrong::

      round  0: OK   32 samples, offset 0, first pair   6392  32137
      round  1: TX write failed (-5)          <-- -EIO, stream underran
      1/10 rounds verified -> FAIL

   Buffering the results and printing them afterwards, as this sample does,
   costs nothing. The driver has the same property for its own logging: it
   deliberately says nothing above ``LOG_DBG`` while a stream is running.
