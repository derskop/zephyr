.. zephyr:code-sample:: nsing-n32-qspi-flash
   :name: N32G45x QSPI flash
   :relevant-api: mspi_interface

   Drive the N32 QSPI controller through the MSPI API and check the external flash one layer at a time.

Overview
********

The sample talks to the P25Q40HA that is already on the board, but through the
QSPI controller instead of the SPI1 controller the board devicetree points at
it by default. It goes through the :ref:`MSPI API <mspi_api>` directly rather
than through the flash device layer, so that the controller is what is being
tested.

The part is a 4 Mbit (512 KiB) Puya device rather than the 128 Mbit one the
board is sold as: it answers command ``0x9f`` with ``85 60 13``, which is
Puya's manufacturer code, the SPI NOR memory type and the capacity code for
4 Mbit. That is the first thing the sample checks, because every step after it
assumes the geometry the board devicetree states for the flash.

It is written as a sequence of steps, each of which reports on one layer and
has to pass before the next one means anything:

.. list-table::
   :header-rows: 1

   * - Step
     - What it covers
   * - 1. JEDEC ID
     - Clock, chip select, the four pins and 8-8-8 framing. The answer is a
       constant -- ``85 60 13`` -- so it also settles byte ordering.
   * - 2. Protection and QE
     - Status register access and the write path, still over 8-8-8. Quad
       transfers need QE, and a part that arrives with the block protect bits
       set would refuse the erase below.
   * - 3. Erase and program
     - Erasing a sector and writing two 16 byte patterns into it, so that the
       reads have something to compare against that is not just ``ff``.
   * - 4. Two reads
     - The frame count test. The second read is what catches a controller that
       clocks one frame too many; see below.
   * - 5. Quad read
     - The same two regions read over four data lines and compared byte for
       byte against step 4.
   * - 6. Write paths
     - Three writes into one erased sector, each read back over one data line:
       a short run over four lines, a whole page over four, and a whole page
       over one, which is the write half. Only the data phase of the quad
       commands goes over four lines. The two page programs are then written
       again at 6 MHz, which separates a write that is wrong from a write whose
       feed could not keep up with the bus. The page is read after the erase and
       before the program, so that the comparison at the end cannot pass against
       what a previous run left in the sector.

The output names the layer that failed rather than reporting a single verdict,
because the failures that are possible here look nothing alike: a wrong pin
gives an ID of ``ff ff ff``, a wrong frame count gives a read that is a byte
short or a byte late, and a wrong transfer type gives bytes that are shifted
but otherwise plausible.

Reads longer than the FIFO
==========================

A read of a whole page is not one transfer. The controller clocks a frame out of
the flash for every frame pushed into its transmit FIFO, and it ends a transfer
whose FIFO has run dry, so a single transaction cannot carry a read longer than
the FIFO at a clock the CPU cannot match: the flash is left mid-read and the
caller gets the first part of the page followed by whatever was in the buffer,
with no count and no error flag to say so. The driver runs such a read as several
transactions instead, each re-sending the instruction and address for the next
stretch of it, which is what step 6's read-backs of a page depend on. Nothing on
the caller's side has to know -- a read of any length is one call. See
``N32_QSPI_RX_CHUNK_FRAMES`` in ``drivers/mspi/mspi_n32.c``.

Steps 3 and 6 erase the first two sectors of the flash. Anything stored there
is destroyed.

Requirements
************

The board carries the flash on PA4-PA7, which is also what ``&spi1`` uses, so
those four wires already exist. QSPI needs two more for quad transfers:

.. list-table::
   :header-rows: 1

   * - Signal
     - QSPI pin
     - P25Q40HA pin
   * - NSS / CS#
     - PA4
     - CS# (1)
   * - SCK / CLK
     - PA5
     - CLK (6)
   * - IO0
     - PA6
     - DI / IO0 (5)
   * - IO1
     - PA7
     - DO / IO1 (2)
   * - IO2
     - PC4
     - WP# / IO2 (3)
   * - IO3
     - PC5
     - HOLD# / IO3 (7)

PC4 and PC5 are unused by everything else on this board. They are WP# and HOLD#
on the flash, both active low, and the controller only drives them for the
duration of a quad transfer -- outside one it leaves them alone. If the board
does not pull them up, a floating HOLD# can leave the flash driving nothing,
which shows up as step 4 reading ``ff`` where step 3 wrote something.

The overlay disables ``&spi1`` and the flash node under it: the two controllers
share all four of PA4-PA7, so only one of them can be enabled, and the SPI flash
driver would otherwise build a device whose bus is switched off.

A page program is longer than the controller's FIFO, and the CPU cannot feed one
at these clocks, so the driver hands it to the controller's transmit DMA channel.
The overlay enables ``&dma2`` for that reason: without it a page write is refused
rather than sent short.

Building and Running
********************

.. zephyr-app-commands::
   :zephyr-app: samples/nsing/n32_qspi_flash
   :board: n32g45xml_stb
   :goals: build flash
   :compact:

The controller and its pin control live in the board devicetree, and the flash
node under it is what supplies the device index the sample passes to
``mspi_dev_config()``. It stays disabled, so that the flash layer does not probe
the same device during driver initialisation, ahead of ``main()``.

The flash layer route
*********************

Enabling the flash node and building with ``CONFIG_FLASH_MSPI_NOR=y`` puts the
upstream flash test in ``tests/drivers/flash/common`` on this board's external
flash instead, which is a much broader check than this sample. Two things have
to be settled first, and both are what step 5 and step 6 of this sample report
on:

* Which transfer type this flash accepts for a quad read. Step 5 answers it:
  ``MSPI_IO_MODE_QUAD_1_1_4``, the address on one line, which is what command
  ``0x6b`` asks for. ``MSPI_IO_MODE_QUAD_1_4_4`` answers nine bytes of ``cc``
  and then pattern A whichever of the two regions is read, so the address phase
  is not reaching the flash at all. The board's flash node states
  ``MSPI_IO_MODE_SINGLE`` and leaves the quad read to ``read-io-mode``, which is
  the property that means "for reads only" -- the erase, the write enable and
  the status polls around them have one form each and are single line commands.
* Whether the geometry should come from SFDP or from the devicetree. The flash
  node in the board file states it from the datasheet -- 512 KiB, 4 KiB sectors,
  256 byte pages -- which is what a configuration without
  ``CONFIG_FLASH_MSPI_NOR_USE_SFDP`` uses. The test sizes itself with
  ``CONFIG_TEST_DRIVER_FLASH_SIZE``, and the board conf for ``n32g45xml_stb``
  already sets that to 524288 for the internal flash: the same 512 KiB as this
  part, so it happens to be right here too.

Reading a failure
*****************

``ff ff ff`` as the JEDEC ID
  Nothing answered. The flash is not getting a clock, or chip select never
  asserted, or PA4-PA7 are still configured for SPI1 -- which is what happens
  if the overlay was not applied and ``&spi1`` is still enabled.

The ID is right but the patterns read back shifted by one byte
  A frame count off by one. The first read still returns the right bytes and
  the second one starts a frame late, which is exactly what step 4 prints: it
  compares the first byte of the second read against the last byte of the
  first one and says so when they are equal.

The first read is one byte short, the rest shifted
  The same field wrong in the other direction.

Step 4 passes and step 5 returns bytes that are wrong but not shifted
  The address is going out on the wrong number of lines. Step 5 tries both
  transfer types and prints which one worked; that name is what the flash node
  wants for ``read-io-mode``.

Every read is ``ff`` from step 4 on
  The erase and program did not take. Check that status register 2 reports QE
  in step 2, and that WP# is not being held low.

Step 4 and step 5 pass, but a page read back in step 6 agrees for its first
bytes and then stops
  A read longer than the controller's FIFO that was not split, which is what
  ``N32_QSPI_RX_CHUNK_FRAMES`` in ``drivers/mspi/mspi_n32.c`` is for. The bytes
  it did return are the flash's, and the rest is what the buffer held before
  the read: nothing is wrong with the flash, and the same page reads back
  correctly in the 16 byte packets the earlier steps use. The comparison prints
  the first byte that differs, which is where the transfer stopped.
