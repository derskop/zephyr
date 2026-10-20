N32G45x ADC and DAC
###################

Build and flash from the Zephyr repository root::

   west build -p always -b n32g45xml_stb/n32g457 samples/nsing/n32_adc_dac -d build/n32_adc_dac
   west flash -d build/n32_adc_dac --runner jlink --speed 100

Read the CH340 serial console at 115200 8N1. Replace ``/dev/ttyUSB0`` with
the serial device assigned on your host::

   python3 -m serial.tools.miniterm /dev/ttyUSB0 115200 --raw

The sample sweeps DAC channel 1 on PA4 through 0, 1024, 2048, 3072, and
4095, reading ADC1 channel 1 on PA0 after each step. Connect J1 pin 18 (PA4)
to J1 pin 14 (PA0) with a jumper; leave S1 and S6 released. Do not connect
either pin to an external voltage source while the jumper is present. The
ADC code should rise with the DAC code. The middle steps should be near
1024, 2048 and 3072; the exact values depend on analog tolerances and PA0's
on-board 10 kOhm pull-down. The printed ``delta`` is ADC code minus DAC code.

The board overlay disables SPI1 because its chip select uses PA4. DAC channel
2 uses PA5, which also conflicts with SPI1 SCK. ADC and DAC are not enabled in
the board's default devicetree.

The driver uses polling for ADC reads. It supports 6/8/10/12-bit single-ended
conversions and 8/12-bit DAC writes; ADC DMA, injected conversions, hardware
oversampling, and asynchronous reads are not implemented.

Replacement board result (2026-10-20)
*************************************

With PA4 connected to PA0, the firmware built and flashed through J-Link.
The CH340 console reported this full sweep on the replacement board:

======== ======== =========
DAC code ADC code ADC - DAC
======== ======== =========
0        52       +52
1024     1024     0
2048     2043     -5
3072     3064     -8
4095     4042     -53
======== ======== =========

The three middle points agreed within eight counts. The larger endpoint
offsets are consistent with the buffered DAC output approaching the supply
rails and the PA0 input's on-board pull-down. The test confirms that DAC1
output on PA4 reaches ADC1 channel 1 on PA0 and the ADC follows its changes.
