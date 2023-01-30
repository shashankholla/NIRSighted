The iCE40's flash configuration memory can be programmed by an ftdi ft232h usb-to-spi chip.

You can use an [Adafruit ft232h breakout](https://www.adafruit.com/product/2264) board for this. Make sure you consult the groundtruth cam vision board schematic to make sure that the 10-pin header is wired up properly.

Once you have a ft232h "programming cable" put togheter, deploy this test program to a board by running:

```
    make
    sudo iceprog blinky.bin
```

if you really don't want to use sudo, [set up your udev rules correctly](https://github.com/YosysHQ/icestorm/issues/236) to allow non-sudo users to write to the ft232h chip.
