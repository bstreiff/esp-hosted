# BeagleBone Device Tree overlays

This directory contains sample device tree overlays for the
BeagleBone/BeagleBone Black.

The "cape" system for BeagleBone uses an I2C EEPROM present on the cape for
discovery, but this can be avoided (and an ESP32 connected directly) by
copying the desired compiled .dtbo to `/boot/dtbs/$(KERNELVER)/overlays` and
specifying the overlay with the `dtb_overlay` option in `/boot/uEnv.txt`.

    ###Custom Cape
    dtb_overlay=ESP32-SPI0-00A0.dtbo

Further reading:
- [Using Device Tree Overlays, example on BeagleBone Cape add-on boards](https://www.beagleboard.org/blog/2022-02-15-using-device-tree-overlays-example-on-beaglebone-cape-add-on-boards)
- [BeagleBone Cape Interface Specification](https://docs.beagleboard.org/boards/capes/cape-interface-spec.html)
