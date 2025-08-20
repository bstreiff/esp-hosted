# BeagleBone Device Tree overlays

This directory contains sample device tree overlays for the Raspberry Pi.

The device tree overlay system allows you to configure extra hardware
attached to your Raspberry Pi, by copying the desired compiled .dtbo to
`/boot/firmware/overlays` and specifying the overlay with the `dtoverlay`
option in `/boot/firmware/config.txt`.

    dtoverlay=esp32-spi0.dtbo

Further reading:
- [Raspberry Pi Device Tree Overlay README](https://github.com/raspberrypi/firmware/blob/1.20250430/boot/overlays/README)
- [Raspberry Pi Pinout](https://pinout.xyz/pinout/spi)
