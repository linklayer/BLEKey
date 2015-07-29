Firmware Update Instructions
============================

Application
-----------

This is probably what you want to update if you just want the latest BLEKey software. For FOTA update instructions see the video included in this folder.

Superhex
--------

The superhex file includes the application, bootloader and the softdevice. To flash it use the following command:

`JLinkExe -device nrf51822_xxaa -if swd -speed 4000 flash-superhex.jlink`
