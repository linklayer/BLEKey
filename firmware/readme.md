Firmware Update Instructions
============================

Application
-----------

This is probably what you want to update if you just want the latest BLEKey software. For FOTA update instructions see the video [included in this folder.](fw_update.mp4)

Superhex
--------

The superhex file includes the application, bootloader and the softdevice. To flash it with a J-Link use the following command:

`JLinkExe -device nrf51822_xxaa -if swd -speed 4000 flash-superhex.jlink`

Building a Superhex
-------------------
 
 UNDER CONSTRUCTION
 
* J-Link Software
* Softdevice (in the `/nordic` directory)
* Bootloader hex (in the `/firmware` directory) 

References:
* https://devzone.nordicsemi.com/question/1462/nrf51822-how-to-merge-software-device-and-application-code-to-one-hex-file/
