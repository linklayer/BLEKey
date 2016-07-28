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
** UNDER CONSTRUCTION - NOT TESTED YET **
 
* Softdevice (in the `/nordic` directory)
* bootloader.hex, app_valid_setting_apply.hex, and mergehex.exe (in the `/firmware` directory) 
* Compiled firmware (buid it)
* Alternative to mergehex.exe you can use the J-Link Software (SEGGER J-Flash) GUI to merge files. Read the first reference below to see how to do this. 

Commands: 
```
mergehex --merge s110_nrf51822_7.1.0_softdevice.hex bootloader.hex --output SD_BL.hex
mergehex --merge SD_BL.hex blekey_s110_xxaa.hex --output SD_BL_APP.hex
mergehex --merge SD_BL_APP.hex app_valid_setting_apply.hex --output super.hex
```

References:
* https://devzone.nordicsemi.com/question/22056/combining-sd-dfu-and-application-hex-and-programming/
* https://devzone.nordicsemi.com/question/1462/nrf51822-how-to-merge-software-device-and-application-code-to-one-hex-file/
