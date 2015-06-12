BLEKey
======

Pinout
------

Harware rev. 1 code name Acamas 

| Pin	| Function 		|
| ------|---------------|
| 0     | Data 0 in 	|
| 1		| Data 1 in		|
| ...	|				|
| 9		| Serial		|
| 11	| Serial		|

Flashing
--------

###MacOS

To flash the softdevice and BLEKey firmware from the CLI you can use JLinkExe which is provided in the [Segger Software](https://www.segger.com/jlink-software.html)

```
$ /usr/bin/JLinkExe -device nrf51822_xxaa -if swd -speed 4000

J-Link>loadbin s110_nrf51822_7.0.0_softdevice.bin 0
J-Link>loadbin blekey_s110_xxaa.hex 0x16000
J-Link>r
J-Link>g
```

Erasing:
```
J-Link>w4 4001e504 2
J-Link>w4 4001e50c 1
```

If you prefer a GUI you can always use [rknrfgo](http://sourceforge.net/projects/rknrfgo/)

The cheatsheet above is [ripped from Nordic's blog](https://devzone.nordicsemi.com/blogs/22/getting-started-with-nrf51-development-on-mac-os-x/)

###Windows

Use tools from Nordic. 

Using BLEKey
------------

Find your Bluetooth device with `hcitool dev` scan for BLEKey with `hcitool -i <dev> lescan`


Tell BLEKey to send Wiegand data:
```
sudo gatttool -t random -b D4:34:E8:CA:6F:6A --char-write-req -a 0x000d -n 01
```



Notes:
------

* Bluetooth Explorer is in the [Hardware IO Tools from Apple](http://adcdownload.apple.com/Developer_Tools/Hardware_IO_Tools_for_Xcode_6.3/HardwareIOTools_Xcode_6.3.dmg) it's probably the best BLE utility for Mac.


