BLEKey
======

Pinout
------

| Pin	| What 	  		|
| ------|---------------|
| 0     | Data 0  		|
| 1		| Data 1		|

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

###Windows

Use tools from Nordic. 

Notes:
------

* Bluetooth Explorer is in the [Hardware IO Tools from Apple](http://adcdownload.apple.com/Developer_Tools/Hardware_IO_Tools_for_Xcode_6.3/HardwareIOTools_Xcode_6.3.dmg) it's probably the best BLE utility for Mac.
