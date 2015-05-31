BLEKey
======

Flashing
--------

###MacOS

```
$ /usr/bin/JLinkExe -device nrf51822_xxaa -if swd -speed 4000

J-Link> loadbin s110_nrf51822_7.0.0_softdevice.bin 0
J-Link> loadbin blekey_s110_xxaa.hex 0x16000
J-Link> r
J-Link> g
```

Erasing:
```
J-Link>w4 4001e504 2
J-Link>w4 4001e50c 1
```

Notes:

* Bluetooth Explorer is in the [Hardware IO Tools from Apple](http://adcdownload.apple.com/Developer_Tools/Hardware_IO_Tools_for_Xcode_6.3/HardwareIOTools_Xcode_6.3.dmg) it's probably the best BLE utility for Mac.
