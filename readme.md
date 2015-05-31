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

