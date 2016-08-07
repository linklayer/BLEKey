BLEKey
======

by Mark Baseggio and Eric Evenchick

BLEKey is a tool designed to demonstrate the risks of using the Wiegand protocol. 

[Here's a video](https://www.youtube.com/watch?v=seKas8KFcSI) of us talking about the BLEKey at Blackhat USA 2015.

You can get BLEKey here: http://store.cantact.io/products/blekey-1?variant=18261748871 **ANY OTHER STORE/SITE IS UNOFFICIAL** 

Building
--------

1. Download the GNU ARM Embedded Toolchain for your OS (gcc-arm-none-eabi-4_9-2015q1 was used to build this version): https://launchpad.net/gcc-arm-embedded/+download
2. Clone the repo: `git clone https://github.com/linklayer/BLEKey.git`
3. Edit the `Makefile.posix` file in the `BLEKey/gcc/` directory and make sure the path to the downloaded ARM toolchain is correct.
4. Change to the `BLEKey/gcc` directory and run `make`

If you have a Segger hooked up to the board you can also run:

* `make erase` to erase the nrf 
* `make flash-sd` to flash the soft device
* `make flash` to flash the compiled hex 

Flashing
--------

###Firmware Over the Air (FOTA)
We've included a file in the `/firmware` folder called `super.hex`. This firmware has a bootloader included that allows the BLEKey to be flashed from a cell phone by shorting the first and last pins of the P2 port, and inserting a battery. The firmware update has been tested on the Android platform using the Nordic Master Control Panel tool.

[We've created a video to demonstrate the process.](/firmware/fw_update.mp4)

###MacOS or Linux

To flash the softdevice and BLEKey firmware from the CLI you can use JLinkExe which is provided in the [Segger Software](https://www.segger.com/jlink-software.html)

The BLEKey requires a softdevice (for BLE) and the blekey firmware. These can be flashed using the commands below:

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

###TODO ADD CHARACTERISTIC INFO HERE.

###Client

There is a BLEKey client in the client/ directory of the git repo. See readme.md and requirements.txt for more information on its use.

###Phones/Tablets

A client is planned. Until then download any BLE utility that can read/write BLE characteristics. See below for more information on specific characteristics.

###CLI (Linux)

Find your Bluetooth device with `hcitool dev` scan for BLEKey with `hcitool -i <dev> lescan`

Start an interactive connection to BLEKey

Last three Wiegand cards are stored in the `0x000b` handle. Currently to cause BLEKey to send out the last read card on the Wiegand lines write to `0x000d`

```
[blark@archvm blekey]$ sudo gatttool -t random -b D4:34:E8:CA:6F:6A -I
[D4:34:E8:CA:6F:6A][LE]> connect
Attempting to connect to D4:34:E8:CA:6F:6A
Connection successful
[D4:34:E8:CA:6F:6A][LE]> characteristics
handle: 0x0002, char properties: 0x0a, char value handle: 0x0003, uuid: 00002a00-0000-1000-8000-00805f9b34fb
handle: 0x0004, char properties: 0x02, char value handle: 0x0005, uuid: 00002a01-0000-1000-8000-00805f9b34fb
handle: 0x0006, char properties: 0x02, char value handle: 0x0007, uuid: 00002a04-0000-1000-8000-00805f9b34fb
handle: 0x000a, char properties: 0x02, char value handle: 0x000b, uuid: 0000aaaa-0000-1000-8000-00805f9b34fb
handle: 0x000c, char properties: 0x08, char value handle: 0x000d, uuid: 0000bbbb-0000-1000-8000-00805f9b34fb
handle: 0x000e, char properties: 0x08, char value handle: 0x000f, uuid: 0000cccc-0000-1000-8000-00805f9b34fb
handle: 0x0010, char properties: 0x0a, char value handle: 0x0011, uuid: 0000dddd-0000-1000-8000-00805f9b34fb
handle: 0x0013, char properties: 0x12, char value handle: 0x0014, uuid: 00002a19-0000-1000-8000-00805f9b34fb
handle: 0x0017, char properties: 0x02, char value handle: 0x0018, uuid: 00002a29-0000-1000-8000-00805f9b34fb
[D4:34:E8:CA:6F:6A][LE]> char-write-req d 01
Characteristic value was written successfully
[D4:34:E8:CA:6F:6A][LE]> char-read-hnd b
Characteristic value/descriptor: 22 bd 58 60 7c 26 00 24 47 60 85 08 37 00 24 47 60 85 08 37 00 00 
[D4:34:E8:CA:6F:6A][LE]>
```

You can also just use gatttool from the command line to yell BLEKey to send Wiegand data (replace with the address of your device):
```
sudo gatttool -t random -b D4:34:E8:CA:6F:6A --char-write-req -a 0x000d -n 01
```

###Notes:

* Bluetooth Explorer is in the [Hardware IO Tools from Apple](http://adcdownload.apple.com/Developer_Tools/Hardware_IO_Tools_for_Xcode_6.3/HardwareIOTools_Xcode_6.3.dmg) it's probably the best BLE utility for Mac.

Pinouts
-------

###Harware rev. 0 code name Acamas

| Pin	| Function 		|
| ------|---------------|
| 0     | Data 0 in 	|
| 1		| Data 1 in		|
| 2     | Data 0 ctl	|
| 3		| Data 1 ctl	|
| 9		| Serial		|
| 11	| Serial		|
| 26	| Xtal 1		|
| 27	| Xtal 2		|
| 28	| GPIO			|
| 29	| GPIO			|
| 35	| DIO			|
| 36	| CLK			|

###Hardware rev. 1 Agapenor

Pinouts are identical to Acamas unless otherwise noted.

| Pin	| Function 		|
| ------|---------------|
| 0     | Data 0 in 	|
| 7		| Data 1 in		|
| 8		| GPIO			|
| 20	| LED0			|

####P1 - Programming port (square solder mask pin 0)

| Pin	| Function 		|
| ------|---------------|
| 0     | Vcc		 	|
| 1		| DIO			|
| 2		| CLK			|
| 3     | Gnd			|

####P2 - Serial/GPIO port (square solder mask pin 0)

| Pin	| Function 		|
| ------|---------------|
| 0     | GPIO (Tx)	 	|
| 1		| DATA0			|
| 2		| DATA1			|
| 3     | Gnd			|
