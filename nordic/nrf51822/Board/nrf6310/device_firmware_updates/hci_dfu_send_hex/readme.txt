'hci_dfu_send_hex.py' python script that can perform device firmware upgrade over HCI-UART interface.
This application accepts firmware to be upgraded in hex format as a command line argument along with
the COM Port to be used for the transport.

Please note that you need the following third-party python libraries:
 - InteHex (https://pypi.python.org/pypi/IntelHex/1.1)
 - PySerial (http://pyserial.sourceforge.net)


The application can take following command line arguments:

  --help, -h                       Show help message and exits.
  --file FILE, -f FILE             Filename of Hex file. This is mandatory to provide.
  --port COMPORT, -p COMPORT       COM Port to which the device is connected. This is mandatory to
                                   provide.
  --flowcontrol, -fc               Enable flow control, default: disabled.
                                   Note1: Flow control must be enabled when using the SDK bootloader
                                          example.
  --baudrate BAUDRATE, -b BAUDRATE Desired baudrate 
                                   38400/96000/115200/230400/250000/460800/921600/1000000 (default: 38400). 
                                   Note1: Baud rates >115200 are supported by nRF51822,
                                          but may not be supported by all RS232 devices on windows.
                                   Note2: The SDK bootloader example uses the default baud rate.

The following is an example of how to use the script:
 
   python hci_dfu_send_hex.py -f C:\NewFirmware\ble_app_hrs.hex -p COM7 -fc

The test_images_update folder contains dummy images that can be used to test the script.

Note 1: HCI features like retransmission have not been implement in this application.

The application provides comprehensive messages to provide the progress of the firmware 
upgrade or the reason for failure in case the procedure did not succeed. However, the 
application is experimental and is has only been tested on Windows 7 systems.

