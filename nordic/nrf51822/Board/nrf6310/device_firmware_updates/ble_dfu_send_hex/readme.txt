Testing the DFU:

Pre-requisite:
- Install Nordic nRF51 SDK
- Install IronPython which can be found at http://ironpython.net
- You need a Master Control Panel dongle connected.

Program initial SoftDevice
1.  Make sure the device is cleared (nrfjprog --eraseall)
2.  Program s110_nrf51822_7.0.0_softdevice.hex (nrfjprog --program s110_nrf51822_7.0.0_softdevice.hex)
3.  Confirm the data at location 0x00003000: FFFFFF10 51B1E5DB 00016000 FFFF004F
    (nrfjprog --memrd 0x3000 --w 4 --n 16)
    Note1: Other versions of the SoftDevice will have other values for the last two bytes (e.g. 004E).

Program Bootloader with SoftDevice update support
4.  Go to <Install Folder>\Nordic\nrf51822\Board\nrf6310\device_firmware_updates\bootloader\arm\
5.  Open bootloader.uvproj, compile and download.

Perform DFU
6.  Using the script enclosed in this folder it is possible to transfer a new SoftDevice,
    Bootloader, or Application.
    a. Open Master Control Panel, find the device advertising as: 'DfuTarg', and take note of the
       device address. Make sure MCP is closed when running the script below.
    b. Go to
       <Install Folder>\Nordic\nrf51822\Board\nrf6310\device_firmware_updates\ble_dfu_send_hex\dfu
    c. Execute the DFU Update script from Command line:
       ipy main.py --file <image_file.hex> --address <target address>
       where:
          <image_file.hex> can be any application, softdevice, or bootloader image, subject to
                           restrictions described in notes below.
          <target address> is the address of the device advertising 'DfuTarg' found in a).
    d. the test_images_update folder contains dummy images that can be used to test the script.

When using the dummy images in test_images update, the transfer can be verified:
7.  The SoftDevice update can be verified as:
        Data at location 0x00003000: FFFFFF10 51B1E5DB 00016000 FFFFFFFE to verify that the new
        softdevice is in place (nrfjprog --memrd 0x3000 --w 4 --n 16). Notice the change in the last
        four hex digits (004F -> FFFE).
8.  The Bootloader update can be verified as:
        Open Master Control Panel, verify that the device is now advertising as: 'DfuTest' istead of
        'DfuTarg'. Also, LED3 will be lit in place of LED2.
         
When performing Application DFU:
9.  Remember to port your application to the new SoftDevice specifications. Refer to release notes
    for details.
    Important: The application must be compiled to execute from the correct address (0x16000 for
    s110 v7.0.0), and size adjusted accordingly.
   

Note:
1.  DFU mode can be entered, when an application is present, by holding button 7 on the nRF6310
    motherboard when resetting the nRF51822 device.
2.  Be aware that if an application is present when updating the SoftDevice, it will be deleted
    during SoftDevice update.
3.  Application needs to be uploaded using DFU if Bootloader is present.
4.  It is also possible to transfer a SoftDevice and Bootloader together. In order to do this, you
    must merge the hex files. This can be done manually or using the mergehex.exe tool provided with
    nrfjprog (Part of SDK installer). The default install folder of mergehex.exe is
       c:\Program Files (x86)\Nordic Semiconductor\nrf51\bin\
    To see how to use mergehex, type
       mergehex --help
    It might be neccesary to provide the full path to the mergehex.exe in case the tool is not
    included in PATH.
5.  If you get the error '[EXCEPTION] Access to the port 'COMxx' is denied.', try to close Master
    Control Panel.
6.  If the UICR.CLENR0 register is not 0xFFFFFFFF (e.g. when "Enable SoftDevice protection" is on in
    nRFgo Studio), any SoftDevice update must be to a SoftDevice of the same size. The UICR.CLENR0
    register can be read using:
      nrfjprog --memrd 0x10001000 --w 4 --n 1
    output examples:
      0x10001000: 00016000 (Only SoftDevice of size 0x16000 is supported (SoftDevice is protected))
      0x10001000: FFFFFFFF (SoftDevice of any size is supported)
