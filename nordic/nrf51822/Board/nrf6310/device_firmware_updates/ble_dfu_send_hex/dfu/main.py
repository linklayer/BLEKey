import time
import argparse
import sys
import os
import clr
sys.path.append(r'.\..')
import dfu

try:
    programfilesPath = os.environ['PROGRAMFILES']
    masterApiBasePath = os.path.join(programfilesPath, r'Nordic Semiconductor\Master Emulator')
    dirsandfiles = os.listdir(masterApiBasePath)
    dirs = []
    for element in dirsandfiles:
        if os.path.isdir(os.path.join(masterApiBasePath, element)):
            dirs.append(element)
    if len(dirs) == 0:
        raise Exception('Master Emulator directory not found.')
    dirs.sort()
    masterApiPath = os.path.join(masterApiBasePath, dirs[-1])
    print masterApiPath
    sys.path.append(masterApiPath)
    clr.AddReferenceToFile("MasterEmulator.dll")
except Exception, e:
    raise Exception("Cannot load MasterEmulator.dll")

from dfu.ble_dfu import BleDfu


def main():
    parser = argparse.ArgumentParser(description='Send hex file over-the-air via BLE')
    parser.add_argument('--file', '-f',
                        type=str,
                        required=True,
                        dest='file',
                        help='Filename of Hex file.')
    parser.add_argument('--address', '-a',
                        type=str,
                        required=True,
                        dest='address',
                        help='Advertising address of nrf51822 device.')

    args = parser.parse_args()
    print 'Sending file {0} to device {1}'.format(args.file, args.address.upper())

    ble_dfu = BleDfu(args.address.upper(), args.file)

    # Connect to peer device.
    ble_dfu.scan_and_connect()

    # Transmit the hex image to peer device.
    ble_dfu.dfu_send_image()

    # wait a second to be able to recieve the disconnect event from peer device.
    time.sleep(1)

    # Disconnect from peer device if not done already and clean up.
    ble_dfu.disconnect()

if __name__ == '__main__':
    main()
