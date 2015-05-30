import sys, os
import time
import traceback

from dfu.log_handler import TimestampLogger

import Nordicsemi

class MasterEmulator(object):
    """ Master Emulator DLL wrapper. """

    def __init__(self, peer_device_address):

        self.peer_device_address = peer_device_address

        self.num_of_errors = 0
        self.log_handler = TimestampLogger()
        self.connected = False
        self.disconnect_event_expected = True
        self.service_setup_done = True
        self.last_disconnect_reason = -1

    def log_message_handler(self, sender, e):
        """ Log function to be passed to the DLL. """
        self.log_handler.log("[Emulator] %s" % e.Value)

    def data_received_handler(self, sender, e):
        """ Callback for the DLL if any data is recieved from peer device. """
        pipe_number = e.PipeNumber
        data = "".join("%02x" % byte for byte in e.PipeData)
        self.log_handler.log("Received unhandled data on pipe {0} ({1})!".format(pipe_number, data))

    def connected_handler(self, sender, e):
        """ Callback for the DLL if connected event occur device. """
        self.connected = True
        self.log_handler.log("Connected to peer device")

    def disconnected_handler(self, sender, e):
        """ Callback for the DLL if disconnect from peer device occur. """
        self.connected = False
        self.last_disconnect_reason = e.Value
        if self.disconnect_event_expected:
            self.log_handler.log("Disconnected from peer device")
            self.disconnect_event_expected = False
        else:
            self.log_handler.log("Error: Unexpected disconnection event!")
            self.num_of_errors += 1

    def pipe_error_handler(self, sender, e):
        """ Callback for the DLL if pipe errors occur. """
        self.log_handler.log("Pipe error received on pipe {0}. ErrorCode = {1}".format(e.PipeNumber, e.ErrorCode))
        self.num_of_errors += 1

    def _wait_for_disconnect(self, wait_for_disconnect_delay=None, expected_disconnected_reason=None):
        """ Internal function that waits for N seconds for disconnected_handler
        to be called, setting self.connected to False. Then, it will compare
        disconnect reason against the expected. """
        self.disconnect_event_expected = True
        for i in range(50):
            if self.connected:
                time.sleep(0.1)
                sys.stdout.write(".")
            else:
                # Disconnect received. Check disconnect reason if needed.
                if (expected_disconnected_reason != None):
                    if (expected_disconnected_reason != int(self.last_disconnect_reason)):
                        self.log_handler.log("Incorrect disconnect reason. Expected = {0} Received = {1}".format(expected_disconnected_reason, self.last_disconnect_reason))
                        raise(Exception("Incorrect disconnect reason. Expected = {0} Received = {1}".format(expected_disconnected_reason, self.last_disconnect_reason)))
                break
        else:
            raise(Exception("Peer device did not disconnect!"))

    def _disconnect(self):
        """ Internal function for making device disconnect from peer device. """
        self.disconnect_event_expected = True
        return self.master.Disconnect()

    def _discover_peer_named(self, btle_address):
        """ Internal function which starts Master Emulator discovery, compare found devices against
        name and address. If a match is found, return its address. """
        peer_device = None

        found_devices = self.master.DiscoverDevices()

        if found_devices:
            for device in found_devices:
                # print "device %r"%device.DeviceInfo
                self.log_handler.log("Device address %r" % (device.DeviceAddress.Value))

                if btle_address == device.DeviceAddress.Value:
                    peer_device = device
                    break

        return peer_device

    def _discover_peer_device(self):
        """ Internal function that does 10 retries to find the peer device
        with correct name and address. """
        for i in range(10):
            peerDevice = self._discover_peer_named(self.peer_device_address)
            if peerDevice != None:
                return peerDevice
        else:
            raise(Exception("Peer device with address {0} not found".format(self.peer_device_address)))

    def send_data(self, pipe, msg, count, description):
        """ Send data to peer device. """
        if not self.connected:
            return
        self.log_handler.log(description)
        for i in range(count):
            if not self.master.SendData(pipe, msg):
                raise Exception("Data could not be sent - failed!!!")


    def setup_service(self):
        """ Function for setting up services.

        If not no child class implements this function, service setup will be
        flagged as not done. This will prevent service discovery.
        """
        self.log_handler.log("Not setting up any services!")
        self.service_setup_done = False

    def set_local_data(self):
        """ Function for setting up local data.

        If not no child class implements this function, nothing will be set up.
        """
        self.log_handler.log("Not setting local data before run!")

    def filter_emulator_device(self, emulator_devices, emulator_filter):

        if not type(emulator_filter) is str:
            return None

        for device in emulator_devices:
            if emulator_filter in device:
                return device

        return None

    def scan_and_connect(self, emulator_filter=""):
        """ Function for doing discovery of the peer device, connect and
        discover pipes. The function will also open the Master Emulator. """
        try:
            self.master = None

            self.master = Nordicsemi.MasterEmulator()

            self.master.LogMessage += self.log_message_handler
            self.master.DataReceived += self.data_received_handler
            self.master.Connected += self.connected_handler
            self.master.Disconnected += self.disconnected_handler
            self.master.PipeError += self.pipe_error_handler

            emulator_devices = self.master.EnumerateUsb(Nordicsemi.UsbDeviceType.AnyMasterEmulator)

            emulator_device = self.filter_emulator_device(emulator_devices, emulator_filter)

            if emulator_device is None:
                raise Exception("Could not find emulator device")

            self.master.Open(emulator_device)

            self.setup_service()

            # Start Master Emulator
            self.master.Run()

            self.set_local_data()

            # Run discovery procedure
            self.myPeerDevice = self._discover_peer_device()
            self.log_handler.log("Found peer device")

            # Connect and bond to peer device
            if self.master.Connect(self.myPeerDevice.DeviceAddress):
                self.log_handler.log("--- Connected ---")
            else:
                raise(Exception("Connection failed"))

            if self.service_setup_done:
                # Service discovery
                found_pipes = self.master.DiscoverPipes()

                print "PIPES RETURN VALUE: %s" % found_pipes

                if not found_pipes:
                    return False

                self.log_handler.log("--- Pipes discovered ---")

        except Exception, ex:
            self.log_handler.log("[EXCEPTION] %s" % ex)
            self.log_handler.log("Call stack:")
            tb = traceback.extract_tb(sys.exc_info()[2])
            for f in reversed(tb):
                self.log_handler.log("  File {0}, line {1}".format(os.path.basename(f[0]), f[1]))
            self.num_of_errors += 1
            self.log_handler.log("number of errors: %i" % self.num_of_errors)
            return False

        return True

    def disconnect(self):
        """ Function for disconnecting from peer device and close the
        Master Emulator if opened. """
        if self.connected:
            try:
                self.log_handler.log("Will disconnect now")
                self._disconnect()
                self._wait_for_disconnect()
            except Exception, ex:
                self.log_handler.log("[EXCEPTION] %s" % ex)
                self.log_handler.log("Call stack:")
                tb = traceback.extract_tb(sys.exc_info()[2])
                for f in reversed(tb):
                    self.log_handler.log("  File {0}, line {1}".format(os.path.basename(f[0]), f[1]))
                self.num_of_errors += 1
        else:
            self.log_handler.log("Not connected - Skipping")

        # close MasterEmulator if open
        if (self.master != None) and self.master.IsOpen:
            self.master.Close()
