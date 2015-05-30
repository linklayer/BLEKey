import time
from intelhex import IntelHex
from dfu.master_emulator import MasterEmulator

import System
import Nordicsemi

# DFU OpCodes
class OpCodes:
    START_DFU = 1
    INITIALIZE_DFU = 2
    RECEIVE_FIRMWARE_IMAGE = 3
    VALIDATE_FIRMWARE_IMAGE = 4
    ACTIVATE_FIRMWARE_AND_RESET = 5
    SYSTEM_RESET = 6
    REQ_PKT_RCPT_NOTIF = 8
    RESPONSE = 16
    PKT_RCPT_NOTIF = 17

class DfuMode:
  SOFTDEVICE  = 1
  BOOTLOADER  = 2
  SD_BL       = 3
  APPLICATION = 4

# Textual description lookup table for status codes received from peer.
status_code_lookup = {
     1: "SUCCESS",
     2: "Invalid State",
     3: "Not Supported",
     4: "Data Size Exceeds Limit",
     5: "CRC Error",
     6: "Operation Failed"
}

# Helper functions
def create_byte_array(size, value=0x55):
    """ Create a IronPython byte array with initial value. """
    return System.Array[System.Byte]([value]*size)

def convert_uint32_to_array(value):
    """ Convert a number into an array of 4 bytes (LSB). """
    return [(value >> 0 & 0xFF), (value >> 8 & 0xFF),
            (value >> 16 & 0xFF), (value >> 24 & 0xFF)]

def convert_uint16_to_array(value):
    """ Convert a number into an array of 2 bytes (LSB). """
    return [(value >> 0 & 0xFF), (value >> 8 & 0xFF)]

# Service UUID
uuid_dfu_service = Nordicsemi.BtUuid('000015301212EFDE1523785FEABCD123')
# Characteristic UUID
uuid_dfu_control_state_characteristic = Nordicsemi.BtUuid('000015311212EFDE1523785FEABCD123')
uuid_dfu_packet_characteristic = Nordicsemi.BtUuid('000015321212EFDE1523785FEABCD123')
#Descriptor UUID
uuid_client_characteristic_configuration_descriptor = Nordicsemi.BtUuid(0x2902)

# number of retries for sending a packet
num_of_send_tries = 1

# NOTE:  If packet receipt notification is enabled, a packet receipt
#        notification will be received for each 'num_of_packets_between_notif'
#        number of packets.
#
# Configuration tip: Increase this to get lesser notifications from the DFU
# Target about packet receipts. Make it 0 to disable the packet receipt
# notification
num_of_packets_between_notif = 0

class BleDfu(MasterEmulator):
    """ Class to handle upload of a new hex image to the peer device. """

    def __init__(self, peer_device_address, hexfile_path):
        super(BleDfu, self).__init__(peer_device_address)

        self.hexfile_path = hexfile_path
        self.ready_to_send = True

    def ProgressChanged(self, percent, message, finished):
        pass

    def data_received_handler(self, sender, e):
        """ Handle received data from the peer device.
        Note: This function overrides the parent class.
        """
        if (e.PipeNumber == self.pipe_dfu_control_point_notify):
            op_code = int(e.PipeData[0])

            if op_code == OpCodes.PKT_RCPT_NOTIF:
                if self.ready_to_send == True:
                    self.log_handler.log("ERROR: !!!! Pkt receipt notification received when it is not expected")
                else:
                    self.log_handler.log("Pkt receipt notification received.")

                self.log_handler.log("Number of bytes LSB = {0}".format(e.PipeData[1]))
                self.log_handler.log("Number of bytes MSB = {0}".format(e.PipeData[2]))
                self.ready_to_send = True

            elif op_code == OpCodes.RESPONSE:
                request_op_code = int(e.PipeData[1])
                response_value = int(e.PipeData[2])
                self.log_handler.log("Response received for Request Op Code = {0}".format(request_op_code))

                status_text = "UNKNOWN"
                if status_code_lookup.has_key(response_value):
                    status_text = status_code_lookup[response_value]

                if request_op_code == 1:
                    self.log_handler.log("Response for 'Start DFU' received - Status: %s" % status_text)
                elif request_op_code == 2:
                    self.log_handler.log("Response for 'Initialize DFU Params' received - Status: %s" % status_text)
                elif request_op_code == 3:
                    self.log_handler.log("Response for 'Receive FW Data' received - Status: %s" % status_text)
                elif request_op_code == 4:
                    self.log_handler.log("Response for 'Validate' received - Status: %s" % status_text)
                else:
                    self.log_handler.log("!!ERROR!! Response for Unknown command received.")
        else:
            self.log_handler.log("Received data on unexpected pipe %r"%e.PipeNumber)

    def setup_service(self):
        """ Set up DFU service database. """
        # Add DFU Service
        self.master.SetupAddService(uuid_dfu_service, Nordicsemi.PipeStore.Remote)

        # Add DFU characteristics
        self.master.SetupAddCharacteristicDefinition(uuid_dfu_packet_characteristic, 2, create_byte_array(2))
        self.pipe_dfu_packet = self.master.SetupAssignPipe(Nordicsemi.PipeType.Transmit)

        self.master.SetupAddCharacteristicDefinition(uuid_dfu_control_state_characteristic, 2, create_byte_array(2))
        self.pipe_dfu_control_point = self.master.SetupAssignPipe(Nordicsemi.PipeType.TransmitWithAck)
        self.pipe_dfu_control_point_notify = self.master.SetupAssignPipe(Nordicsemi.PipeType.Receive)

    def dfu_send_image(self):
        """ Send hex to peer in chunks of 20 bytes. """
        if not self.connected:
            return
     
        padding = [0x00] * 12

        # Open the hex file to be sent
        ih = IntelHex(self.hexfile_path)

        updating_sd  = False
        updating_bl  = False
        updating_app = False
        
        bl_start_address = 0x30000
        bl_end_address = 0x40000
        app_start_address = 0xFFFFFFFF

        if ih.minaddr() < 0x1004:
          updating_sd = True
          sd_end_address_str = ih.gets(0x3008, 4)
          sd_end_address = int(sd_end_address_str[::-1].encode('hex'), 16)

        if ih.minaddr() > 0x13FFC:
            if ih.minaddr() < 0x30000:
                updating_app = True
                app_start_address = ih.minaddr()

        if ih.maxaddr() > 0x30000:
          while bl_start_address < 0x40000:
              try:
                  bl_loader_start = ih.gets(bl_start_address, 4)
                  init_sp = int(bl_loader_start[::-1].encode('hex'), 16)
                  if init_sp > 0x20000000:
                      updating_bl = True
                      break
	          else:
                      bl_start_address = bl_start_address + 0x1000
              except Exception, e:
                  bl_start_address = bl_start_address + 0x1000

          while bl_start_address < bl_end_address:
              try:
                  bl_loader_start = ih.gets(bl_end_address, 4)
                  bl_end_address = bl_end_address + 0x04
                  break
              except Exception, e:
                  bl_end_address = bl_end_address - 0x04


        image_size_packet = []
        bin_array = []
        hex_size = 0

        if updating_sd:
          sd_bin_array = ih.tobinarray(start = 4096, end = (sd_end_address - 1))
          hex_size = hex_size + len(sd_bin_array)
          sd_hex_size_array_lsb = convert_uint32_to_array(len(sd_bin_array))
          image_size_packet.extend(sd_hex_size_array_lsb)
          bin_array.extend(sd_bin_array)
        else:
          image_size_packet.extend(padding[0:4])

        if updating_bl:
          bl_bin_array = ih.tobinarray(start = bl_start_address, end = (bl_end_address - 1))
          hex_size = hex_size + len(bl_bin_array)
          bl_hex_size_array_lsb = convert_uint32_to_array(len(bl_bin_array))
          image_size_packet.extend(bl_hex_size_array_lsb)
          bin_array.extend(bl_bin_array)
        else:
          image_size_packet.extend(padding[0:4])

        if updating_app:
          app_bin_array = ih.tobinarray(start = app_start_address)
          hex_size = hex_size + len(app_bin_array)
          app_hex_size_array_lsb = convert_uint32_to_array(len(app_bin_array))
          image_size_packet.extend(app_hex_size_array_lsb)
          bin_array.extend(app_bin_array)
        else:
          image_size_packet.extend(padding[0:4])


        # CCCD Enable notification bytes
        start_time = time.time()

        try:
            self.master.OpenRemotePipe(self.pipe_dfu_control_point_notify)
        except Exception, e:
#            self.ProgressChanged(0, "Device does not have the required DFU service.", True)
            return False


        if num_of_packets_between_notif:
            # Subscribing for packet receipt notifications
            self.send_data(self.pipe_dfu_control_point,
                           System.Array[System.Byte]([OpCodes.REQ_PKT_RCPT_NOTIF] + convert_uint16_to_array(num_of_packets_between_notif)),
                           num_of_send_tries,
                           "Enabling Packet receipt notifications from peer device")

        # Sending 'START DFU' command
        if updating_sd:
          if updating_bl:
            self.send_data(self.pipe_dfu_control_point,
                           System.Array[System.Byte]([OpCodes.START_DFU, DfuMode.SD_BL]),
                           num_of_send_tries,
                           "Sending 'START DFU' command")
          else:
            self.send_data(self.pipe_dfu_control_point,
                           System.Array[System.Byte]([OpCodes.START_DFU, DfuMode.SOFTDEVICE]),
                           num_of_send_tries,
                           "Sending 'START DFU' command")

        elif updating_bl:
            self.send_data(self.pipe_dfu_control_point,
                           System.Array[System.Byte]([OpCodes.START_DFU, DfuMode.BOOTLOADER]),
                           num_of_send_tries,
                           "Sending 'START DFU' command")
        else:
          self.send_data(self.pipe_dfu_control_point,
                         System.Array[System.Byte]([OpCodes.START_DFU, DfuMode.APPLICATION]),
                         num_of_send_tries,
                         "Sending 'START DFU' command")

        # Sending image size
        self.send_data(self.pipe_dfu_packet,
                       System.Array[System.Byte](image_size_packet),
                       num_of_send_tries,
                       "Sending image size")

        # Send 'RECEIVE FIRMWARE IMAGE' command to set DFU in firmware receive state.
        self.send_data(self.pipe_dfu_control_point,
                       System.Array[System.Byte]([OpCodes.RECEIVE_FIRMWARE_IMAGE]),
                       num_of_send_tries,
                       "Send 'RECEIVE FIRMWARE IMAGE' command")

        self.ready_to_send = True;
        pkts_sent = 0;

        if updating_sd:
          time.sleep(10)

        # Send application data packets
        for i in range(0, hex_size, 20):
            last_progress = 0
            progress = float(i) / hex_size * 100

            if progress != last_progress:
#              self.ProgressChanged(progress, "Uploading firmware...")
              last_progress = progress

            #if not self.master.IsConnected:
                #return False
            if num_of_packets_between_notif:
                while not self.ready_to_send:
                    self.log_handler.log("Waiting for packet receipt notification")
                    time.sleep(0.1)
                    #wait for 'self.ready_to_send' to be True


            data_to_send = bin_array[i:i + 20]
            # Send 20 bytes of hex image data
            self.send_data(self.pipe_dfu_packet,
                           System.Array[System.Byte](data_to_send),
                           num_of_send_tries,
                           "Sending Firmware bytes [%i, %i]" % (i, i + len(data_to_send)))

            pkts_sent = pkts_sent + 1

            if ((num_of_packets_between_notif != 0) and ((pkts_sent % num_of_packets_between_notif) == 0)):
                # Need to wait for a notification from peer
                self.log_handler.log("Need to wait for a notification from peer")
                self.ready_to_send = False

        # Send Validate
        self.send_data(self.pipe_dfu_control_point,
                       System.Array[System.Byte]([OpCodes.VALIDATE_FIRMWARE_IMAGE]),
                       num_of_send_tries,
                       "Sending 'VALIDATE FIRMWARE IMAGE' command")

        # Wait for notification
        time.sleep(1)

        # Send Activate and Reset
        self.send_data(self.pipe_dfu_control_point,
                       System.Array[System.Byte]([OpCodes.ACTIVATE_FIRMWARE_AND_RESET]),
                       num_of_send_tries,
                       "Sending 'ACTIVATE FIRMWARE AND RESET' command")

        end_time = time.time()
        self.log_handler.log("Total size of the Image = {0} bytes".format(len(bin_array)))
        self.log_handler.log("Time taken (excluding the service discovery) = {0} seconds".format(end_time - start_time))

        return True
