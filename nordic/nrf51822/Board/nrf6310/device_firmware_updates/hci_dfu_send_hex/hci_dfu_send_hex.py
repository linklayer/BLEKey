import sys, os
import time
import intelhex
from ctypes import c_uint32, c_uint8
from threading import Thread
from serial import Serial
from datetime import datetime, timedelta
import argparse
import crc16pure

VERSION = '1.0'


import getopt

def convert_uint32_to_array(val):
    uint16_data = c_uint32(val)
    byte0 = c_uint8((uint16_data.value & 0x000000FF))
    byte1 = c_uint8((uint16_data.value & 0x0000FF00) >> 8)
    byte2 = c_uint8((uint16_data.value & 0x00FF0000) >> 16)
    byte3 = c_uint8((uint16_data.value & 0xFF000000) >> 24)
    
    return [byte0.value, byte1.value, byte2.value, byte3.value]

class ThreadedFunction(Thread):
    
    def __init__(self, func, *args, **kwargs):
        self.func = func
        self.args = args 
        self.kwargs = kwargs
        super(ThreadedFunction, self).__init__()
    
    def run(self):
        self.func(*self.args, **self.kwargs)

class HexType:
  SOFTDEVICE  = 1
  BOOTLOADER  = 2
  SD_BL       = 3
  APPLICATION = 4

class Nrf51Hex(intelhex.IntelHex):
       
    def __init__(self, *args, **kwargs):
        super(Nrf51Hex, self).__init__(*args, **kwargs)
        self.sd_address_start = 0
        self.sd_address_end = 0
        self.bl_address_start = 0
        self.bl_address_end = 0
        self.app_address_start = 0
        self.app_address_end = 0

        if self.is_softdevice():
            self.sd_address_start = 4096
            self.sd_address_end = int(self.gets(0x3008, 4)[::-1].encode('hex'), 16)

        if self.is_bootloader():
            self.bl_address_start = self._resolve_start_address(0x30000, 0x40000)
            self.bl_address_end = self._resolve_end_address(0x30000, 0x40000)
            
        if self.is_application():
            self.app_address_start = self.minaddr()
            self.app_address_end = self.maxaddr() + 1

    def hex_type_get(self):
        if self.is_softdevice() and self.is_bootloader():
            return HexType.SD_BL

        if self.is_softdevice():
            return HexType.SOFTDEVICE

        if self.is_bootloader():
            return HexType.BOOTLOADER

        if self.is_application():
            return HexType.APPLICATION

    def hex_info(hexfile):
        start = hexfile.minaddr()
        end   = hexfile.maxaddr() + 1
        size  = end - start
        return size, start, end

    def is_softdevice(self):
        inside_boundry = self.minaddr() < 0x1004

        return True if (inside_boundry and self._has_sp(4096)) else False

    def is_application(self):
        inside_boundry = self.minaddr() > 0x13FFC and self.minaddr() < 0x30000

        return True if (inside_boundry and self._has_sp(self.minaddr())) else False

    def _resolve_start_address(self, lower_limit, upper_limit):
        test_address = lower_limit
        while test_address < upper_limit:
            try:
                has_sp = self._has_sp(test_address)
                if has_sp:
                    break
            except intelhex.NotEnoughDataError, e:
                test_address += 0x400
        
        # Return the found start address if this is less than the upper limit
        # else return None to indicate nothing was found        
        return test_address if test_address < upper_limit else None

    def _resolve_end_address(self, lower_limit, upper_limit):
        # find the last word in the bootloader, by going from the back until code is hit.
        end_address = upper_limit
        while upper_limit >= lower_limit:
            try:
                data = self.gets(end_address, 4)
                end_address = end_address + 0x04
                break

            except intelhex.NotEnoughDataError, e:
                end_address = end_address - 0x04

        return end_address

    def is_bootloader(self):
        inside_boundry = self.maxaddr() > 0x30000 
        has_sp = False

        addr_start = 0x30000
        addr_end = 0x40000

        start_address = self._resolve_start_address(addr_start, addr_end)

        # if not None, we have a valid SP in the start address
        if start_address:
            has_sp = True

        return True if (inside_boundry and has_sp) else False

    def _has_sp(self, sp_address):
        little_endian_sp_address = self.gets(sp_address, 4) 
        init_sp = int(little_endian_sp_address[::-1].encode('hex'), 16)
        #print "%x" % init_sp

        return True if (init_sp > 0x20000000) else False


def open_hex(filename):
    try:
        ih = intelhex.IntelHex(filename)
        return ih
    except (IOError, intelhex.IntelHexError), e:
        print ('Error reading file: %s\n' % e)
        raise Exception("Could not read hex format")

def parts_to_four_bytes(seq, dicp, rel, pkt_type, pkt_len):
    ints = [0,0,0,0]
    ints[0] = (seq) | (((seq+1)%8) << 3) | (dicp << 6) | (rel << 7)
    ints[1] = pkt_type | ((pkt_len & 0x000F) << 4)
    ints[2] = (pkt_len & 0x0FF0) >> 4
    ints[3] = (~(sum(ints[0:3]))+1) & 0xFF

    return ''.join(chr(b) for b in ints)

def int32_to_bytes(nr):
    ints = [0,0,0,0]
    ints[0] = (nr & 0x000000FF) 
    ints[1] = (nr & 0x0000FF00) >> 8
    ints[2] = (nr & 0x00FF0000) >> 16
    ints[3] = (nr & 0xFF000000) >> 24
    return ''.join(chr(b) for b in ints)     

def decode_esc_chars(data):
        '''Replace 0xDBDC with 0xCO and 0xDBDD with 0xDB'''
        result = []
        while len(data):
            char = data.pop(0)
            if char == 0xDB:
                char2 = data.pop(0)
                if char2 == 0xDC:
                    result.append(0xC0)
                elif char2 == 0xDD:
                    result.append(0xDB)
                else:
                    raise Exception('Char 0xDB NOT followed by 0xDC or 0xDD')
            else:
                result.append(char)
        return result
        
def encode_packet(data_in):
        '''Replace 0xCO  with 0xDBDC and 0xDB with 0xDBDD'''
        result = []
        data = []
        for i in data_in:
            data.append(ord(i))
        
        while len(data):
            char = data.pop(0)
            if char == 0xC0:
                result.extend([0xDB, 0xDC])
            elif char == 0xDB:
                result.extend([0xDB, 0xDD])
            else:
                result.append(char)
        return ''.join(chr(i) for i in result)

DATA_INTEGRITY_CHECK_PRESENT = 1
RELIABLE_PACKET = 1
HCI_PACKET_TYPE = 14

DFU_START_PACKET = 2
DFU_DATA_PACKET = 3
DFU_END_PACKET = 4

DFU_UPDATE_MODE_NONE = 0
DFU_UPDATE_MODE_SD = 1
DFU_UPDATE_MODE_BL = 2
DFU_UPDATE_MODE_APP = 4

class HciPacket(object):
    '''Class representing a single HCI packet'''
    
    sequence_number = 0
    def __init__(self, data=''):
        HciPacket.sequence_number = (HciPacket.sequence_number+1) % 8
        self.temp_data = ''
        self.temp_data += parts_to_four_bytes(HciPacket.sequence_number, 
                                        DATA_INTEGRITY_CHECK_PRESENT,
                                        RELIABLE_PACKET,
                                        HCI_PACKET_TYPE,
                                        len(data))
        self.temp_data += data
        # Add escape caracters
        crc = crc16pure.crc16xmodem(self.temp_data, crc=0xFFFF)

        self.temp_data += chr(crc & 0xFF)
        self.temp_data += chr((crc & 0xFF00) >> 8)
                
        self.temp_data = encode_packet(self.temp_data)
        
        self.data = chr(0xc0)
        self.data += self.temp_data
        self.data += chr(0xc0)

class Controller(object):
    def __init__(self):
        self.file_path = None
        self.com_port = None
        self.flow_control = None
        self.baud_rate = None
        self.verbose = None
        self.progress_value = -1
        
    def progress_callback(self, value, bar_length=30):
        if value != self.progress_value:
            self.progress_value = value
            hashes = '#' * int((float(self.progress_value)/100) * bar_length)
            spaces = ' ' * (bar_length - len(hashes))
            sys.stdout.write("\rProgress: [{0}] {1}%".format(hashes + spaces, self.progress_value))
            sys.stdout.flush()
            #print "Progress: ", self.progress_value
  
    def timeout_callback(self):
        print "Transmission timeout."
        
    def set_filepath(self, filepath):
        self.file_path = filepath
        if not os.path.isfile(os.path.abspath(self.file_path)):
            print "File does not exist: %s" % os.path.abspath(self.file_path)
            sys.exit(1)
        
    def upload_file(self):
        if not self.file_path:
            raise Exception("File path not set, or not found.")
        
        s = ThreadedFunction(self.upload_firmware)
        s.start()

    # Verbose summary
    def verbose_info(self, hex_file):
            print ""
            print "SoftDevice update:", hex_file.is_softdevice()
            print "    > SD start address: 0x%0.8x" % hex_file.sd_address_start
            print "    > SD end address: 0x%0.8x" % hex_file.sd_address_end
            print "    > SD size: 0x%0.8x" % (hex_file.sd_address_end - hex_file.sd_address_start)
            print "Bootloader update:", hex_file.is_bootloader()
            print "    > BL start address: 0x%0.8x" % hex_file.bl_address_start
            print "    > BL end address: 0x%0.8x" % hex_file.bl_address_end
            print "    > BL size: 0x%0.8x" % (hex_file.bl_address_end - hex_file.bl_address_start)
            print "Application update:", hex_file.is_application()
            print "    > APP start address: 0x%0.8x" % hex_file.app_address_start
            print "    > APP start address: 0x%0.8x" % hex_file.app_address_end
            print "    > APP size: 0x%0.8x" % (hex_file.app_address_end - hex_file.app_address_start)
            print ""

    def upload_firmware(self):
        
        def percentage(part, whole):
            return int(100 * float(part)/float(whole))
        
        def get_ack_nr(uart):
        
            def is_timeout(start_time, timeout_sec):
                return not (datetime.now() - start_time <= timedelta(0, timeout_sec))
                
            uart_buffer = []
            start = datetime.now()
            while uart_buffer.count(0xC0) < 2:
                #Disregard first of the two C0
                temp = uart.read(1)
                if temp:
                    uart_buffer.append(ord(temp))
                    
                if is_timeout(start, 5):
                    # reset HciPacket numbering back to 0
                    HciPacket.sequence_number = 0
                    
                    # call timeout callback from parent layer
                    self.timeout_callback()

                    # quit loop
                    break
                    
                #read until you get a new C0
                #RESUME_WORK
            data = decode_esc_chars(uart_buffer)
            # Remove 0xC0 at start and beginning
            data = data[1:-1]
#            print "non-slip data ", [data]
            return (data[0] >> 3) & 0x07
        
        def start_packet_generate(hex_file):
            def word_align(a, b):
                return (((a) + (b - 1)) &~(b - 1))
                
            hex_type = hex_file.hex_type_get()

            start_data = int32_to_bytes(DFU_START_PACKET)
            start_data += int32_to_bytes(hex_type)
            # size of SoftDevice
            start_data += int32_to_bytes(word_align(hex_file.sd_address_end, 4) - hex_file.sd_address_start) 
            # size of BootLoader
            start_data += int32_to_bytes(word_align(hex_file.bl_address_end, 4) - hex_file.bl_address_start) 
            # size of Application
            start_data += int32_to_bytes(word_align(hex_file.app_address_end, 4) - hex_file.app_address_start) 

            packet = HciPacket(start_data)

            # print "\n\nstart packet data:",
            # for i in packet.data:
            #       print hex(ord(i)),
            # print ""

            return packet

        def stop_packet_generate():

            stop_data = int32_to_bytes(DFU_END_PACKET)
            
            packet = HciPacket(stop_data)

            # print "\n\nstop packet data:",
            # for i in packet.data:
            #       print hex(ord(i)),

            return packet
        
        def data_packets_generate(hex_file):  
            def word_align(a, b):
                return (((a) + (b - 1)) &~(b - 1))
            foo_packets = []  
            bin_image = ""
            if ih.is_softdevice():
                bin_image += ih.tobinstr(start = hex_file.sd_address_start, end = hex_file.sd_address_end - 1) # remove last address as intelhex module includes the end address.
            if ih.is_bootloader():
                bin_image += ih.tobinstr(start = hex_file.bl_address_start, end = hex_file.bl_address_end - 1)
            if ih.is_application():
                bin_image += ih.tobinstr(start = hex_file.app_address_start, end = hex_file.app_address_end - 1)
            for i in range(0, len(bin_image), 512):
                data_packet = HciPacket(int32_to_bytes(DFU_DATA_PACKET) + bin_image[i:i+512])

                # print "\n\ndata packet data:",
                # for i in data_packet.data:
                #     print hex(ord(i)),

                foo_packets.append(data_packet)

            return foo_packets

        #self.test_sd()

        ih = Nrf51Hex(self.file_path)

        # if verbose flag is set.
        if self.verbose:
            self.verbose_info(ih)

        packets = []
        
        # Add start packet
        start_packet = start_packet_generate(ih)
        packets.append(start_packet)

        # Add hex data packets
        data_packets = data_packets_generate(ih)
        packets.extend(data_packets)
        
        # Add stop packet
        stop_packet = stop_packet_generate()
        packets.append(stop_packet)
        
        if self.verbose:
            print "Total number of HCI packets: %i" % len(packets)

        uart = None
        
        try:
            uart = Serial(self.com_port, self.baud_rate, rtscts=self.flow_control, timeout=1)
        except Exception, e:
            print "UART could not be opened on %s" % self.com_port
            print e 
        try:
            if uart:
                print ""
                # Transmit START
                for seq, pkt in enumerate(packets[:-1]):
                    attempts = 0
                    last_ack = None
                    packet_sent = False
                    while packet_sent == False:
                        uart.write(pkt.data)
                        attempts += 1
                        ack = get_ack_nr(uart)
                        if last_ack == None:
                            last_ack = ack
                            break

                        if ack == (last_ack+1) % 8:
                            last_ack = ack
                            packet_sent = True
                            
                        if attempts > 3:
                            raise Exception("Three failed tx attempts encountered on packet {0}".format(seq))
                
                
                    self.progress_callback(percentage(seq, len(packets)))
                time.sleep(1)
                uart.write(packets[-1].data)
                self.progress_callback(percentage(len(packets), len(packets)))
                print ""
                print ""
                print "Success!"
                print ""
                uart.close()

        except IndexError, e:
            print "Ack out of sequence, or no ack returned"
        
def main(argv,arglen):    
    print ("")
           

if __name__ == '__main__':
    
    controller = Controller()
    
    parser = argparse.ArgumentParser(description='Hex File of firmware to be upgraded.')
    parser.add_argument('--file', '-f',
                        type=str,
                        required=True,
                        dest='file',
                        help='Filename of Hex file.')
    parser.add_argument('--port', '-p',
                        type=str,
                        required=True,
                        dest='comport',
                        help='COM Port to which the device is connected.')
    parser.add_argument('--flowcontrol', '-fc',
                        action='store_true',
                        required=False,
                        dest='flowcontrol_bool',
                        help='Enable flow control, default: disabled.')
    parser.add_argument('--baudrate', '-b',
                        type=int,
                        required=False,
                        default=38400,
                        dest='baudrate',
                        help='Desired baudrate 38400/96000/115200/230400/250000/460800/921600/1000000 (default: 38400). Note: Baud rates >115200 are supported by nRF51822, but may not be supported by all RS232 devices on windows.')
    parser.add_argument('--verbose', '-v',
                        action='store_true',
                        default=False,
                        dest='verbose',
                        help='Enable verbose mode.')


    args = parser.parse_args()
    print 'Sending file {0} to {1}, flow control = {2}'.format(args.file, args.comport, args.flowcontrol_bool)
    
    controller.com_port = args.comport.strip()
    controller.flow_control = args.flowcontrol_bool
    controller.baud_rate = args.baudrate 
    controller.verbose = args.verbose
    
    controller.set_filepath(args.file.strip())
    controller.upload_file()
    
