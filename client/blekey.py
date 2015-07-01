import cmd
import pygatt
import pprint as pp
import os

DEFAULT_MAC = "D4:34:E8:CA:6F:6A"


class BLEKeyClient(cmd.Cmd):
    """Derp de derp"""

    macs = []

    def emptyline(self):
        pass

    def do_scan(self, arg):
        print("scanning...")
        scan_result = pygatt.util.lescan()
        pp.pprint(scan_result)
        for item in scan_result:
            self.macs.append(item['address'])

    def help_scan(self):
        print("scan help goes here")

    def do_connect(self, mac):
        if not mac:
            mac = DEFAULT_MAC
        print("connecting to %s" % mac)
        self.bk = pygatt.BluetoothLEDevice(mac, app_options="-t random")
        self.bk.connect(timeout=10)

    def complete_connect(self, text, line, begidx, endidx):
        if not text:
            completions = self.macs[:]
        else:
            completions = [f
                           for f in self.macs
                           if f.startswith(text)
                           ]
        return completions

    def do_tx(self, data):
        print("sending...")
        self.bk.char_write(0x0d, [0x01])

    def do_readcards(self, _):
        print("reading last cards...")
        last_cards = self.bk.char_read_hnd(0x0b)
        for i in range(0, 3):
            start = i * 7
            print ("Bits: %d" % last_cards[start])
            card_data = reversed(last_cards[start + 1:start + 6])
            fixed = ''.join('{:02x}'.format(x) for x in card_data)
            print ("0x%s" % fixed)
            fixed = None
            card_data = None

    def do_EOF(self, line):
        return True

    do_exit = do_quit = do_EOF

if __name__ == '__main__':
    if os.getuid() is not 0:
        print("Warning BLE tools need to be run as root! UID 0 not detected")
    BLEKeyClient().cmdloop()
