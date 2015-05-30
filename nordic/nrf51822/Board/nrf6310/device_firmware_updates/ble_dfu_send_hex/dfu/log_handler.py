import sys
from datetime import datetime

class TimestampLogger(object):
    """ Class for printing out log text with a timestamp. """
    
    def log(self, msg):
        """ Function for printing out message with timestamp. """
        ct = datetime.now()
        formattedMsg = "[{0:02d}:{1:02d}:{2:02d}.{3:03d}] {4}\n".format(ct.hour, ct.minute, ct.second, ct.microsecond / 1000, msg)
        sys.stdout.write(formattedMsg)
        sys.stdout.flush()

