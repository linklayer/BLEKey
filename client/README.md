BLEKey Client
=============

The pygatt python library needs several modifications to work properly.

1. Use the repo at https://github.com/blark/pygatt as I had to add functionality to pass options to gatttool to make it work. I've submitted a pull request but for now, use this.
2. I had to manually edit the constants.py to increase the DEFAULT_TIMEOUT_S timeout to around 10sec because I couldn't figure out how to do it in blekey.py. 

