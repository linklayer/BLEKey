BLEKey Client
=============

Requirements
------------

1. hcitool and gatttool are required to use this client. Install them on your Linux OS.
2. The pygatt python library needs several modifications to work properly. Use the repo at https://github.com/blark/pygatt

Notes about pygatt module:

- I had to manually edit the constants.py to increase the `DEFAULT_TIMEOUT_S` timeout to around 10sec because I couldn't figure out how to do it in blekey.py.
- I've submitted a pull request so to merge my changes into the default pip package. 
