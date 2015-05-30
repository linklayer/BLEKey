TARGET_CHIP := NRF51822_QFAA_CA
BOARD := BOARD_NRF6310

C_SOURCE_FILES += main_esb_ptx_ack_payload.c

SDK_PATH = ../../../../../
INCLUDEPATHS += -I"$(SDK_PATH)Include/esb"
LIBRARIES += $(SDK_PATH)Lib/esb/gcc/esb_gcc.a
OUTPUT_FILENAME := esb_ptx_ack_payload_gcc

DEVICE_VARIANT := xxaa

include $(SDK_PATH)Source/templates/gcc/Makefile.common