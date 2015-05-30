TARGET_CHIP := NRF51822_QFAA_CA
BOARD := BOARD_NRF6310

C_SOURCE_FILES += main_device_ack_payload.c

SDK_PATH = ../../../../../
INCLUDEPATHS += -I"$(SDK_PATH)Include/gzll"
LIBRARIES += $(SDK_PATH)Lib/gzll/gcc/gzll_gcc.a
OUTPUT_FILENAME := gzll_device_ack_payload_gcc

DEVICE_VARIANT := xxaa
#DEVICE_VARIANT := xxab

#USE_SOFTDEVICE := S110
#USE_SOFTDEVICE := S210

include $(SDK_PATH)Source/templates/gcc/Makefile.common