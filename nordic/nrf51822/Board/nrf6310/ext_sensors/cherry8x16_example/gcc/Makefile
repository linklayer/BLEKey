TARGET_CHIP := NRF51822_QFAA_CA
BOARD := BOARD_NRF6310

C_SOURCE_FILES += main.c
C_SOURCE_FILES += cherry8x16.c
C_SOURCE_FILES += simple_uart.c
C_SOURCE_FILES += nrf_delay.c

OUTPUT_FILENAME := cherry8x16_example_gcc
SDK_PATH = ../../../../../


#Uncomment correct line if you have s110 programmed on the chip.
DEVICE_VARIANT := xxaa
#DEVICE_VARIANT := xxab

#USE_SOFTDEVICE := s110
#USE_SOFTDEVICE := s210

include $(SDK_PATH)Source/templates/gcc/Makefile.common
