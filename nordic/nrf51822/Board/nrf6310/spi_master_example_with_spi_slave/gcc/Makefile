TARGET_CHIP := NRF51822_QFAA_CA
BOARD := BOARD_NRF6310

CFLAGS += -DSPI_MASTER_0_ENABLE

C_SOURCE_FILES += main.c
C_SOURCE_FILES += spi_master.c
C_SOURCE_FILES += nrf_delay.c
C_SOURCE_FILES += nrf_soc.c

OUTPUT_FILENAME := spi_master_example_gcc
SDK_PATH = ../../../../


#Uncomment correct line if you have s110 programmed on the chip.
DEVICE_VARIANT := xxaa
#DEVICE_VARIANT := xxab

#USE_SOFTDEVICE := s110
#USE_SOFTDEVICE := s210

include $(SDK_PATH)Source/templates/gcc/Makefile.common
INCLUDEPATHS += -I"$(SDK_PATH)Include/app_common"
INCLUDEPATHS += -I"$(SDK_PATH)Include/sdk_soc"
#INCLUDEPATHS += -I"$(SDK_PATH)Include/s110"