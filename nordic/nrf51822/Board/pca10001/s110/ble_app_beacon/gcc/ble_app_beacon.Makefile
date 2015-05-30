TARGET_CHIP := NRF51822_QFAA_CA
BOARD := BOARD_PCA10001

C_SOURCE_FILES += main.c
C_SOURCE_FILES += softdevice_handler.c
C_SOURCE_FILES += ble_advdata.c
C_SOURCE_FILES += ble_debug_assert_handler.c
C_SOURCE_FILES += ble_srv_common.c

SDK_PATH = ../../../../../


OUTPUT_FILENAME := ble_app_beacon

DEVICE_VARIANT := xxaa
#DEVICE_VARIANT := xxab

USE_SOFTDEVICE := S110
#USE_SOFTDEVICE := S210

# we do not use heap in this app
ASMFLAGS := -D__HEAP_SIZE=0

INCLUDEPATHS += -I"$(SDK_PATH)Include/s110"
INCLUDEPATHS += -I"$(SDK_PATH)Include/ble"
INCLUDEPATHS += -I"$(SDK_PATH)Include/ble/ble_services"
INCLUDEPATHS += -I"$(SDK_PATH)Include/app_common"
INCLUDEPATHS += -I"$(SDK_PATH)Include/sd_common"

C_SOURCE_PATHS += "$(SDK_PATH)Source/ble"
C_SOURCE_PATHS += "$(SDK_PATH)Source/app_common"
C_SOURCE_PATHS += "$(SDK_PATH)Source/sd_common"

include $(SDK_PATH)Source/templates/gcc/Makefile.common
