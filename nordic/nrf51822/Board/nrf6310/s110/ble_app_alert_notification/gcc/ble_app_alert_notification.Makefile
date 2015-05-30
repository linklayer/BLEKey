TARGET_CHIP := NRF51822_QFAA_CA
BOARD := BOARD_NRF6310

# application source files
C_SOURCE_FILES += main.c

C_SOURCE_FILES += ble_ans_c.c

C_SOURCE_FILES += ble_error_log.c
C_SOURCE_FILES += softdevice_handler.c
C_SOURCE_FILES += ble_advdata.c
C_SOURCE_FILES += ble_conn_params.c
C_SOURCE_FILES += ble_debug_assert_handler.c
C_SOURCE_FILES += app_timer.c
C_SOURCE_FILES += pstorage.c
C_SOURCE_FILES += crc16.c
C_SOURCE_FILES += app_gpiote.c
C_SOURCE_FILES += app_button.c
C_SOURCE_FILES += simple_uart.c
C_SOURCE_FILES += app_trace.c
C_SOURCE_FILES += device_manager_peripheral.c

SDK_PATH = ../../../../../

OUTPUT_FILENAME := ble_app_alert_notification

DEVICE_VARIANT := xxaa
#DEVICE_VARIANT := xxab

USE_SOFTDEVICE := S110
#USE_SOFTDEVICE := S210

CFLAGS := -DDEBUG_NRF_USER -DBLE_STACK_SUPPORT_REQD

# we do not use heap in this app
ASMFLAGS := -D__HEAP_SIZE=0

# keep every function in separate section. This will allow linker to dump unused functions
CFLAGS += -ffunction-sections -fno-strict-aliasing

# let linker to dump unused sections
LDFLAGS := -Wl,--gc-sections

INCLUDEPATHS += -I"$(SDK_PATH)Include/s110"
INCLUDEPATHS += -I"$(SDK_PATH)Include/ble"
INCLUDEPATHS += -I"$(SDK_PATH)Include/ble/device_manager"
INCLUDEPATHS += -I"$(SDK_PATH)Include/ble/ble_services"
INCLUDEPATHS += -I"$(SDK_PATH)Include/app_common"
INCLUDEPATHS += -I"$(SDK_PATH)Include/sd_common"
INCLUDEPATHS += -I"$(SDK_PATH)Include/sdk"

C_SOURCE_PATHS += $(SDK_PATH)Source/ble
C_SOURCE_PATHS += $(SDK_PATH)Source/ble/device_manager
C_SOURCE_PATHS += $(SDK_PATH)Source/app_common
C_SOURCE_PATHS += $(SDK_PATH)Source/sd_common

include $(SDK_PATH)Source/templates/gcc/Makefile.common
