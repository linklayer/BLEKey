TARGET_CHIP := NRF51822_QFAA_CA
BOARD := BOARD_NRF6310

# application source
C_SOURCE_FILES += main.c

C_SOURCE_FILES += ble_dis.c
C_SOURCE_FILES += ble_bas.c
C_SOURCE_FILES += ble_wiegand.c
C_SOURCE_FILES += wiegand.c
C_SOURCE_FILES += retarget.c

C_SOURCE_FILES += ble_srv_common.c
C_SOURCE_FILES += ble_sensorsim.c
C_SOURCE_FILES += softdevice_handler.c
C_SOURCE_FILES += ble_advdata.c
C_SOURCE_FILES += ble_debug_assert_handler.c
C_SOURCE_FILES += ble_error_log.c
C_SOURCE_FILES += ble_conn_params.c
C_SOURCE_FILES += app_timer.c
C_SOURCE_FILES += pstorage.c
C_SOURCE_FILES += crc16.c
C_SOURCE_FILES += device_manager_peripheral.c
C_SOURCE_FILES += app_trace.c
C_SOURCE_FILES += simple_uart.c

SDK_PATH = ../nordic/nrf51822/

OUTPUT_FILENAME := blekey

DEVICE_VARIANT := xxaa
#DEVICE_VARIANT := xxab

USE_SOFTDEVICE := S110
#USE_SOFTDEVICE := S210

CFLAGS := -DDEBUG_NRF_USER -DBLE_STACK_SUPPORT_REQD -DS110

# we do not use heap in this app
ASMFLAGS := -D__HEAP_SIZE=0

# keep every function in separate section. This will allow linker to dump unused functions
CFLAGS += -ffunction-sections -g

# let linker to dump unused sections
#LDFLAGS := -Wl,--gc-sections

INCLUDEPATHS += -I"$(SDK_PATH)Include/s110"
INCLUDEPATHS += -I"$(SDK_PATH)Include/ble"
INCLUDEPATHS += -I"$(SDK_PATH)Include/ble/device_manager"
INCLUDEPATHS += -I"$(SDK_PATH)Include/ble/ble_services"
INCLUDEPATHS += -I"$(SDK_PATH)Include/app_common"
INCLUDEPATHS += -I"$(SDK_PATH)Include/sd_common"
INCLUDEPATHS += -I"$(SDK_PATH)Include/sdk"
INCLUDEPATHS += -I"$(SDK_PATH)Include/bootloader_dfu"

C_SOURCE_PATHS += $(SDK_PATH)Source/ble
C_SOURCE_PATHS += $(SDK_PATH)Source/ble/device_manager
C_SOURCE_PATHS += $(SDK_PATH)Source/app_common
C_SOURCE_PATHS += $(SDK_PATH)Source/sd_common

include $(SDK_PATH)Source/templates/gcc/Makefile.common

#
# Stuff for flashing
#

GDB_PORT_NUMBER := 9992

JLINK_PATH = /home/blark/Downloads/JLink_Linux_V500c_x86_64
JLINK_OPTS = -device nrf51822 -if swd -speed 4000
JLINK_GDB_OPTS = -noir
JLINK = $(JLINK_PATH)/JLinkExe $(JLINK_OPTS)
JLINKD_GDB = JLinkGDBServer $(JLINK_GDB_OPTS)


flash: flash.jlink
	$(JLINK) flash.jlink

flash.jlink:
	printf "loadbin $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin 0x16000\nr\ng\nexit\n" > flash.jlink

erase-all: erase-all.jlink
	$(JLINK) erase-all.jlink

erase-all.jlink:
	# Write to NVMC to enable erase, do erase all, wait for completion. reset
	printf "w4 4001e504 2\nw4 4001e50c 1\nsleep 100\nr\nexit\n" > erase-all.jlink

run-debug:
	$(JLINKD_GDB) $(JLINK_OPTS) $(JLINK_GDB_OPTS) -port $(GDB_PORT_NUMBER)

.PHONY:  flash-jlink flash.jlink erase-all erase-all.jlink run-debug
