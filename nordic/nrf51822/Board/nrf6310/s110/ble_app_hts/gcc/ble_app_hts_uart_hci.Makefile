TARGET_CHIP := NRF51822_QFAA_CA
BOARD := BOARD_NRF6310

SDK_PATH = ../../../../../

OUTPUT_FILENAME := ble_app_hts_uart_hci

CFLAGS := -DDEBUG_NRF_USER -DBLE_STACK_SUPPORT_REQD -DSVCALL_AS_NORMAL_FUNCTION  

# keep every function in separate section. This will allow linker to dump unused functions
CFLAGS += -ffunction-sections

# let linker to dump unused sections
LDFLAGS := -Wl,--gc-sections

# we do not use heap in this app
ASMFLAGS := -D__HEAP_SIZE=0

DEVICE_VARIANT := xxaa
#DEVICE_VARIANT := xxab

#USE_SOFTDEVICE := S110
#USE_SOFTDEVICE := S210

INCLUDEPATHS += -I"$(SDK_PATH)Include/s110"
INCLUDEPATHS += -I"$(SDK_PATH)Include/ble"
INCLUDEPATHS += -I"$(SDK_PATH)Include/ble/device_manager"
INCLUDEPATHS += -I"$(SDK_PATH)Include/ble/ble_services"
INCLUDEPATHS += -I"$(SDK_PATH)Include/app_common"
INCLUDEPATHS += -I"$(SDK_PATH)Include/sdk_soc"
INCLUDEPATHS += -I"$(SDK_PATH)Include/sd_common"
INCLUDEPATHS += -I"$(SDK_PATH)Include/sdk"
INCLUDEPATHS += -I"$(SDK_PATH)Include/serialization/application/codecs/s110/serializers"
INCLUDEPATHS += -I"$(SDK_PATH)Include/serialization/application/transport"
INCLUDEPATHS += -I"$(SDK_PATH)Include/serialization/common"
INCLUDEPATHS += -I"$(SDK_PATH)Include/serialization/common/transport"
INCLUDEPATHS += -I"$(SDK_PATH)Include/serialization/common/struct_ser/s110"
INCLUDEPATHS += -I"$(SDK_PATH)Source/serialization/common/transport/debug"

C_SOURCE_PATHS += $(SDK_PATH)Source/ble
C_SOURCE_PATHS += $(SDK_PATH)Source/ble/device_manager
C_SOURCE_PATHS += $(SDK_PATH)Source/sd_common
C_SOURCE_PATHS += $(SDK_PATH)Source/spi_master
C_SOURCE_PATHS += $(SDK_PATH)Source/serialization/application/hal
C_SOURCE_PATHS += $(SDK_PATH)Source/serialization/application/codecs/common
C_SOURCE_PATHS += $(SDK_PATH)Source/serialization/application/codecs/s110/middleware
C_SOURCE_PATHS += $(SDK_PATH)Source/serialization/application/codecs/s110/serializers
C_SOURCE_PATHS += $(SDK_PATH)Source/serialization/application/transport
C_SOURCE_PATHS += $(SDK_PATH)Source/serialization/common
C_SOURCE_PATHS += $(SDK_PATH)Source/serialization/common/transport
C_SOURCE_PATHS += $(SDK_PATH)Source/serialization/common/struct_ser/s110
C_SOURCE_PATHS += $(SDK_PATH)Source/app_common
C_SOURCE_PATHS += $(SDK_PATH)Source/serialization/common/transport/debug

# application source
C_SOURCE_FILES += main.c

C_SOURCE_FILES += ble_hts.c
C_SOURCE_FILES += ble_bas.c
C_SOURCE_FILES += ble_dis.c

C_SOURCE_FILES += ble_advdata.c
C_SOURCE_FILES += ble_srv_common.c
C_SOURCE_FILES += ble_sensorsim.c
C_SOURCE_FILES += softdevice_handler.c
C_SOURCE_FILES += ble_debug_assert_handler.c
C_SOURCE_FILES += ble_error_log.c
C_SOURCE_FILES += ble_conn_params.c
C_SOURCE_FILES += pstorage.c
C_SOURCE_FILES += crc16.c
C_SOURCE_FILES += device_manager_peripheral.c
C_SOURCE_FILES += app_gpiote.c
C_SOURCE_FILES += app_button.c
C_SOURCE_FILES += app_timer.c

# ser_codecs
C_SOURCE_FILES += ble_event.c
C_SOURCE_FILES += ble_enable.c
C_SOURCE_FILES += ble_evt_tx_complete.c
C_SOURCE_FILES += ble_gap_address_get.c
C_SOURCE_FILES += ble_gap_address_set.c
C_SOURCE_FILES += ble_gap_adv_data_set.c
C_SOURCE_FILES += ble_gap_adv_start.c
C_SOURCE_FILES += ble_gap_adv_stop.c
C_SOURCE_FILES += ble_gap_appearance_get.c
C_SOURCE_FILES += ble_gap_appearance_set.c
C_SOURCE_FILES += ble_gap_auth_key_reply.c
C_SOURCE_FILES += ble_gap_authenticate.c
C_SOURCE_FILES += ble_gap_conn_param_update.c
C_SOURCE_FILES += ble_gap_conn_sec_get.c
C_SOURCE_FILES += ble_gap_device_name_get.c
C_SOURCE_FILES += ble_gap_device_name_set.c
C_SOURCE_FILES += ble_gap_disconnect.c
C_SOURCE_FILES += ble_gap_evt_auth_key_request.c
C_SOURCE_FILES += ble_gap_evt_auth_status.c
C_SOURCE_FILES += ble_gap_evt_conn_param_update.c
C_SOURCE_FILES += ble_gap_evt_conn_sec_update.c
C_SOURCE_FILES += ble_gap_evt_connected.c
C_SOURCE_FILES += ble_gap_evt_disconnected.c
C_SOURCE_FILES += ble_gap_evt_passkey_display.c
C_SOURCE_FILES += ble_gap_evt_rssi_changed.c
C_SOURCE_FILES += ble_gap_evt_sec_info_request.c
C_SOURCE_FILES += ble_gap_evt_sec_params_request.c
C_SOURCE_FILES += ble_gap_evt_timeout.c
C_SOURCE_FILES += ble_gap_ppcp_get.c
C_SOURCE_FILES += ble_gap_ppcp_set.c
C_SOURCE_FILES += ble_gap_rssi_start.c
C_SOURCE_FILES += ble_gap_rssi_stop.c
C_SOURCE_FILES += ble_gap_sec_info_reply.c
C_SOURCE_FILES += ble_gap_sec_params_reply.c
C_SOURCE_FILES += ble_gap_tx_power_set.c
C_SOURCE_FILES += ble_gattc_char_value_by_uuid_read.c
C_SOURCE_FILES += ble_gattc_char_values_read.c
C_SOURCE_FILES += ble_gattc_characteristics_discover.c
C_SOURCE_FILES += ble_gattc_descriptors_discover.c
C_SOURCE_FILES += ble_gattc_evt_char_disc_rsp.c
C_SOURCE_FILES += ble_gattc_evt_char_val_by_uuid_read_rsp.c
C_SOURCE_FILES += ble_gattc_evt_char_vals_read_rsp.c
C_SOURCE_FILES += ble_gattc_evt_desc_disc_rsp.c
C_SOURCE_FILES += ble_gattc_evt_hvx.c
C_SOURCE_FILES += ble_gattc_evt_prim_srvc_disc_rsp.c
C_SOURCE_FILES += ble_gattc_evt_read_rsp.c
C_SOURCE_FILES += ble_gattc_evt_rel_disc_rsp.c
C_SOURCE_FILES += ble_gattc_evt_timeout.c
C_SOURCE_FILES += ble_gattc_evt_write_rsp.c
C_SOURCE_FILES += ble_gattc_hv_confirm.c
C_SOURCE_FILES += ble_gattc_primary_services_discover.c
C_SOURCE_FILES += ble_gattc_read.c
C_SOURCE_FILES += ble_gattc_relationships_discover.c
C_SOURCE_FILES += ble_gattc_write.c
C_SOURCE_FILES += ble_gatts_characteristic_add.c
C_SOURCE_FILES += ble_gatts_descriptor_add.c
C_SOURCE_FILES += ble_gatts_evt_hvc.c
C_SOURCE_FILES += ble_gatts_evt_rw_authorize_request.c
C_SOURCE_FILES += ble_gatts_evt_sc_confirm.c
C_SOURCE_FILES += ble_gatts_evt_sys_attr_missing.c
C_SOURCE_FILES += ble_gatts_evt_timeout.c
C_SOURCE_FILES += ble_gatts_evt_write.c
C_SOURCE_FILES += ble_gatts_hvx.c
C_SOURCE_FILES += ble_gatts_include_add.c
C_SOURCE_FILES += ble_gatts_rw_authorize_reply.c
C_SOURCE_FILES += ble_gatts_service_add.c
C_SOURCE_FILES += ble_gatts_service_changed.c
C_SOURCE_FILES += ble_gatts_sys_attr_get.c
C_SOURCE_FILES += ble_gatts_sys_attr_set.c
C_SOURCE_FILES += ble_gatts_value_get.c
C_SOURCE_FILES += ble_gatts_value_set.c
C_SOURCE_FILES += ble_l2cap_cid_register.c
C_SOURCE_FILES += ble_l2cap_cid_unregister.c
C_SOURCE_FILES += ble_l2cap_evt_rx.c
C_SOURCE_FILES += ble_l2cap_tx.c
C_SOURCE_FILES += ble_tx_buffer_count_get.c
C_SOURCE_FILES += ble_uuid_decode.c
C_SOURCE_FILES += ble_uuid_encode.c
C_SOURCE_FILES += ble_uuid_vs_add.c
C_SOURCE_FILES += ble_version_get.c
C_SOURCE_FILES += power_system_off.c
C_SOURCE_FILES += temp_get.c
C_SOURCE_FILES += ble_gap_struct_serialization.c
C_SOURCE_FILES += ble_gattc_struct_serialization.c
C_SOURCE_FILES += ble_gatts_struct_serialization.c
C_SOURCE_FILES += ble_struct_serialization.c
C_SOURCE_FILES += ble_serialization.c
C_SOURCE_FILES += cond_field_serialization.c

# ser_codecs_mw
C_SOURCE_FILES += app_mw_ble.c
C_SOURCE_FILES += app_mw_ble_gap.c
C_SOURCE_FILES += app_mw_ble_gattc.c
C_SOURCE_FILES += app_mw_ble_gatts.c
C_SOURCE_FILES += app_mw_ble_l2cap.c
C_SOURCE_FILES += app_mw_nrf_soc.c

# ser_utils
C_SOURCE_FILES += ser_softdevice_handler.c
C_SOURCE_FILES += ser_sd_transport.c
C_SOURCE_FILES += ser_hal_transport.c
C_SOURCE_FILES += app_mailbox.c
C_SOURCE_FILES += app_util_platform.c
C_SOURCE_FILES += debug_hci_nrf6310.c

# ser_hal
C_SOURCE_FILES += nrf_delay.c
C_SOURCE_FILES += ser_app_hal_nrf51.c
C_SOURCE_FILES += ser_app_power_system_off.c

# ser_hal_serial
C_SOURCE_FILES += app_uart.c
C_SOURCE_FILES += ser_phy_hci.c
C_SOURCE_FILES += ser_phy_hci_slip.c


# file needed by pstorage.c Should be replaced on with target MCU flash programming functions
C_SOURCE_FILES += ble_flash.c

include $(SDK_PATH)Source/templates/gcc/Makefile.common
