/* Copyright (c) 2013 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#include "bootloader_util.h"
#include <stdint.h>
#include "nordic_common.h"
#include "bootloader_types.h"
#include "dfu_types.h"

#if __GNUC__ && __ARM_EABI__

__attribute__ ((section(".bootloaderSettings"))) uint8_t m_boot_settings[1024] ;
__attribute__ ((section(".uicrBootStartAddress"))) uint32_t m_uicr_bootloader_start_address = BOOTLOADER_REGION_START;
const bootloader_settings_t const * const mp_bootloader_settings = (bootloader_settings_t *) &m_boot_settings[0];   /**< Read only pointer to bootloader settings in flash. */


static inline void StartApplication(uint32_t start_addr)
{
    __asm volatile("LDR   R2, [R0]\t\n"
          "MSR   MSP, R2\t\n"
          "LDR   R3, [R0, #0x00000004]\t\n"
          "BX    R3\t\n"
          ".ALIGN\t\n");
}
#endif


void bootloader_util_app_start(uint32_t start_addr)
{
    StartApplication(start_addr);
}


void bootloader_util_settings_get(const bootloader_settings_t ** pp_bootloader_settings)
{
    *pp_bootloader_settings = mp_bootloader_settings;
}
