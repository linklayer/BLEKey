/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
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

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdarg.h>
#include "app_uart.h"
#include "simple_uart.h"
#include "app_gpiote.h"
#include "nordic_common.h"

#include "boards.h"
#include "app_trace.h"

struct __FILE { int handle; /* Add whatever you need here */ };
FILE __stdout;
FILE __stdin;

#ifdef ENABLE_DEBUG_LOG_SUPPORT

void app_trace_init(void)
{
    simple_uart_config(RTS_PIN_NUMBER, TX_PIN_NUMBER, CTS_PIN_NUMBER, RX_PIN_NUMBER, HWFC);
}

int fputc(int ch, FILE * p_file)
{
    simple_uart_put((uint8_t)ch);
    return ch;
}

void app_trace_dump(uint8_t * p_buffer, uint32_t len)
{
    app_trace_log("\r\n");
    for (uint32_t index = 0; index <  len; index++)
    {
        app_trace_log("0x%02X ", p_buffer[index]);
    }
    app_trace_log("\r\n");
}

#endif // ENABLE_DEBUG_LOG_SUPPORT

/**
 *@}
 **/

