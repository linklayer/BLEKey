/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
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

/**@file
 * @defgroup spi_master_example_with_slave_main main.c
 * @{
 * @ingroup spi_master_example
 *
 * @brief SPI master example application to be used with the SPI slave example application.
 */

#include <stdio.h>
#include <stdbool.h>
#include "app_error.h"
#include "app_util_platform.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "spi_master.h"
#include "boards.h"

/*
 * This example uses only one instance of the SPI master.
 * Please make sure that only one instance of the SPI master is enabled in config file.
 */
 
#if defined(SPI_MASTER_0_ENABLE)
    #define SPI_MASTER_HW SPI_MASTER_0
#elif defined(SPI_MASTER_1_ENABLE)
    #define SPI_MASTER_HW SPI_MASTER_1
#endif

#define TX_RX_BUF_LENGTH    16u     /**< SPI transaction buffer length. */
#define DELAY_MS            100u    /**< Timer delay in milliseconds. */

//Data buffers.
static uint8_t m_tx_data[TX_RX_BUF_LENGTH] = {0}; /**< A buffer with data to transfer. */
static uint8_t m_rx_data[TX_RX_BUF_LENGTH] = {0}; /**< A buffer for incoming data. */

static volatile bool m_transfer_completed = true; /**< A flag to inform about completed transfer. */

/**@brief Function for error handling, which is called when an error has occurred. 
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the handler is called.
 * @param[in] p_file_name Pointer to the file name. 
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    //Set LED2 high to indicate that error has occurred.
    nrf_gpio_pin_set(LED_2);
    
    for (;;)
    {
        //No implementation needed.
    }
}

/**@brief Function for checking if data coming from a SPI slave are valid.
 *
 * @param[in] p_buf     A pointer to a data buffer.
 * @param[in] len       A length of the data buffer.
 * 
 * @note Expected ASCII characters from: 'a' to: ('a' + len - 1).
 *
 * @retval true     Data are valid.
 * @retval false    Data are invalid.
 */
static __INLINE bool buf_check(uint8_t * p_buf, uint16_t len)
{
    uint16_t i;
    for (i = 0; i < len; i++)
    {
        if (p_buf[i] != (uint8_t)('a' + i))
        {
            return false;
        }
    }
    
    return true;
}

/** @brief Function for initializing a SPI master driver.
 */
static uint32_t spi_master_init(void)
{
    spi_master_config_t spi_config = SPI_MASTER_INIT_DEFAULT;
    
    #if defined(SPI_MASTER_0_ENABLE)
        spi_config.SPI_Pin_SCK = SPIM0_SCK_PIN;
        spi_config.SPI_Pin_MISO = SPIM0_MISO_PIN;
        spi_config.SPI_Pin_MOSI = SPIM0_MOSI_PIN;
        spi_config.SPI_Pin_SS = SPIM0_SS_PIN;
    #elif defined(SPI_MASTER_1_ENABLE)
        spi_config.SPI_Pin_SCK = SPIM1_SCK_PIN;
        spi_config.SPI_Pin_MISO = SPIM1_MISO_PIN;
        spi_config.SPI_Pin_MOSI = SPIM1_MOSI_PIN;
        spi_config.SPI_Pin_SS = SPIM1_SS_PIN;
    #endif /* SPI_MASTER_ENABLE */
    
    return spi_master_open(SPI_MASTER_HW, &spi_config);
}

/**@brief Function for SPI master event callback.
 *
 * Upon receiving an SPI transaction complete event, checks if received data are valid.
 *
 * @param[in] spi_master_evt    SPI master driver event.
 */
static void spi_master_event_handler(spi_master_evt_t spi_master_evt)
{
    bool result = false;
    switch (spi_master_evt.evt_type)
    {
        case SPI_MASTER_EVT_TRANSFER_COMPLETED:
            
            //Check if data are vaild.
            result = buf_check(m_rx_data, spi_master_evt.data_count);
            APP_ERROR_CHECK_BOOL(result);
        
            //Inform application that transfer is completed.
            m_transfer_completed = true;
            break;
        
        default:
            //No implementation needed.
            break;
    }
}

/**@brief The function initializes TX buffer to values to be sent and clears RX buffer.
 *
 * @note Function initializes TX buffer to values from 'A' to ('A' + len - 1)
 *       and clears RX buffer (fill by 0).
 *
 * @param[in] p_tx_data     A pointer to a buffer TX.
 * @param[in] p_rx_data     A pointer to a buffer RX.
 * @param[in] len           A length of the data buffers.
 */
static void init_buffers(uint8_t * const p_tx_data,
                         uint8_t * const p_rx_data,
                         const uint16_t len)
{
    uint16_t i;
    for (i = 0; i < len; i++)
    {
        p_tx_data[i] = (uint8_t)('A' + i);
        p_rx_data[i] = 0;
    }
}

/**@brief Functions prepares buffers and starts data transfer.
 *
 * @param[in] p_tx_data     A pointer to a buffer TX.
 * @param[in] p_rx_data     A pointer to a buffer RX.
 * @param[in] len           A length of the data buffers.
 */
static void spi_send_recv(uint8_t * const p_tx_data,
                          uint8_t * const p_rx_data,
                          const uint16_t len)
{
    //Initalize buffers.
    init_buffers(p_tx_data, p_rx_data, len);
    
    //Start transfer.
    uint32_t err_code = spi_master_send_recv(SPI_MASTER_HW, p_tx_data, len, p_rx_data, len);
    APP_ERROR_CHECK(err_code);
    
    nrf_gpio_pin_toggle(LED_1);
    nrf_delay_ms(DELAY_MS);
}

/**@brief Function for application main entry. Does not return. */
int main(void)
{
    //Configure all LEDs as outputs. 
    nrf_gpio_range_cfg_output(LED_START, LED_STOP);
        
    //Set LED_0 high to indicate that the application is running. 
    nrf_gpio_pin_set(LED_0);
    
    //Initialize SPI master.
    uint32_t err_code = spi_master_init();
    APP_ERROR_CHECK(err_code);
    
    //Register SPI master event handler.
    spi_master_evt_handler_reg(SPI_MASTER_HW, spi_master_event_handler);
    
    for (;;)
    {
        if (m_transfer_completed)
        {
            m_transfer_completed = false;
            
            //Set buffers and start data transfer.
            spi_send_recv(m_tx_data, m_rx_data, TX_RX_BUF_LENGTH);
        }
    }
}

/** @} */
