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
#ifndef LIB_DEBOUNCE_H
#define LIB_DEBOUNCE_H

#include <stdint.h>

/** @brief Debouncer state */
typedef struct
{
  uint_fast8_t integrator_state;            /**< Integrator state. */
  uint_fast8_t output_state;                /**< Output state. */
  uint_fast8_t debounce_time_in_ms;         /**< How long a button is debounced in ms. */
  uint_fast8_t input_sampling_freq_in_hz;   /**< Debouncer button sampling frequency in Hertz. */
  uint_fast8_t integrator_max_value;        /**< Integrator max value. */
} deb_t;

/**@brief Function for Debouncer initializartion
 * @param[in] debounce_time_in_ms           Period of debounce time in milliseconds.
 * @param[in] input_sampling_freq_in_hz     Button input sampling frequency in hertz.
 * @param[out] deb                          Struct to be initialized.
 */
void debounce_init(deb_t *deb, uint_fast8_t debounce_time_in_ms, \
                   uint_fast8_t input_sampling_freq_in_hz);

/**@brief Function for handling Debouncer button
 * @param[in]   input   Input sample.
 * @param[out]  deb     Debounce struct to be updated.
 */
void debounce(uint_fast8_t input, deb_t *deb);

#endif  // LIB_DEBOUNCE_H
