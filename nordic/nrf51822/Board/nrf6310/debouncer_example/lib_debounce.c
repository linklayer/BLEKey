/* Copyright (c) 2009 Nordic Semiconductor. All Rights Reserved.
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
#include "lib_debounce.h"

void debounce_init(deb_t *deb, uint_fast8_t debounce_time_in_ms, 
                   uint_fast8_t input_sampling_freq_in_hz)
{
    deb->integrator_state          = 0U;
    deb->output_state              = 0U;
    deb->debounce_time_in_ms       = debounce_time_in_ms;
    deb->input_sampling_freq_in_hz = input_sampling_freq_in_hz;
    deb->integrator_max_value      = (uint_fast8_t)((uint_least16_t)debounce_time_in_ms * \
                                     (uint_least16_t)input_sampling_freq_in_hz / 1000U);
}

void debounce(uint_fast8_t input, deb_t *deb)
{
    if (input == 0)
    {
        if (deb->integrator_state > 0)
        {
            deb->integrator_state--;
        }
    }
    else if (deb->integrator_state < deb->integrator_max_value)
    {
        deb->integrator_state++;
    }

    if (deb->integrator_state == 0)
    {
        deb->output_state = 0;
    }
    else if (deb->integrator_state >= deb->integrator_max_value)
    {
        deb->output_state = 1;
        deb->integrator_state = deb->integrator_max_value;
    }
}
