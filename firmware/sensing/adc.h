/**
 * @file    adc.h
 * @brief   ADC0 driver for the IR distance sensor (PB4 / AIN10).
 */

#ifndef ADC_H
#define ADC_H

#include <stdint.h>

/// Configure PB4 as an analog input and ADC0 sample sequencer 3 (16x averaging).
void adc_init(void);

/// Perform one blocking conversion and return the raw 12-bit value (0-4095).
uint16_t adc_read(void);

#endif // ADC_H
