/**
 * @file    adc.c
 * @brief   Bare-metal ADC0 driver for the Sharp IR distance sensor on PB4 (AIN10).
 *
 * Uses sample sequencer 3 (single-sample FIFO), software-triggered, with
 * 16x hardware oversampling. adc_read() is a blocking single conversion.
 */

#include <stdint.h>
#include <inc/tm4c123gh6pm.h>
#include "adc.h"
#include "cyBot_Scan.h"

void adc_init(void)
{
    /* ---- Clock gating ---- */
    SYSCTL_RCGCGPIO_R |= 0x02; //Enable GPIO Port B //Right
    SYSCTL_RCGCADC_R |= 0x01; // Enable ADC0

    volatile int delay = SYSCTL_RCGCGPIO_R;   // Dummy read: give the peripheral clocks time to start

    /* ---- PB4 as analog input ---- */
    GPIO_PORTB_DIR_R &= ~0x10;   // PB4 as input
    GPIO_PORTB_AFSEL_R |= 0x10;  // Enable alternate function
    GPIO_PORTB_DEN_R &= ~0x10;   // Disable digital
    GPIO_PORTB_AMSEL_R |= 0x10;  // Enable analog

    /* ---- Sample sequencer 3 configuration ---- */
    ADC0_ACTSS_R &= ~0x08;  // Disable SS3 while configuring (clearing the bit)
    ADC0_EMUX_R &= ~0xF000; // SS3 trigger = processor (software)
    ADC0_SSMUX3_R = 10;     // Input channel AIN10 = PB4
    ADC0_SSCTL3_R = 0x06;   // Single sample: END0 | IE0 (raw interrupt flag on completion)
    ADC0_SAC_R = 0x04;      // 16x hardware sample averaging
    ADC0_ACTSS_R |= 0x08;   // Re-enable SS3 (setting the bit)


}

// Blocking single conversion: trigger SS3, wait for completion, return the 12-bit result.
uint16_t adc_read(void)
{
    ADC0_PSSI_R = 0x08;                         // Start a conversion on SS3
    while ((ADC0_RIS_R & 0x08) == 0);           // Wait for the SS3 raw interrupt flag
    uint16_t result = ADC0_SSFIFO3_R & 0xFFF;   // Read the 12-bit sample
    ADC0_ISC_R = 0x08;                          // Clear the completion flag
    return result;
}
