/**
 * @file    new_uart_interrupt.c
 * @brief   Interrupt-driven UART1 driver (PB0 = RX, PB1 = TX) at 115200 baud, 8-N-1.
 *
 * UART1 connects the TM4C123 to the CyBot Wi-Fi module, which bridges the
 * serial link to a TCP socket for the Python clients.
 *
 *   RX: interrupt-driven. UART1_Handler() stores the latest byte in
 *       `received_char`, which the main loop polls as a command.
 *   TX: polled. uart_sendChar() waits for room in the TX FIFO.
 */

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "new_uart_interrupt.h"

volatile char received_char;    // Latest byte received; written by the ISR, read by main()

void uart_interrupt_init(void) {

    /* ---- Clock gating: GPIO Port B + UART1, then wait until ready ---- */
    SYSCTL_RCGCGPIO_R |= 0x02;
    SYSCTL_RCGCUART_R |= 0x02;

    while ((SYSCTL_PRGPIO_R & 0x02) == 0) {};
    while ((SYSCTL_PRUART_R & 0x02) == 0) {};

    /* ---- Pin mux: PB0/PB1 -> U1RX/U1TX ---- */
    GPIO_PORTB_DEN_R |= 0x03;
    GPIO_PORTB_AFSEL_R |= 0x03;
    GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R & 0xFFFFFF00)  + 0x00000011;

    /* ---- Baud rate: 16 MHz / (16 * 115200) = 8.6806 -> IBRD = 8, FBRD = round(0.6806 * 64) = 44 ---- */
    uint16_t iBRD = 8;
    uint16_t fBRD = 44;

    UART1_CTL_R &= ~0x01;       // Disable UART1 while configuring
    UART1_IBRD_R = iBRD;
    UART1_FBRD_R = fBRD;
    UART1_LCRH_R = 0x60;        // 8 data bits, no parity, 1 stop bit, FIFOs off
    UART1_CC_R = 0x0;           // Clock source = system clock


    /* ---- Interrupts: RX only, NVIC priority 1, IRQ 6 ---- */
    UART1_ICR_R |= 0b00010000;  // Clear any pending RX interrupt
    UART1_IM_R |= 0b00010000;   // Unmask the RX interrupt
    NVIC_PRI1_R = (NVIC_PRI1_R & 0xFF0FFFFF) | 0x00200000;
    NVIC_EN0_R |= 0x40;         // Enable IRQ 6 (UART1) in the NVIC

    IntRegister(INT_UART1, UART1_Handler);   // Bind the ISR in the vector table
    IntMasterEnable();

    UART1_CTL_R = 0x301;        // Enable UART1 with RXE | TXE | UARTEN

}

/*
 * Alternative version of uart_interrupt_init() written with the named
 * TivaWare bit-field macros instead of raw hex values. Kept for reference.
 */
/*
void uart_interrupt_init(void) {
    // Enable UART1 and GPIOB clocks
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;

    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R1) == 0) {};
    while ((SYSCTL_PRUART_R & SYSCTL_PRUART_R1) == 0) {};

    // Configure PB0 and PB1 for UART1 RX/TX
    GPIO_PORTB_AFSEL_R |= 0x03;
    GPIO_PORTB_PCTL_R |= (GPIO_PCTL_PB0_U1RX | GPIO_PCTL_PB1_U1TX);
    GPIO_PORTB_DEN_R |= 0x03;

    // UART1 configuration
    UART1_CTL_R &= ~UART_CTL_UARTEN;        // Disable UART
    UART1_IBRD_R = 8;                       // Integer Baud Rate Divisor
    UART1_FBRD_R = 44;                      // Fractional Baud Rate Divisor
    UART1_LCRH_R = UART_LCRH_WLEN_8;        // 8-bit word length
    UART1_CC_R = 0x0;                       // Use system clock
    UART1_IM_R |= UART_IM_RXIM;            // Enable RX interrupt
    NVIC_PRI1_R = (NVIC_PRI1_R & 0xFF0FFFFF) | 0x00200000; // Set interrupt priority
    NVIC_EN0_R |= (1 << 6);                // Enable IRQ 6 (UART1)

    UART1_CTL_R |= (UART_CTL_UARTEN | UART_CTL_RXE | UART_CTL_TXE);  // Enable UART
}
*/

// UART1 interrupt service routine: latch the received byte and acknowledge the interrupt.
void UART1_Handler(void) {
    if (UART1_MIS_R & UART_MIS_RXMIS) {
        received_char = UART1_DR_R & 0xFF;      // Read received byte
        UART1_ICR_R |= UART_ICR_RXIC;           // Clear RX interrupt
    }
}

// Transmit one byte (blocking).
void uart_sendChar(char data) {
    while ((UART1_FR_R & UART_FR_TXFF));       // Wait if TX FIFO is full
    UART1_DR_R = data;
}

// Transmit a NUL-terminated string (blocking).
void uart_sendStr(const char *data) {
    while (*data) {
        uart_sendChar(*data++);
    }
}
