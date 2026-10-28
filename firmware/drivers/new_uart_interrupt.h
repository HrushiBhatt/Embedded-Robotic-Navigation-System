/**
 * @file    new_uart_interrupt.h
 * @brief   Interrupt-driven UART1 driver (115200 8-N-1) used for robot <-> client telemetry.
 */

#ifndef UART_INTERRUPT_H_
#define UART_INTERRUPT_H_

#include <stdint.h>

/// Most recent byte received on UART1 (written by UART1_Handler, consumed by main()).
extern volatile char received_char;

/// Configure UART1 on PB0/PB1 at 115200 baud with RX interrupts enabled.
void uart_interrupt_init(void);

/// Send a single byte (blocks while the TX FIFO is full).
void uart_sendChar(char data);

/// Send a NUL-terminated string.
void uart_sendStr(const char *data);

/// UART1 RX interrupt service routine.
void UART1_Handler(void);

#endif /* UART_INTERRUPT_H_ */
