#ifndef UART_H
#define UART_H

#include <stddef.h>
#include <stdint.h>

/* USART3 on PB10/PB11, interrupt-driven ring buffers. */
void uart_init(void);
/* Next received byte, or -1. */
int uart_getc(void);
/* Queue bytes for transmit. Drops what doesn't fit (never blocks). */
void uart_write(const char *s, size_t n);
void uart_puts(const char *s);
uint32_t uart_dropped(void);

#endif
