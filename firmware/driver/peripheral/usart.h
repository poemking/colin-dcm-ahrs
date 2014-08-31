#ifndef __USART_H
#define __USART_H

void usart3_init(int baudrate);
void usart3_putc(uint8_t data);
void usart3_puts(uint8_t *str);
uint8_t usart3_getc();
int printf(const char *format, ...);

#endif
