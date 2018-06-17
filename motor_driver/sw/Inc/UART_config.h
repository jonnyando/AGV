#ifndef __UART_CONFIG__
#define __UART_CONFIG__

#include "stm32f103xb.h"

void setup_uart1(void);
void setup_uart2(void);
void setup_uart3(void);

void transmit_uart1(char *ch);
void transmit_uart2(char *ch);
void transmit_uart3(char *ch);

void print_reg(uint32_t reg, uint8_t reg_sz);

#endif // endif __UART_CONFIG__
