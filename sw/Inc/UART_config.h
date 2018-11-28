#ifndef __UART_CONFIG_H__
#define __UART_CONFIG_H__

#include "stm32f103xb.h"
#include <stdio.h>

void UART1_setup(void);
void UART2_setup(void);
void UART3_setup(void);

void transmit_uart1(char *ch);
void transmit_uart2(char *ch);
void transmit_uart3(char *ch);

void print_reg(uint32_t reg, uint8_t reg_sz);

#endif // endif __UART_CONFIG_H__
