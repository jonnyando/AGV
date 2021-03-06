#ifndef __SPI_H__
#define __SPI_H__

#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <stdint.h>

#define SPI_TIMEOUT 1000
#define SPI_CS_PIN GPIO_PIN_4
#define SPI_CS_PORT GPIOA

void SPI1_setup(void);
void SPI1_Transfer(uint16_t data);
uint16_t SPI1_Transcieve(uint16_t TxData);
// void print_reg(UART_HandleTypeDef *huart, uint32_t reg, uint8_t reg_sz);


#endif // endif __SPI_H__
