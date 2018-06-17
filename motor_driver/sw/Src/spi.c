#include "spi.h"


void test_func(void){
    printf("test\n");
}

void SPI1_setup(void){

    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN | RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN;
    // pin 4,5,6,7
    GPIOA->CRL &=   0x0000FFFF; // clear bits we want to set
    GPIOA->CRL |=   0b10110100101110110000000000000000;
    // PA4, NSS,  cnf = AFPP, mode = 50MHz
    // PA5, SCK,  cnf = AFPP, mode = 50MHz
    // PA6, MISO, cnf = input floating, mode = input
    // PA7, MOSI, cnf = AFPP, mode = 50MHz

    SPI1->CR1 =   0b0000101100011100;
    // BIDIMODE    0   2-line unidirectional mode
    // BIDIOE      x   not one line bidirectional
    // CRCEN       0   CRC disabled
    // CRCNEXT     0   /X. No CRC
    // DFF         1   16 bit mode
    // RXONLY      0   transmit and receive
    // SSM         1   software slave management
    // SSI         1   slave select active
    // LSBFIRST    0   MSB first
    // SPE         0   SPI NOT enabled (enabled below)
    // BR[2:0]     011 fpclk/16    arbitrary setting
    // MSTR        1   Master configuration
    // CPOL        0   low when idle
    // CPHA        0   rising (first) edge

    SPI1->CR2      = 0b0000000000000100;
    // TXEIE       0   Tx buffer interupt not enabled
    // RXEIE       0   Rx buffer interupt not enabled
    // ERRIE       0   error interupt not enabled
    // SSOE        1   SS output is enabled
    // TXDMAEN     0   Tx buffer DMA disabled
    // RXDMAEN     0   Rx buffer DMA disabled
    // CR2 = xxxxxxxx000xx100

    SPI1->I2SCFGR  = 0x0000;
    // I2SMOD      0   SPI mode selected
    // I2SCFGR = xxxx0xxxxxxxxxxx

    HAL_Delay(10);

    // enable SPI1
    SPI1->CR1      |= SPI_CR1_SPE;
}

void SPI1_Transfer(uint16_t data)
{
    // Wait until SPI is not busy anymore
    while (SPI1->SR & (SPI_SR_BSY)){
        // printf("stuck (bsy) %d\n", (SPI1->SR));
    }

    // Write data to be transmitted to the SPI data register
	SPI1->DR = data;

	// Wait until transmit complete
	while (!(SPI1->SR & (SPI_SR_TXE))){
		// printf("stuck (txe) %d\n", (SPI1->SR));
	}
}

/*      SPI REGISTERS
---CR1---
BIDIMODE    0   2-line unidirectional mode
BIDIOE      x   not one line bidirectional
CRCEN       0   CRC disabled
CRCNEXT     0   /X. No CRC
DFF         1   16 bit mode
RXONLY      0   transmit and receive
SSM         1   software slave management
SSI         1   slave select active
LSBFIRST    0   MSB first
SPE         1   SPI enabled
BR[2:0]     011 fpclk/16    arbitrary setting
MSTR        1   Master configuration
CPOL        0   low when idle
CPHA        0   rising (first) edge
result:
CR1 = 0b0000101101011100
    = 0x0D5C
---CR2---
TXEIE       0   Tx buffer interupt not enabled
RXEIE       0   Rx buffer interupt not enabled
ERRIE       0   error interupt not enabled
SSOE        1   SS output is enabled
TXDMAEN     0   Tx buffer DMA disabled
RXDMAEN     0   Rx buffer DMA disabled
result:
CR2 = xxxxxxxx000xx100
    = 0000000000000100
    = 0x0004
---SR---
N/A
---DR---
N/A
---CRCPR---
N/A
---RXCRCR---
N/A
---TXCRCR---
N/A
---I2SCFGR---
I2SMOD      0   SPI mode selected
result:
I2SCFGR = xxxx0xxxxxxxxxxx
        == 0x0000
*/

// 
// void print_reg(UART_HandleTypeDef *huart, uint32_t reg, uint8_t reg_sz){
//     for (uint8_t i = reg_sz; i > 0; i--) {
//         if (reg & (1<<(i-1))) {
//             HAL_UART_Transmit(huart, "1", 1, 100);
//         } else {
//             HAL_UART_Transmit(huart, "0", 1, 100);
//         }
//     }
//     HAL_UART_Transmit(huart, "\n", 1, 100);
// }
