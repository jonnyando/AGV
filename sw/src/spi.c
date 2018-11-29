#include "spi.h"

void SPI1_setup(void){

    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN | RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN;
    // pin 4,5,6,7
    GPIOA->CRL &=   0x0000FFFF; // clear bits we want to set
    GPIOA->CRL |=   0b10100100101000110000000000000000;
    // PA4, NSS,  cnf = AFPP, mode = 50MHz
    // PA5, SCK,  cnf = AFPP, mode = 50MHz
    // PA6, MISO, cnf = input floating, mode = input
    // PA7, MOSI, cnf = AFPP, mode = 50MHz

    SPI1->CR1 =   0b0000100100011101;
    // BIDIMODE    0   2-line unidirectional mode
    // BIDIOE      x   not one line bidirectional
    // CRCEN       0   CRC disabled
    // CRCNEXT     0   /X. No CRC
    // DFF         1   16 bit mode
    // RXONLY      0   transmit and receive
    // SSM         0   software slave management
    // SSI         1   slave select high
    // LSBFIRST    0   MSB first
    // SPE         0   SPI NOT enabled (enabled below)
    // BR[2:0]     011 fpclk/16    arbitrary setting
    // MSTR        1   Master configuration
    // CPOL        0   low when idle
    // CPHA        1   second edge

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
    SPI1->CR1  |= SPI_CR1_SPE;
}

void SPI1_Transfer(uint16_t data){
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

// uint16_t SPI1_Receive(void){}

uint16_t SPI1_Transcieve(uint16_t TxData){
    uint16_t RxData = 0;
    // Wait until SPI is not busy anymore
    while (SPI1->SR & (SPI_SR_BSY)){
        // printf("stuck (bsy) %d\n", (SPI1->SR));
    }

    //SPI_CS_PORT->BRR |= SPI_CS_PIN;
    // Write data to be transmitted to the SPI data register
	SPI1->DR = TxData;

	// Wait until transmit complete
	while (!(SPI1->SR & (SPI_SR_TXE))){
		// printf("stuck (txe) %d\n", (SPI1->SR));
	}
    //SPI_CS_PORT->ODR |= SPI_CS_PIN;
    //HAL_Delay(1);
    //SPI_CS_PORT->BRR |= SPI_CS_PIN;
    while(!(SPI1->SR & SPI_SR_RXNE)){
        // printf("received data");
    }
    RxData = SPI1->DR;
    //SPI_CS_PORT->ODR |= SPI_CS_PIN;
    return RxData;
}
