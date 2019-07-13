#include "spi.h"

void SPI1_Init(void){

    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN | RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN;
    // PA4, NSS,  cnf = AFPP, mode = 50MHz
    // PA5, SCK,  cnf = AFPP, mode = 50MHz
    // PA6, MISO, cnf = input floating, mode = input
    // PA7, MOSI, cnf = AFPP, mode = 50MHz
    pin_mode(GPIOA, 4, GPIO_OUT_PP);
    pin_mode(GPIOA, 5, GPIO_AF_PP);
    pin_mode(GPIOA, 6, GPIO_IN_FL);
    pin_mode(GPIOA, 7, GPIO_AF_PP);

    SPI1->CR1 =   0b0000100100111101;
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

    // enable SPI1
    SPI1->CR1  |= SPI_CR1_SPE;
}

void SPI1_Transfer(uint16_t data){
    // Wait until SPI is not busy anymore
    while (SPI1->SR & (SPI_SR_BSY)){}

    pin_reset(GPIOA, SPI_CS_PIN);
    // Write data to be transmitted to the SPI data register
	SPI1->DR = data;

	while (!(SPI1->SR & (SPI_SR_TXE))){}
    while (SPI1->SR & (SPI_SR_BSY)){}
    pin_set(GPIOA, SPI_CS_PIN);
    SPI1->DR;
}

// uint16_t SPI1_Receive(void){}

uint16_t SPI1_Transcieve(uint16_t TxData){
    uint16_t RxData = 0;
    // Wait until SPI is not busy anymore
    SPI1_Transfer(TxData);

    pin_reset(GPIOA, SPI_CS_PIN);
    uint16_t timeout = 10000;
    while(!(SPI1->SR & SPI_SR_RXNE)){
        if(timeout == 0){
            break;
            printf("break\n");
        }
        timeout--;
    }
    
    RxData = SPI1->DR;

    while (SPI1->SR & (SPI_SR_BSY)){}
    pin_set(GPIOA, SPI_CS_PIN);
    // printf("SPI1->SR\t");  print_reg(SPI1->SR, 16);

    return RxData;
}
