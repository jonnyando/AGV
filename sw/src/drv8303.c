#include "drv8303.h"
#include "spi.h"
#include "main.h"
#include "uart.h"
#include "gpio.h"


uint16_t drv_transceive(uint16_t tx_reg){
    //tx_reg = 0b1000100000000000;

    SPI1_Transfer(tx_reg);
    // printf("transmitting\t");  print_reg(tx_reg, 16);
    uint16_t rx_reg = SPI1_Transcieve(0b1000000000000000);

    return rx_reg;
}

uint16_t drv_write(uint8_t drv_reg, uint16_t payload){
    //tx_reg = 0b1000100000000000;
    payload &= 0b0000011111111111;
    uint16_t tx_reg = 0;
    tx_reg |= drv_reg << 11;
    // tx_reg |= (1<<15);
    tx_reg |= payload;

    printf("transmitting\t");  print_reg(tx_reg, 16);

    SPI1_Transfer(tx_reg);
    uint16_t rx_reg = SPI1_Transcieve(0b1000000000000000);

    return rx_reg;
}

uint16_t drv_read(uint8_t drv_reg){
    //tx_reg = 0b1000100000000000;
    uint16_t tx_reg = 0;
    tx_reg |= drv_reg << 11;
    tx_reg |= (1<<15);
    // tx_reg |= payload;

    // printf("reading\t");  print_reg(tx_reg, 16);

    SPI1_Transfer(tx_reg);
    uint16_t rx_reg = SPI1_Transcieve(0b1000000000000000);

    return rx_reg;
}