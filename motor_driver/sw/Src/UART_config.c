#include "UART_config.h"


void setup_uart1(void){
    // TX PA9
    // RX PA10
}

void setup_uart2(void){
    // TX PA2
    // RX PA3

}

void setup_uart3(void){
    // enable RCC for USART3
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
    // enable RCC for GPIO port of USART3 (GPIOB)
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    // enable RCC for alternate function clocks
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;

    // setup GPIO pins for USART
    // TX PB10
    // RX PB11
    GPIOB->CRH |=   ( GPIO_CRH_MODE10_0  // pin 10, alternate function
                    | GPIO_CRH_MODE10_1  // TX
                    // | GPIO_CRH_CNF10_0
                    | GPIO_CRH_CNF10_1

                    | GPIO_CRH_MODE11_0  // pin 11, alternate function
                    | GPIO_CRH_MODE11_1  // RX
                    // | GPIO_CRH_CNF11_0
                    | GPIO_CRH_CNF11_1 );

    USART3->BRR;
    USART3->CR1;
    USART3->CR2;
    USART3->CR3;


}

void transmit_uart1(void){
    USART1->DR = (uint8_t)0x61;
}

void transmit_uart2(void){
    USART2->DR = (uint8_t)0x61;
}

void transmit_uart3(void){
    USART3->DR = (uint8_t)0x61;
}
