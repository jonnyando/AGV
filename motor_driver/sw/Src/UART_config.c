#include "UART_config.h"


void setup_uart1(void){
    // TX PA9
    // RX PA10
}

void setup_uart2(void){

    // enable RCC for USART2
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    // enable RCC for GPIO port of USART2 (GPIOA)
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    // enable RCC for alternate function clocks
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;

    // setup GPIO pins for USART2
    // TX PA2
    // RX PA3
    GPIOA->CRL &=   0xffff00ff;
    GPIOA->CRL |=   0x00004B00;
    // ( GPIO_CRL_MODE2_0  // pin 10, alternate function
    //                 | GPIO_CRL_MODE2_1  // TX
    //                 // | GPIO_CRL_CNF2_0
    //                 | GPIO_CRL_CNF2_1
    //
    //                 | GPIO_CRL_MODE3_0  // pin 11, alternate function
    //                 | GPIO_CRL_MODE3_1  // RX
    //                 // | GPIO_CRL_CNF3_0
    //                 | GPIO_CRL_CNF3_1 );

    USART2->BRR = 0b0000000100010110;
    USART2->CR1 = 0b0010000000001100;
    USART2->CR2 = 0x0000;
    USART2->CR3 = 0x0000;


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
    GPIOB->CRH &=   0xffff00ff;
    GPIOB->CRH |=   ( GPIO_CRH_MODE10_0  // pin 10, alternate function
                    | GPIO_CRH_MODE10_1  // TX
                    // | GPIO_CRH_CNF10_0
                    | GPIO_CRH_CNF10_1

                    | GPIO_CRH_MODE11_0  // pin 11, alternate function
                    | GPIO_CRH_MODE11_1  // RX
                    // | GPIO_CRH_CNF11_0
                    | GPIO_CRH_CNF11_1 );

    USART3->BRR = 0b0000000100010110;
    USART3->CR1 = 0b0010000000001100;
    USART3->CR2 = 0x0000;
    USART3->CR3 = 0x0000;


}

void transmit_uart1(char *ch){
    int timeout = 1000;
    while(!(USART1->SR & USART_SR_TXE)){
        timeout--;
        if (timeout<=0){
            break;
        }
    }
    int i = 1;
    while(i > 0){
        i--;
        USART1->DR = (*ch++ & (uint8_t)0xFF);
    }
    timeout = 1000;
    while(!(USART1->SR & USART_SR_TC)){
        timeout--;
        if (timeout<=0){
            break;
        }
    }
}

void transmit_uart2(char *ch){
    // HAL_Delay(1);
    int timeout = 1000;
    while(!(USART2->SR & USART_SR_TXE)){
        timeout--;
        if (timeout<=0){
            break;
        }
    }
    int i = 1;
    while(i > 0){
        i--;
        USART2->DR = (*ch++ & (uint8_t)0xFF);
    }
    timeout = 1000;
    while(!(USART2->SR & USART_SR_TC)){
        timeout--;
        if (timeout<=0){
            break;
        }
    }
}

void transmit_uart3(char *ch){
    int timeout = 1000;
    while(!(USART3->SR & USART_SR_TXE)){
        timeout--;
        if (timeout<=0){
            break;
        }
    }
    int i = 1;
    while(i > 0){
        i--;
        USART3->DR = (*ch++ & (uint8_t)0xFF);
    }
    timeout = 1000;
    while(!(USART3->SR & USART_SR_TC)){
        timeout--;
        if (timeout<=0){
            break;
        }
    }
}


void print_reg(uint32_t reg, uint8_t reg_sz){
    for (uint8_t i = 0; i < reg_sz; i++) {
        uint8_t shift = reg_sz - 1 - i;
        if (reg & (1<<shift)) {
            printf("1");
            // HAL_UART_Transmit(huart, "1", 1, 100);
        } else {
            printf("0");
            // HAL_UART_Transmit(huart, "0", 1, 100);
        }
    }
    printf("\n");
    // HAL_UART_Transmit(huart, "\n", 1, 100);
}
