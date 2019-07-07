#include "tim.h"

void TIM1_Init(void){
    RCC->APB2ENR |= (1<<11); // enable Tim1 clock

    uint16_t period = 30000;
    TIM1->CCMR1 |= (0b110 << 4);
    TIM1->CCMR1 |= (0b110 << 12);
    TIM1->CCMR2 |= (0b110 << 4);
    
    TIM1->ARR   = period;
    TIM1->CCR1  = period/2;
    TIM1->CCR2  = period/4;
    TIM1->CCR3  = period/8;
}

void TIM1_enable(void){
    TIM1->CR1   |= (1<<0);
    TIM1->CCER  |= (1<<0);
    TIM1->CCER  |= (1<<4);
    TIM1->CCER  |= (1<<8);
    TIM1->BDTR  |= (1<<15);
}

void TIM1_cc1_mode(void){}