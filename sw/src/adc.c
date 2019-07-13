#include "adc.h"


void ADC1_Init(void){
    RCC->CFGR |= RCC_CFGR_ADCPRE_DIV2;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    pin_mode(GPIOA, 1, GPIO_IN_AN);
    // pin_mode(GPIOA, 2, GPIO_IN_AN);
    // pin_mode(GPIOA, 3, GPIO_IN_AN);


}