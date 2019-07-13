#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

// #include "syscalls.h"
// #include "stm32f1xx_hal.h"
#ifndef STM32F103xB
    #define STM32F103xB
#endif
#include "stm32f1xx.h"
#include "spi.h"
#include "uart.h"
#include "gpio.h"
#include "tim.h"
#include <stdio.h>
#include <math.h>
#include "drv8303.h"

/* Private define ------------------------------------------------------------*/
#define PI 3.14159265358979323846264


                // Pin     // Port
#define nOCTW       15      // A
#define nFAULT      12      // B
#define DC_CAL      12     // A 
#define EN_GATE     11     // A 
#define INH_A       8      // A
#define INH_B       9      // A
#define INH_C       10     // A
#define LED_FAULT   11     // B
#define INL_A       13     // B
#define INL_B       14     // B
#define INL_C       15     // B
#define ENC_I       8      // B
#define ENC_A       4      // B
#define ENC_B       5      // B

#define LED         1      // A


#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

static void GPIO_Init(void);



#endif /* __MAIN_H__ */