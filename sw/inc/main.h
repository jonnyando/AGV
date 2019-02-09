#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

                // Pin     // Port
#define nOCTW       5      // B
#define nFAULT      4      // B
#define DC_CAL      12     // A 
#define EN_GATE     11     // A 
#define INH_A       8      // A
#define INH_B       9      // A
#define INH_C       10     // A
#define LED_FAULT   11     // B
#define INL_C       13     // B
#define INL_B       14     // B
#define INL_A       15     // B

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


#endif /* __MAIN_H__ */
