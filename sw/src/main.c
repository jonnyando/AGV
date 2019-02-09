#include "main.h"
// #include "syscalls.h"
#include "stm32f1xx_hal.h"
#include "spi.h"
#include "uart.h"
#include "gpio.h"
#include <stdio.h>
                    // Pin          // Pin
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

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart3;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
uint16_t drv_transceive(uint16_t tx_reg);
uint16_t drv_write(uint8_t drv_reg, uint16_t payload);

int __io_putchar(int ch)
{
    uint8_t ch8 = ch;
    transmit_uart1((uint8_t*)&ch8);
    return ch;
}

// void pin_mode(GPIO_TypeDef  *port_reg, uint8_t pin, uint8_t mode){
//     volatile uint32_t *configreg;
//     uint32_t config;
//     uint8_t  pin_offset;
//     configreg = (pin < 8) ? &port_reg->CRL : &port_reg->CRH;
//     pin_offset = (pin < 8) ? pin : pin - 8;
//     config = 0xFFFFFFFF ^ ((0b1111) << (pin_offset * 4));
//     *configreg &= config;
//     *configreg |= mode << (pin_offset * 4);
// }

int main(void)
{

    HAL_Init();

    SystemClock_Config();

    UART1_setup();
    MX_GPIO_Init();
    SPI1_setup();

    printf("spi initialized\n\n");
    fflush(stdout);
    // printf("USART1->BRR\t");    print_reg(USART1->BRR,   32);
    // printf("USART1->CR1\t");    print_reg(USART1->CR1,   32);
    // printf("USART1->CR2\t");    print_reg(USART1->CR2,   32);
    // printf("USART1->CR3\t");    print_reg(USART1->CR3,   32);
    // printf("RCC->APB1ENR\t");   print_reg(RCC->APB1ENR,  32);
    // printf("RCC->APB2ENR\t");   print_reg(RCC->APB2ENR,  32);
    // printf("GPIOA->CRL\t");     print_reg(GPIOA->CRL,    32);
    // printf("GPIOA->CRH\t");     print_reg(GPIOA->CRH,    32);
    // printf("GPIOB->CRH\t");     print_reg(GPIOB->CRH,    32);
    printf("SPI1->CR1\t\t");      print_reg(SPI1->CR1,     16);
    printf("SPI1->CR2\t\t");      print_reg(SPI1->CR2,     16);
    printf("SPI1->SR\t\t");       print_reg(SPI1->SR,      16);
    printf("SPI1->DR\t\t");       print_reg(SPI1->DR,      16);
    printf("SPI1->SR\t\t");       print_reg(SPI1->SR,      16);
    fflush(stdout);
    HAL_Delay(1000);
    GPIOA->BRR |= INH_A;
    GPIOA->BRR |= INH_B;
    GPIOA->BRR |= INH_C;
    GPIOB->BRR |= INL_A;
    GPIOB->BRR |= INL_B;
    GPIOB->BRR |= INL_C;

    GPIOA->BRR |= DC_CAL;
    printf("DC_CAL set LOW (off)\n");
    GPIOA->ODR |= EN_GATE;
    printf("\nEnabled DRV8303\n");

    uint16_t tx = 0b1001100000000011;
    uint16_t rx = 0x0000;

    // sets OC_ADJ_SET to 8 (Vds = 0.155v)
    // drv_write(0x02, 0b01000111000);
    // Disable OCP
    // drv_write(0x02, 0b00000110000);
    // sets drv to 3-PWM mode
    // drv_write(0x02, 0b00000001000);

    if(!(GPIOB->IDR && nOCTW)){
        printf("nOCTW is LOW\n");
        GPIOB->ODR |= LED_FAULT;
    } else {
        GPIOB->BRR |= LED_FAULT;
    }
    if(!(GPIOB->IDR && nFAULT)){
        printf("nFAULT is LOW\n");
        GPIOB->ODR |= LED_FAULT;
    } else {
        GPIOB->BRR |= LED_FAULT;
    }

    
    // printf("GPIOB->CRL\t\t");      print_reg(GPIOB->CRL,     32);
    // printf("GPIOB->ODR\t\t");      print_reg(GPIOB->ODR,     16);

    uint16_t t_del = 500;
    while (1){
        HAL_Delay(t_del);
        pin_set(GPIOA, 1);        
        HAL_Delay(t_del);
        pin_reset(GPIOA, 1);
        // GPIOA->BRR |= INH_A; // 1
        // GPIOB->BRR |= INL_A;
        // GPIOA->ODR |= INH_B;
        // GPIOB->BRR |= INL_B;
        // GPIOA->BRR |= INH_C;
        // HAL_Delay(t_del);
        // GPIOA->BRR |= INH_A; // 2
        // GPIOB->ODR |= INL_A;
        // GPIOA->ODR |= INH_B;
        // GPIOB->BRR |= INL_B;
        // GPIOA->BRR |= INH_C;
        // GPIOB->BRR |= INL_C;
        // HAL_Delay(t_del);
        // GPIOA->BRR |= INH_A; // 3
        // GPIOB->ODR |= INL_A;
        // GPIOA->BRR |= INH_B;
        // GPIOB->BRR |= INL_B;
        // GPIOA->ODR |= INH_C;
        // GPIOB->BRR |= INL_C;
        // HAL_Delay(t_del);
        // GPIOA->BRR |= INH_A; // 4
        // GPIOB->BRR |= INL_A;
        // GPIOA->BRR |= INH_B;
        // GPIOB->ODR |= INL_B;
        // GPIOA->ODR |= INH_C;
        // GPIOB->BRR |= INL_C;
        // HAL_Delay(t_del);
        // GPIOA->ODR |= INH_A; // 5
        // GPIOB->BRR |= INL_A;
        // GPIOA->BRR |= INH_B;
        // GPIOB->ODR |= INL_B;
        // GPIOA->BRR |= INH_C;
        // GPIOB->BRR |= INL_C;
        // HAL_Delay(t_del);
        // GPIOA->ODR |= INH_A; // 6
        // GPIOB->BRR |= INL_A;
        // GPIOA->BRR |= INH_B;
        // GPIOB->BRR |= INL_B;
        // GPIOA->BRR |= INH_C;
        // GPIOB->ODR |= INL_C;

        // tx = 0b1000000000000000;
        // //printf("Sending\t");  print_reg(tx, 16);
        // rx = drv_transceive(tx);
        // printf("Status register 1\t");  print_reg(rx, 16);
        // HAL_Delay(10);

        // tx = 0b1000100000000000;
        // //printf("Sending\t");  print_reg(tx, 16);
        // rx = drv_transceive(tx);
        // printf("Status register 2\t");  print_reg(rx, 16);
        // HAL_Delay(10);

        // tx = 0b1001000000000000;
        // //printf("Sending\t");  print_reg(tx, 16);
        // rx = drv_transceive(tx);
        // printf("Control register 1\t");  print_reg(rx, 16);
        // HAL_Delay(10);

        // tx = 0b1001100000000000;
        // //printf("Sending\t");  print_reg(tx, 16);
        // rx = drv_transceive(tx);
        // printf("Control register 2\t");  print_reg(rx, 16);
        // HAL_Delay(10);

        // printf("\n");

        // GPIOB->ODR |= GPIO_PIN_8;

        // if(!(GPIOB->IDR && nOCTW)){
        //     printf("nOCTW is LOW\n");
        //     GPIOB->ODR |= LED_FAULT;
        // } else {
        //     GPIOB->BRR |= LED_FAULT;
        // }
        // if(!(GPIOB->IDR && nFAULT)){
        //     printf("nFAULT is LOW\n");
        //     GPIOB->ODR |= LED_FAULT;
        // } else {
        //     GPIOB->BRR |= LED_FAULT;
        // }

    }

}

uint16_t drv_transceive(uint16_t tx_reg){
    //tx_reg = 0b1000100000000000;
    pin_reset(SPI_CS_PORT, SPI_CS_PIN);
    SPI1_Transfer(tx_reg);
    for(int i=0;i<35;i++){__ASM("nop");}
    pin_set(SPI_CS_PORT, SPI_CS_PIN);
    HAL_Delay(1);
    pin_reset(SPI_CS_PORT, SPI_CS_PIN);
    uint16_t rx_reg = SPI1_Transcieve(0b1000000000000000);
    HAL_Delay(1);
    pin_set(SPI_CS_PORT, SPI_CS_PIN);

    return rx_reg;
}

uint16_t drv_write(uint8_t drv_reg, uint16_t payload){
    //tx_reg = 0b1000100000000000;
    uint16_t tx_reg = 0;
    tx_reg |= drv_reg << 11;
    // tx_reg |= (1<<15);
    tx_reg |= payload;

    printf("transmitting\t");  print_reg(tx_reg, 16);

    SPI_CS_PORT->BRR |= SPI_CS_PIN;
    SPI1_Transfer(tx_reg);
    for(int i=0;i<35;i++){__ASM("nop");}
    SPI_CS_PORT->ODR |= SPI_CS_PIN;
    HAL_Delay(1);
    SPI_CS_PORT->BRR |= SPI_CS_PIN;
    uint16_t rx_reg = SPI1_Transcieve(0b1000000000000000);
    HAL_Delay(1);
    SPI_CS_PORT->ODR |= SPI_CS_PIN;

    return rx_reg;
}

/**
* @brief System Clock Configuration
* @retval None
*/
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = 16;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }
    /**Initializes the CPU, AHB and APB busses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
    |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }
    /**Configure the Systick interrupt time
    */
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
    /**Configure the Systick
    */
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/** Configure pins as
* Analog
* Input
* Output
* EVENT_OUT
* EXTI
*/
static void MX_GPIO_Init(void)
{


    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    // HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);

    // GPIO_InitTypeDef GPIOA_InitStruct;
    // GPIO_InitTypeDef GPIOB_InitStruct;


    /*Configure nOCTW pin : B05_Pin */
    pin_mode(GPIOB, nOCTW, GPIO_IN_PULL);
    pin_pullmode(GPIOB, nOCTW, GPIO_PULLUP);
    /*Configure nFAULT pin : B04_Pin */
    pin_mode(GPIOB, nFAULT, GPIO_IN_PULL);
    pin_pullmode(GPIOB, nFAULT, GPIO_PULLUP);
    /*Configure DC_CAL pin : A12_Pin */
    pin_mode(GPIOA, DC_CAL, GPIO_OUT_PP);
    /*Configure EN_GATE pin : A11_Pin */
    pin_mode(GPIOA, EN_GATE, GPIO_OUT_PP);
    /*Configure TIM1_CH1 pin : A08_Pin */
    pin_mode(GPIOA, INH_A, GPIO_OUT_PP);
    /*Configure TIM1_CH2 pin : A09_Pin */
    pin_mode(GPIOA, INH_B, GPIO_OUT_PP);
    /*Configure TIM1_CH3 pin : A10_Pin */
    pin_mode(GPIOA, INH_C, GPIO_OUT_PP);
    /*Configure LED_FAULT pin : B11_Pin */
    pin_mode(GPIOB, LED_FAULT, GPIO_OUT_PP);
    /*Configure TIM1_CH3N pin : B13_Pin */
    pin_mode(GPIOB, INL_C, GPIO_OUT_PP);
    /*Configure TIM1_CH2N pin : B14_Pin */
    pin_mode(GPIOB, INL_B, GPIO_OUT_PP);
    /*Configure TIM1_CH1N pin : B15_Pin */
    pin_mode(GPIOB, INL_A, GPIO_OUT_PP);
    pin_mode(GPIOA, 1, GPIO_OUT_PP);
}

/**
* @brief  This function is executed in case of error occurrence.
* @param  file: The file name as string.
* @param  line: The line in file as a number.
* @retval None
*/
void _Error_Handler(char *file, int line)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    while(1)
    {
        printf("error in %s, line %d", file, line);
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
* @brief  Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param  file: pointer to the source file name
* @param  line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t* file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
    tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
