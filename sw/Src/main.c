#include "main.h"
// #include "syscalls.h"
#include "stm32f1xx_hal.h"
#include "spi.h"
#include "UART_config.h"
#include <stdio.h>

#define nOCTW       GPIO_PIN_4
#define nFAULT      GPIO_PIN_5
#define DC_CAL      GPIO_PIN_6
#define EN_GATE     GPIO_PIN_7
#define INH_A       GPIO_PIN_8
#define INH_B       GPIO_PIN_9
#define INH_C       GPIO_PIN_10
#define LED_FAULT   GPIO_PIN_11
#define INL_C       GPIO_PIN_13
#define INL_B       GPIO_PIN_14
#define INL_A       GPIO_PIN_15

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart3;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
uint16_t drv_transceive(uint16_t tx_reg);

int __io_putchar(int ch)
{
    uint8_t ch8 = ch;
    transmit_uart1((uint8_t*)&ch8);
    return ch;
}

int main(void)
{

    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();
    SPI1_setup();
    UART1_setup();

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
    GPIOB->BRR |= GPIO_PIN_8;
    GPIOB->BRR |= GPIO_PIN_9;
    GPIOB->BRR |= GPIO_PIN_10;
    GPIOB->BRR |= GPIO_PIN_13;
    GPIOB->BRR |= GPIO_PIN_14;
    GPIOB->BRR |= GPIO_PIN_15;

    GPIOB->BRR |= GPIO_PIN_6;
    printf("DC_CAL set LOW (off)\n");
    GPIOB->ODR |= GPIO_PIN_7;
    printf("\nEnabled DRV8303\n");

    uint16_t tx = 0b1001100000000011;
    uint16_t rx = 0x0000;


    while (1){
        HAL_Delay(500);

        GPIOB->ODR |= INH_A;
        GPIOB->BRR |= INL_A;

        tx = 0b1000100000000000;
        printf("Sending\t");  print_reg(tx, 16);
        rx = drv_transceive(tx);
        printf("received\t");  print_reg(rx, 16);
        HAL_Delay(10);

        tx = 0b1001000000000000;
        printf("Sending\t");  print_reg(tx, 16);
        rx = drv_transceive(tx);
        printf("received\t");  print_reg(rx, 16);
        HAL_Delay(10);

        tx = 0b1001100000000000;
        printf("Sending\t");  print_reg(tx, 16);
        rx = drv_transceive(tx);
        printf("received\t");  print_reg(rx, 16);
        HAL_Delay(10);

        // GPIOB->ODR |= GPIO_PIN_8;
        HAL_Delay(500);
        GPIOB->BRR |= INH_A;
        GPIOB->ODR |= INL_A;

        GPIOB->BRR |= GPIO_PIN_8;
        if(GPIOB->IDR && GPIO_PIN_4)
            printf("nOCTW is HIGH\n");
        if(GPIOB->IDR && GPIO_PIN_5)
            printf("nFAULT is HIGH\n");
    }

}

uint16_t drv_transceive(uint16_t tx_reg){
    //tx_reg = 0b1000100000000000;
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

    GPIO_InitTypeDef GPIOB_InitStruct;


    /*Configure nOCTW pin : B04_Pin */
    GPIOB_InitStruct.Pin  = GPIO_PIN_4;
    GPIOB_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIOB_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIOB_InitStruct);
    /*Configure nFAULT pin : B05_Pin */
    GPIOB_InitStruct.Pin  = GPIO_PIN_5;
    GPIOB_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIOB_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIOB_InitStruct);
    /*Configure DC_CAL pin : B06_Pin */
    GPIOB_InitStruct.Pin  = GPIO_PIN_6;
    GPIOB_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIOB_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIOB_InitStruct);
    /*Configure EN_GATE pin : B07_Pin */
    GPIOB_InitStruct.Pin  = GPIO_PIN_7;
    GPIOB_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIOB_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIOB_InitStruct);
    /*Configure TIM1_CH1 pin : B08_Pin */
    GPIOB_InitStruct.Pin  = GPIO_PIN_8;
    GPIOB_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIOB_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIOB_InitStruct);
    /*Configure TIM1_CH2 pin : B09_Pin */
    GPIOB_InitStruct.Pin  = GPIO_PIN_9;
    GPIOB_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIOB_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIOB_InitStruct);
    /*Configure TIM1_CH3 pin : B10_Pin */
    GPIOB_InitStruct.Pin  = GPIO_PIN_10;
    GPIOB_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIOB_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIOB_InitStruct);
    /*Configure LED_FAULT pin : B11_Pin */
    GPIOB_InitStruct.Pin  = GPIO_PIN_11;
    GPIOB_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIOB_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIOB_InitStruct);
    /*Configure TIM1_CH3N pin : B13_Pin */
    GPIOB_InitStruct.Pin  = GPIO_PIN_13;
    GPIOB_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIOB_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIOB_InitStruct);
    /*Configure TIM1_CH2N pin : B14_Pin */
    GPIOB_InitStruct.Pin  = GPIO_PIN_14;
    GPIOB_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIOB_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIOB_InitStruct);
    /*Configure TIM1_CH1N pin : B15_Pin */
    GPIOB_InitStruct.Pin  = GPIO_PIN_15;
    GPIOB_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIOB_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIOB_InitStruct);
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
