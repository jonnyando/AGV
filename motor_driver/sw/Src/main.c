#include "main.h"
#include "stm32f1xx_hal.h"
#include "spi.h"
#include "UART_config.h"

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart3;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

int main(void)
{

    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();
    MX_USART2_UART_Init();
    SPI1_setup();
    // setup_uart3();

	// printf("Status Register: %u\n", (SPI1->SR));
    HAL_UART_Transmit(&huart3, "USART3->BRR:\t", 13, 100);
    print_reg(&huart3, USART3->BRR, 16);
    HAL_UART_Transmit(&huart3, "USART3->CR1:\t", 13, 100);
    print_reg(&huart3, USART3->CR1, 16);
    HAL_UART_Transmit(&huart3, "USART3->CR2:\t", 13, 100);
    print_reg(&huart3, USART3->CR2, 16);
    HAL_UART_Transmit(&huart3, "USART3->CR3:\t", 13, 100);
    print_reg(&huart3, USART3->CR3, 16);
    HAL_UART_Transmit(&huart3, "RCC->APB1ENR:\t", 14, 100);
    print_reg(&huart3, RCC->APB1ENR, 32);
    HAL_UART_Transmit(&huart3, "RCC->APB2ENR:\t", 14, 100);
    print_reg(&huart3, RCC->APB2ENR, 32);
    HAL_UART_Transmit(&huart3, "GPIOA->CRL:\t", 12, 100);
    print_reg(&huart3, GPIOA->CRL, 32);
    HAL_UART_Transmit(&huart3, "GPIOA->CRH:\t", 12, 100);
    print_reg(&huart3, GPIOA->CRH, 32);
    HAL_UART_Transmit(&huart3, "GPIOB->CRH:\t", 12, 100);
    print_reg(&huart3, GPIOB->CRH, 32);
    HAL_UART_Transmit(&huart3, "SPI1->CR1:\t", 11, 100);
    print_reg(&huart3, SPI1->CR1, 16);
    HAL_UART_Transmit(&huart3, "SPI1->CR2:\t", 11, 100);
    print_reg(&huart3, SPI1->CR2, 16);
    HAL_UART_Transmit(&huart3, "SPI1->I2SCFGR:\t", 15, 100);
    print_reg(&huart3, SPI1->I2SCFGR, 16);
    HAL_UART_Transmit(&huart3, "\n", 1, 100);


    while (1)
    {
        // HAL_Delay(100);
        // HAL_UART_Transmit(&huart3, &data, 4, 1000);
        // transmit_uart3();
        // GPIOB->BSRR = GPIO_BSRR_BS2;
        // SPI transmit:
        SPI1_Transfer(0xCA1B);
        // HAL_UART_Transmit(&huart3, "Sending...", 10, 100);
        // SPI1_Transfer(0xF8);

        // HAL_UART_Transmit(&huart3, "done\n", 5, 100);
        // HAL_Delay(1000);
        // GPIOB->BSRR = GPIO_BSRR_BR2;
        HAL_Delay(1000);
        // printf("words\n");
        // test_func();

    }

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

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{
    __HAL_RCC_USART3_CLK_ENABLE();

    huart3.Instance = USART3;
    huart3.Init.BaudRate = 115200;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
        HAL_Delay(200);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
        _Error_Handler(__FILE__, __LINE__);
    }

    __HAL_UART_ENABLE(&huart3);

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
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);

    GPIO_InitTypeDef GPIOC_InitStruct;
    /*Configure GPIO pin : B1_Pin */
    GPIOC_InitStruct.Pin  = GPIO_PIN_8;
    GPIOC_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIOC_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIOC_InitStruct);

    GPIO_InitTypeDef GPIOA_InitStruct;
    /*Configure GPIO pin : B1_Pin */
    GPIOA_InitStruct.Pin  = GPIO_PIN_3;
    GPIOA_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIOA_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIOA_InitStruct);

    GPIO_InitTypeDef GPIO_InitStruct;
    /*Configure GPIO pin : LD2_Pin */
    GPIO_InitStruct.Pin   = GPIO_PIN_2;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* EXTI interrupt init*/
    // HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
    // HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
    /* User can add his own implementation to report the HAL error return state */
    while(1)
    {
        // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
        // HAL_Delay(300);
        // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
        // HAL_Delay(300);
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
