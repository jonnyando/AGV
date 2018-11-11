#include "main.h"
// #include "syscalls.h"
#include "stm32f1xx_hal.h"
#include "spi.h"
#include "UART_config.h"
#include <stdio.h>

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart3;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

int __io_putchar(int ch)
{
    uint8_t ch8 = ch;
    transmit_uart2((uint8_t*)&ch8);
    // HAL_UART_Transmit(&huart3,(uint8_t *)&ch8,1,HAL_MAX_DELAY);
    return ch;
}

int main(void)
{

    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();
    // MX_USART2_UART_Init();
    SPI1_setup();
    setup_uart2();
    // setup_uart3();

    printf("USART2->BRR\t");    print_reg(USART2->BRR,   32);
    printf("USART2->CR1\t");    print_reg(USART2->CR1,   32);
    printf("USART2->CR2\t");    print_reg(USART2->CR2,   32);
    printf("USART2->CR3\t");    print_reg(USART2->CR3,   32);
    printf("RCC->APB1ENR\t");   print_reg(RCC->APB1ENR,  32);
    printf("RCC->APB2ENR\t");   print_reg(RCC->APB2ENR,  32);
    printf("GPIOA->CRL\t");     print_reg(GPIOA->CRL,    32);
    printf("GPIOA->CRH\t");     print_reg(GPIOA->CRH,    32);
    printf("GPIOB->CRH\t");     print_reg(GPIOB->CRH,    32);
    printf("SPI1->CR1\t");      print_reg(SPI1->CR1,     16);
    printf("SPI1->CR2\t");      print_reg(SPI1->CR2,     16);
    printf("SPI1->I2SCFGR\t");  print_reg(SPI1->I2SCFGR, 16);
    fflush(stdout);
    HAL_Delay(1000);
    printf("a\n\n\n");
    GPIOC->ODR |= GPIO_PIN_3;
    printf("Enabled DRV8303\n");

    GPIOA->BRR |= GPIO_PIN_0;
    printf("DC_CAL set LOW (off)\n");

    HAL_Delay(1000);

    while (1){
        // uint16_t CR1 = 0b0001000110001000;
        // printf("Setting CR1: %u\n", CR1);
        // uint16_t rx = SPI1_Transcieve(CR1);
        // // printf("Received: %u\n", rx);
        // // rx = SPI1_Transcieve(CR1);
        // // rx &= 0b0000011111111111;
        // // printf("Received: %u\n", rx);
        // uint16_t command = 0b1001011111111111;
        // printf("Reading CR1: %u\n", command);
        // rx = SPI1_Transcieve(command);
        // rx &= 0b0000011111111111;
        // printf("Received: %u\n", rx);
        // rx = SPI1_Transcieve(command);
        // rx &= 0b0000011111111111;
        // printf("Received: %u\n", rx);
        // HAL_Delay(900);
        // GPIOB->ODR |= GPIO_PIN_2;   // switch on PB2
        // HAL_Delay(100);
        // GPIOB->BRR |= GPIO_PIN_2;   // switch off PB2
        SPI1_Transcieve(0b1001011111111111);
        HAL_Delay(10);
        // SPI1_Transcieve(0b1001011111111111);
        if(!(GPIOC->IDR & (GPIO_PIN_2 | GPIO_PIN_1))){
            GPIOB->ODR |= GPIO_PIN_2;
        }
        else{
            GPIOB->BRR |= GPIO_PIN_2;
        }

        GPIOC->ODR |= GPIO_PIN_10;
        GPIOC->ODR |= GPIO_PIN_12;
        HAL_Delay(500);
        GPIOC->BRR |= GPIO_PIN_10;
        GPIOC->BRR |= GPIO_PIN_12;
        HAL_Delay(500);
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
// static void MX_USART2_UART_Init(void)
// {
//     __HAL_RCC_USART3_CLK_ENABLE();
//
//     huart3.Instance = USART3;
//     huart3.Init.BaudRate = 115200;
//     huart3.Init.WordLength = UART_WORDLENGTH_8B;
//     huart3.Init.StopBits = UART_STOPBITS_1;
//     huart3.Init.Parity = UART_PARITY_NONE;
//     huart3.Init.Mode = UART_MODE_TX_RX;
//     huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//     huart3.Init.OverSampling = UART_OVERSAMPLING_16;
//     if (HAL_UART_Init(&huart3) != HAL_OK)
//     {
//         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
//         HAL_Delay(200);
//         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
//         _Error_Handler(__FILE__, __LINE__);
//     }
//
//     __HAL_UART_ENABLE(&huart3);
//
// }

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

    GPIO_InitTypeDef GPIOB_InitStruct;
    /*Configure LED pin : B2_Pin */
    GPIOB_InitStruct.Pin  = GPIO_PIN_2;
    GPIOB_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIOB_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIOB_InitStruct);

    GPIO_InitTypeDef GPIOC_InitStruct;
    /* EXTI interrupt init*/
    // HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
    // HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

    /*Configure DC_CAL pin : C0_Pin */
    GPIOC_InitStruct.Pin  = GPIO_PIN_0;
    GPIOC_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIOC_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIOC_InitStruct);

    /*Configure nOCTW pin : C1_Pin */
    GPIOC_InitStruct.Pin  = GPIO_PIN_1;
    GPIOC_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIOC_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOC, &GPIOC_InitStruct);

    /*Configure EN_GATE pin : C2_Pin */
    GPIOC_InitStruct.Pin  = GPIO_PIN_2;
    GPIOC_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIOC_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOC, &GPIOC_InitStruct);

    /*Configure EN_GATE pin : C3_Pin */
    GPIOC_InitStruct.Pin  = GPIO_PIN_3;
    GPIOC_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIOC_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIOC_InitStruct);


    /*Configure CH pin : C12_Pin */
    GPIOC_InitStruct.Pin  = GPIO_PIN_12;
    GPIOC_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIOC_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIOC_InitStruct);

    /*Configure CL pin : C10_Pin */
    GPIOC_InitStruct.Pin  = GPIO_PIN_10;
    GPIOC_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIOC_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIOC_InitStruct);
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
