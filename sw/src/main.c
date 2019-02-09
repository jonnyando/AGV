#include "main.h"
// #include "syscalls.h"
#include "stm32f1xx_hal.h"
#include "spi.h"
#include "uart.h"
#include "gpio.h"
#include <stdio.h>


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void GPIO_Init(void);
uint16_t drv_transceive(uint16_t tx_reg);
uint16_t drv_write(uint8_t drv_reg, uint16_t payload);

int __io_putchar(int ch)
{
    uint8_t ch8 = ch;
    transmit_uart1((uint8_t*)&ch8);
    return ch;
}

int main(void)
{
    SystemClock_Config();

    UART1_setup();
    printf("uart initialized\n\n");
    SPI1_setup();
    printf("spi initialized\n\n");
    GPIO_Init();
    printf("gpio initialized\n\n");

    // fflush(stdout);
    // printf("SPI1->CR1\t\t");      print_reg(SPI1->CR1,     16);
    // fflush(stdout);
    HAL_Delay(1000);
    pin_reset(GPIOA, INH_A);
    pin_reset(GPIOA, INH_A);
    pin_reset(GPIOA, INH_A);
    pin_reset(GPIOB, INL_A);
    pin_reset(GPIOB, INL_B);
    pin_reset(GPIOB, INL_C);
    pin_reset(GPIOA, DC_CAL);
    printf("DC_CAL set LOW (off)\n");
    // GPIOA->ODR |= EN_GATE;
    pin_set(GPIOA, EN_GATE);
    printf("\nEnabled DRV8303\n");

    uint16_t tx = 0b1001100000000011;
    uint16_t rx = 0x0000;

    // sets OC_ADJ_SET to 8 (Vds = 0.155v)
    // drv_write(0x02, 0b01000111000);
    // Disable OCP
    // drv_write(0x02, 0b00000110000);
    // sets drv to 3-PWM mode
    // drv_write(0x02, 0b00000001000);

    if(!(GPIOB->IDR && (1 << nOCTW))){
        printf("nOCTW is LOW\n");
        pin_set(GPIOB, LED_FAULT);
    } else {
        pin_reset(GPIOB, LED_FAULT);
    }
    if(!(GPIOB->IDR && (1 << nFAULT))){
        printf("nFAULT is LOW\n");
        pin_set(GPIOB, LED_FAULT);
    } else {
        pin_reset(GPIOB, LED_FAULT);
    }
    uint16_t t_del = 500;
    while (1){
        HAL_Delay(t_del);
        pin_set(GPIOA, 1);        
        HAL_Delay(t_del);
        pin_reset(GPIOA, 1);
        
        // HAL_Delay(t_del);
        // pin_reset(GPIOA, INH_A); // 1
        // pin_reset(GPIOB, INL_A);
        // pin_set(GPIOA, INH_B);
        // pin_reset(GPIOB, INL_B);
        // pin_reset(GPIOA, INH_C);
        // pin_set(GPIOB, INL_C);
        // HAL_Delay(t_del);
        // pin_reset(GPIOA, INH_A); // 2
        // pin_set(GPIOB, INL_A);
        // pin_set(GPIOA, INH_B);
        // pin_reset(GPIOB, INL_B);
        // pin_reset(GPIOA, INH_C);
        // pin_reset(GPIOB, INL_C);
        // HAL_Delay(t_del);
        // pin_reset(GPIOA, INH_A); // 3
        // pin_set(GPIOB, INL_A);
        // pin_reset(GPIOA, INH_B);
        // pin_reset(GPIOB, INL_B);
        // pin_set(GPIOA, INH_C);
        // pin_reset(GPIOB, INL_C);
        // HAL_Delay(t_del);
        // pin_reset(GPIOA, INH_A); // 4
        // pin_reset(GPIOB, INL_A);
        // pin_reset(GPIOA, INH_B);
        // pin_set(GPIOB, INL_B);
        // pin_set(GPIOA, INH_C);
        // pin_reset(GPIOB, INL_C);
        // HAL_Delay(t_del);
        // pin_set(GPIOA, INH_A); // 5
        // pin_reset(GPIOB, INL_A);
        // pin_reset(GPIOA, INH_B);
        // pin_set(GPIOB, INL_B);
        // pin_reset(GPIOA, INH_C);
        // pin_reset(GPIOB, INL_C);
        // HAL_Delay(t_del);
        // pin_set(GPIOA, INH_A); // 6
        // pin_reset(GPIOB, INL_A);
        // pin_reset(GPIOA, INH_B);
        // pin_reset(GPIOB, INL_B);
        // pin_reset(GPIOA, INH_C);
        // pin_set(GPIOB, INL_C);

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

/**@brief System Clock Configuration*/
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks*/
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = 16;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK){/*error*/}

    /**Initializes the CPU, AHB and APB busses clocks*/
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
    |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK){/*error*/}
    /*Configure the Systick interrupt time*/
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
    /**Configure the Systick*/
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

static void GPIO_Init(void)
{
    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    pin_mode(GPIOB, nOCTW,     GPIO_IN_PULL);
    pin_pullmode(GPIOB, nOCTW, GPIO_PULLUP);
    pin_mode(GPIOB, nFAULT,    GPIO_IN_PULL);
    pin_pullmode(GPIOB, nFAULT, GPIO_PULLUP);
    
    pin_mode(GPIOA, DC_CAL,    GPIO_OUT_PP);
    pin_mode(GPIOA, EN_GATE,   GPIO_OUT_PP);
    pin_mode(GPIOA, INH_A,     GPIO_OUT_PP);
    pin_mode(GPIOA, INH_B,     GPIO_OUT_PP);
    pin_mode(GPIOA, INH_C,     GPIO_OUT_PP);
    pin_mode(GPIOB, LED_FAULT, GPIO_OUT_PP);
    pin_mode(GPIOB, INL_C,     GPIO_OUT_PP);
    pin_mode(GPIOB, INL_B,     GPIO_OUT_PP);
    pin_mode(GPIOB, INL_A,     GPIO_OUT_PP);
    pin_mode(GPIOA, 1,         GPIO_OUT_PP);
}