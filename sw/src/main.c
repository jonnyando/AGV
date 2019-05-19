#include "main.h"

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

    UART1_Init();  printf("uart initialized\n\n");
    SPI1_Init();   printf("spi initialized\n\n");
    GPIO_Init();   printf("gpio initialized\n\n");
    
    // printf("RCC->CR\t\t");      print_reg(RCC->CR,     32);
    // printf("RCC->CFGR\t\t");      print_reg(RCC->CFGR,     32);
    // printf("RCC->AHBENR\t");      print_reg(RCC->AHBENR,     32);
    // printf("RCC->APB2ENR\t");      print_reg(RCC->APB2ENR,     32);
    // printf("RCC->CSR\t\t");      print_reg(RCC->CSR,     32);

    HAL_Delay(1000);

    pin_reset(GPIOA, INH_A);
    pin_reset(GPIOA, INH_A);
    pin_reset(GPIOA, INH_A);
    pin_reset(GPIOB, INL_A);
    pin_reset(GPIOB, INL_B);
    pin_reset(GPIOB, INL_C);
    pin_reset(GPIOA, DC_CAL);
    pin_reset(GPIOA, EN_GATE);
    pin_reset(SPI_CS_PORT, SPI_CS_PIN);
    printf("DC_CAL set LOW (off)\n");
    // GPIOA->ODR |= EN_GATE;
    pin_set(GPIOA, EN_GATE);
    printf("\nEnabled DRV8303\n");

    // uint16_t tx = 0b1001100000000011;
    // uint16_t rx = 0x0000;
    uint16_t tx;
    uint16_t rx;

    // sets OC_ADJ_SET to 24 (Vds = 1.043v)
    // drv_write(0x02, 0b1100000000);
    // Disable OCP
    // drv_write(0x02, 0b00000110000);
    // // sets drv to 3-PWM mode
    // drv_write(0x02, 0b00000001000);
    // // sets drv to 3-PWM mode
    // drv_write(0x02, 0b10000110000);

    if(!pin_read(GPIOA, nOCTW)){
        printf("nOCTW is LOW\n");
        pin_set(GPIOB, LED_FAULT);
    } else {
        pin_reset(GPIOB, LED_FAULT);
    }
    if(!pin_read(GPIOB, nFAULT)){
        printf("nFAULT is LOW\n");
        pin_set(GPIOB, LED_FAULT);
    } else {
        pin_reset(GPIOB, LED_FAULT);
    }


    rx = drv_read(0x00);
    printf("Status register 1\t");  print_reg(rx, 16);
    HAL_Delay(10);

    rx = drv_read(0x01);
    printf("Status register 2\t");  print_reg(rx, 16);
    HAL_Delay(10);

    rx = drv_read(0x02);
    printf("Control register 1\t");  print_reg(rx, 16);
    HAL_Delay(10);
    
    rx = drv_read(0x03);
    printf("Control register 2\t");  print_reg(rx, 16);
    HAL_Delay(10);

    printf("\n");
    uint16_t t_del = 1000;
    while (1){
        // pin_set(GPIOA, LED);
        // HAL_Delay(200);
        // pin_reset(GPIOA, LED);
        // HAL_Delay(200);
        // printf("beep..");
        // fflush(stdout);
        
        // pin_set(GPIOA, 4);
        // pin_reset(GPIOA, 4);
        // HAL_Delay(t_del);
        // pin_reset(GPIOA, INH_A); // 1
        // pin_reset(GPIOB, INL_A);
        // pin_set(GPIOA, INH_B);
        // pin_reset(GPIOB, INL_B);
        // pin_reset(GPIOA, INH_C);
        // pin_set(GPIOB, INL_C);
        // // printf("1\n");
        // HAL_Delay(t_del);
        // pin_reset(GPIOA, INH_A); // 2
        // pin_set(GPIOB, INL_A);
        // pin_set(GPIOA, INH_B);
        // pin_reset(GPIOB, INL_B);
        // pin_reset(GPIOA, INH_C);
        // pin_reset(GPIOB, INL_C);
        // // printf("2\n");
        // HAL_Delay(t_del);
        // pin_reset(GPIOA, INH_A); // 3
        // pin_set(GPIOB, INL_A);
        // pin_reset(GPIOA, INH_B);
        // pin_reset(GPIOB, INL_B);
        // pin_set(GPIOA, INH_C);
        // pin_reset(GPIOB, INL_C);
        // // printf("3\n");
        // HAL_Delay(t_del);
        // pin_reset(GPIOA, INH_A); // 4
        // pin_reset(GPIOB, INL_A);
        // pin_reset(GPIOA, INH_B);
        // pin_set(GPIOB, INL_B);
        // pin_set(GPIOA, INH_C);
        // pin_reset(GPIOB, INL_C);
        // // printf("4\n");
        // HAL_Delay(t_del);
        // pin_set(GPIOA, INH_A); // 5
        // pin_reset(GPIOB, INL_A);
        // pin_reset(GPIOA, INH_B);
        // pin_set(GPIOB, INL_B);
        // pin_reset(GPIOA, INH_C);
        // pin_reset(GPIOB, INL_C);
        // // printf("5\n");
        // HAL_Delay(t_del);
        // pin_set(GPIOA, INH_A); // 6
        // pin_reset(GPIOB, INL_A);
        // pin_reset(GPIOA, INH_B);
        // pin_reset(GPIOB, INL_B);
        // pin_reset(GPIOA, INH_C);
        // pin_set(GPIOB, INL_C);
        // printf("6\n");

        rx = drv_read(0x00);
        printf("Status register 1\t");  print_reg(rx, 16);
        HAL_Delay(10);
        rx = drv_read(0x01);
        printf("Status register 2\t");  print_reg(rx, 16);
        HAL_Delay(10);
        rx = drv_read(0x02);
        printf("Status register 2\t");  print_reg(rx, 16);
        HAL_Delay(10);
        rx = drv_read(0x03);
        printf("Status register 2\t");  print_reg(rx, 16);
        HAL_Delay(1000);

        // tx = 0b1001000000000000;
        // //printf("Sending\t");  print_reg(tx, 16);
        // rx = drv_transceive(tx);
        // printf("Control register 1\t");  print_reg(rx, 16);
        // HAL_Delay(1000);

        // tx = 0b1001100000000000;
        // //printf("Sending\t");  print_reg(tx, 16);
        // rx = drv_transceive(tx);
        // printf("Control register 2\t");  print_reg(rx, 16);
        // HAL_Delay(1000);

        
        // drv_write(0x02, 0b00000000100);
        // pin_reset(GPIOA, EN_GATE);
        // HAL_Delay(100);
        // pin_set(GPIOA, EN_GATE);
        // printf("reset\n");
        // HAL_Delay(10);

        // printf("\n");

        // GPIOB->ODR |= GPIO_PIN_8;

        // if(!pin_read(GPIOA, nOCTW)){
        //     printf("nOCTW is LOW\n");
        //     pin_set(GPIOB, LED_FAULT);
        // } else {
        //     pin_reset(GPIOB, LED_FAULT);
        // }
        // if(!pin_read(GPIOB, nFAULT)){
        //     printf("nFAULT is LOW\n");
        //     pin_set(GPIOB, LED_FAULT);
        // } else {
        //     pin_reset(GPIOB, LED_FAULT);
        // }

    }

}

// uint16_t drv_transceive(uint16_t tx_reg){
//     //tx_reg = 0b1000100000000000;
//     pin_reset(SPI_CS_PORT, SPI_CS_PIN);
//     SPI1_Transfer(tx_reg);
//     for(int i=0;i<35;i++){__ASM("nop");}
//     pin_set(SPI_CS_PORT, SPI_CS_PIN);
//     HAL_Delay(1);
//     pin_reset(SPI_CS_PORT, SPI_CS_PIN);
//     uint16_t rx_reg = SPI1_Transcieve(0b1000000000000000);
//     HAL_Delay(1);
//     pin_set(SPI_CS_PORT, SPI_CS_PIN);

//     return rx_reg;
// }

// uint16_t drv_write(uint8_t drv_reg, uint16_t payload){
//     //tx_reg = 0b1000100000000000;
//     uint16_t tx_reg = 0;
//     tx_reg |= drv_reg << 11;
//     // tx_reg |= (1<<15);
//     tx_reg |= payload;

//     printf("transmitting\t");  print_reg(tx_reg, 16);

//     pin_reset(SPI_CS_PORT, SPI_CS_PIN);
//     SPI1_Transfer(tx_reg);
//     for(int i=0;i<35;i++){__ASM("nop");}
//     pin_set(SPI_CS_PORT, SPI_CS_PIN);
//     HAL_Delay(1);
//     pin_reset(SPI_CS_PORT, SPI_CS_PIN);
//     uint16_t rx_reg = SPI1_Transcieve(0b1000000000000000);
//     HAL_Delay(1);
//     pin_set(SPI_CS_PORT, SPI_CS_PIN);

//     return rx_reg;
// }

/**@brief System Clock Configuration*/
void SystemClock_Config(void)
{
    // RCC->CR = 0x00000083; // reset clock to use HSI
    // RCC->CR |= (1 << 0); // HSI enable
    // while(!(RCC->CR && (1<<1))); // wait for HSI to stabilise
    // RCC->CFGR ^= (0b1110 << 18); // PLLMUL = x16
    // RCC->CFGR |= (0b1110 << 18); // PLLMUL = x16
    // RCC->CFGR ^= (1 << 16); // PLLSOURCE = HSI/2
    // RCC->CR |= (1 << 24); // PLL enable
    // while(!(RCC->CR && (1<<25))); // wait for PLL to stabilise
    // RCC->CR      |= 0b00000011000000000101000110000011;
    // RCC->CFGR	 |= 0b00000000001110000000010000001010;
    // RCC->AHBENR  |= 0b00000000000000000000000000010100;
    // RCC->APB2ENR |= 0b00000000000000000101000000111101;

    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks*/
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = 16;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);
    // if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK){/*error*/}

    /**Initializes the CPU, AHB and APB busses clocks*/
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
    |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
    // if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK){/*error*/}
    
    
    /*CANNOT RECONFIGURE RCC WITHOUT ALSO CONFIGURING SYSTICK/DELAY*/
    
    // RCC->CFGR |= (1 << 10);      // (PPRE1) APB1 = HCLK/2
    // RCC->AHBENR |= (1 << 2); //SRAM clock enabled during sleep
    // RCC->AHBENR |= (1 << 4); //FLITFEN clock enabled during sleep
    // RCC->APB2ENR |= (1 << 2); // enable GPIOA
    // RCC->APB2ENR |= (1 << 3); // enable GPIOB
    // RCC->APB2ENR |= (1 << 4); // enable GPIOC
    // RCC->APB2ENR |= (1 << 5); // enable GPIOD
    // RCC->APB2ENR |= (1 << 0); // enable AFIO
    // RCC->APB2ENR |= (1 << 14); // enable USART1
    // RCC->APB2ENR |= (1 << 12); // enable SPI1
    // RCC->CFGR |= (0b10 << 0);    // Set SYSCLK to PLL
    // while(!(RCC->CFGR && (0b10 << 2)));

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

    pin_mode(GPIOA, nOCTW,     GPIO_IN_PULL);
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
    pin_mode(GPIOA, LED,       GPIO_OUT_PP);
}