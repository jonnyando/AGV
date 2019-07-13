#include "main.h"

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

int __io_putchar(int ch)
{
    uint8_t ch8 = ch;
    transmit_byte_uart1(ch8);
    // ITM_SendChar(ch);
    return ch;
}

volatile uint32_t myTicks = 0;
void SysTick_Handler(void)
{
    myTicks++;
    // HAL_IncTick();
}

void myDelay(uint32_t mS){
    myTicks = 0;
    while(myTicks<mS);
}

drv8303 drv;

int main(void)
{   
    // SystemInit();
    SystemClock_Config();

    UART1_Init();  printf("\nuart initialized\n");
    SPI1_Init();   printf("spi initialized\n");
    GPIO_Init();   printf("gpio initialized\n");
    TIM1_Init();   printf("tim1 initialized\n");
    ADC1_Init();    printf("ADC1 initialized\n");
    
    // printf("RCC->CR\t\t");          print_reg(RCC->CR,     32);

    pin_reset(GPIOA, DC_CAL);
    printf("DC_CAL set LOW (off)\n");
    pin_set(GPIOA, EN_GATE);
    printf("\nEnabled DRV8303\n");

    uint8_t tc[3] = "abc";
    // transmit_byte_uart1(*tc);
    // transmit_byte_uart1('\n');
    transmit_uart1(tc, 3);

    uint16_t tx;
    uint16_t rx;

    // sets OC_ADJ_SET to 24 (Vds = 1.043v)
    // Disable OCP
    // sets drv to 3-PWM mode
    uint16_t set_reg = 0b01000000000 | 0b00000110000 | 0b00000001100;
    drv_write(0x02, set_reg);

    // rx = drv_read(0x00);
    // printf("Status register 1\t");  print_reg(rx, 16);
    // myDelay(10);
    // rx = drv_read(0x01);
    // printf("Status register 2\t");  print_reg(rx, 16);
    // myDelay(10);
    rx = drv_read(0x02);
    printf("Control register 1\t");  print_reg(rx, 16);
    // myDelay(10);
    // rx = drv_read(0x03);
    // printf("Control register 2\t");  print_reg(rx, 16);
    // myDelay(10);

    printf("\n");
    uint16_t t_del = 50;
    TIM1_enable();


    uint32_t angle_a = 0;
    uint32_t angle_b = 0;
    uint32_t angle_c = 0;
    uint32_t max_duty_cycle = 7500;
    float duty_cycle_a;
    float duty_cycle_b;
    float duty_cycle_c;
    float angle_step = 30;
    
    
    while (1){
        // pin_set(GPIOA, LED);
        // myDelay(1000);
        // pin_reset(GPIOA, LED);
        // myDelay(200);
        // printf("beep..\n");
        // fflush(stdout);
        
        // pin_set(GPIOA, INH_A);
        // pin_set(GPIOB, INL_A);
        // myDelay(5);
        // pin_reset(GPIOA, INH_A);
        // pin_reset(GPIOB, INL_A);
        // myDelay(5);
        // myDelay(t_del);
        // pin_reset(GPIOA, INH_A); // 1
        // pin_reset(GPIOB, INL_A);
        // pin_set(GPIOA, INH_B);
        // pin_reset(GPIOB, INL_B);
        // pin_reset(GPIOA, INH_C);
        // pin_set(GPIOB, INL_C);
        // printf("1\n");
        // myDelay(t_del);
        // pin_reset(GPIOA, INH_A); // 2
        // pin_set(GPIOB, INL_A);
        // pin_set(GPIOA, INH_B);
        // pin_reset(GPIOB, INL_B);
        // pin_reset(GPIOA, INH_C);
        // pin_reset(GPIOB, INL_C);
        // printf("2\n");
        // myDelay(t_del);
        // pin_reset(GPIOA, INH_A); // 3
        // pin_set(GPIOB, INL_A);
        // pin_reset(GPIOA, INH_B);
        // pin_reset(GPIOB, INL_B);
        // pin_set(GPIOA, INH_C);
        // pin_reset(GPIOB, INL_C);
        // printf("3\n");
        // myDelay(t_del);
        // pin_reset(GPIOA, INH_A); // 4
        // pin_reset(GPIOB, INL_A);
        // pin_reset(GPIOA, INH_B);
        // pin_set(GPIOB, INL_B);
        // pin_set(GPIOA, INH_C);
        // pin_reset(GPIOB, INL_C);
        // printf("4\n");
        // myDelay(t_del);
        // pin_set(GPIOA, INH_A); // 5
        // pin_reset(GPIOB, INL_A);
        // pin_reset(GPIOA, INH_B);
        // pin_set(GPIOB, INL_B);
        // pin_reset(GPIOA, INH_C);
        // pin_reset(GPIOB, INL_C);
        // printf("5\n");
        // myDelay(t_del);
        // pin_set(GPIOA, INH_A); // 6
        // pin_reset(GPIOB, INL_A);
        // pin_reset(GPIOA, INH_B);
        // pin_reset(GPIOB, INL_B);
        // pin_reset(GPIOA, INH_C);
        // pin_set(GPIOB, INL_C);
        // printf("6\n");

        // myDelay(10);
        myDelay(10);
        // rx = drv_read(0x00);
        // printf("Status register 1\t");  print_reg(rx, 16);
        // myDelay(1);
        // rx = drv_read(0x01);
        // printf("Status register 2\t");  print_reg(rx, 16);
        // myDelay(1);
        // rx = drv_read(0x02);
        // printf("Control register 1\t");  print_reg(rx, 16);
        // myDelay(1);
        // rx = drv_read(0x03);
        // printf("Control register 2\t");  print_reg(rx, 16);

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


        angle_b = angle_a + 120;
        angle_c = angle_a + 240;

        duty_cycle_a = ((sin(angle_a * (PI/180))+1)/2)*max_duty_cycle;
        duty_cycle_b = ((sin(angle_b * (PI/180))+1)/2)*max_duty_cycle;
        duty_cycle_c = ((sin(angle_c * (PI/180))+1)/2)*max_duty_cycle;
        // global_angle = fmod(global_angle, 360);
        angle_a =  angle_a + angle_step;
        angle_a %= 360;
        // printf("%d, %d, %d, %d\n", (uint32_t)duty_cycle_a, (uint32_t)duty_cycle_b, (uint32_t)duty_cycle_c, angle_a);

        TIM1->CCR1  = duty_cycle_a;
        TIM1->CCR2  = duty_cycle_b;
        TIM1->CCR3  = duty_cycle_c;

    }



}

/**@brief System Clock Configuration*/
void SystemClock_Config(void)
{
    
    RCC->CR |= (1 << 0);          // HSI enable
    while(!(RCC->CR & (1<<1)));  // wait for HSI to stabilise
    // calibrate
    // RCC->CFGR &= ~(0b1110 << 18);  // PLLMUL = x16
    RCC->CFGR = (RCC->CFGR & (~RCC_CFGR_PLLMULL16))| RCC_CFGR_PLLMULL16;  // PLLMUL = x16
    RCC->CFGR &= ~RCC_CFGR_PLLSRC;       // PLLSOURCE = HSI/2
    RCC->CR |= (1 << 24);         // PLL enable
    while(!(RCC->CR & (1<<25))); // wait for PLL to stabilise

     // increase FLASH latency before setting SYSCLK > 24MHz
    FLASH->ACR = (FLASH->ACR & (~FLASH_ACR_LATENCY)) | FLASH_ACR_LATENCY_2;

    RCC->CFGR |= RCC_CFGR_HPRE_DIV1; // Set AHB prescaler to SYSLCK/1
    RCC->CFGR = (RCC->CFGR & (~RCC_CFGR_SW_Msk)) | RCC_CFGR_SW_PLL;    // Set SYSCLCK to PLL
    while(!((RCC->CR & RCC_CR_PLLRDY) && (RCC->CR & RCC_CR_HSIRDY)));
    while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

    RCC->CFGR = (RCC->CFGR & (~RCC_CFGR_PPRE1_Msk)) | RCC_CFGR_PPRE1_DIV2;  // APB1 = HCLK/2
    RCC->CFGR = (RCC->CFGR & (~RCC_CFGR_PPRE2_Msk)) | RCC_CFGR_PPRE2_DIV1;  // APB2 = HCLK/1

    // SystemCoreClock = 64000000;
    SystemCoreClockUpdate();
    
    /*Configure the Systick interrupt time*/
    SysTick_Config(64000);
    NVIC_SetPriority(SysTick_IRQn, 0);



}

static void GPIO_Init(void)
{
    /* GPIO Ports Clock Enable */
    RCC->APB2ENR |= (1 << 0); // enable AFIO
    RCC->APB2ENR |= (1 << 2); // enable GPIOA
    RCC->APB2ENR |= (1 << 3); // enable GPIOB
    RCC->APB2ENR |= (1 << 4); // enable GPIOC
    RCC->APB2ENR |= (1 << 5); // enable GPIOD

    pin_mode(GPIOA, nOCTW,     GPIO_IN_PULL);
    pin_pullmode(GPIOA, nOCTW, GPIO_PULLUP);
    pin_mode(GPIOB, nFAULT,    GPIO_IN_PULL);
    pin_pullmode(GPIOB, nFAULT, GPIO_PULLUP);
    
    pin_mode(GPIOA, DC_CAL,    GPIO_OUT_PP);
    pin_mode(GPIOA, EN_GATE,   GPIO_OUT_PP);
    pin_mode(GPIOA, INH_A,     GPIO_AF_PP);
    pin_mode(GPIOA, INH_B,     GPIO_AF_PP);
    pin_mode(GPIOA, INH_C,     GPIO_AF_PP);
    pin_mode(GPIOB, LED_FAULT, GPIO_OUT_PP);
    pin_mode(GPIOB, INL_A,     GPIO_AF_PP);
    pin_mode(GPIOB, INL_B,     GPIO_OUT_PP);
    pin_mode(GPIOB, INL_C,     GPIO_OUT_PP);
    pin_mode(GPIOA, LED,       GPIO_OUT_PP);
}