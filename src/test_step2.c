#include <stdint.h>

#define RCC_AHBENR    (*(volatile uint32_t *)0x40021014)
#define RCC_APB1ENR   (*(volatile uint32_t *)0x4002101C)

#define GPIOA_MODER   (*(volatile uint32_t *)0x48000000)
#define GPIOA_OSPEEDR (*(volatile uint32_t *)0x48000008)
#define GPIOA_ODR     (*(volatile uint32_t *)0x48000014)
#define GPIOA_AFRL    (*(volatile uint32_t *)0x48000020)

#define GPIOB_MODER   (*(volatile uint32_t *)0x48000400)
#define GPIOB_AFRL    (*(volatile uint32_t *)0x48000420)

#define TIM2_CR1      (*(volatile uint32_t *)0x40000000)
#define TIM2_CCMR1    (*(volatile uint32_t *)0x40000018)
#define TIM2_CCMR2    (*(volatile uint32_t *)0x4000001C)
#define TIM2_CCER     (*(volatile uint32_t *)0x40000020)
#define TIM2_PSC      (*(volatile uint32_t *)0x40000028)
#define TIM2_ARR      (*(volatile uint32_t *)0x4000002C)
#define TIM2_CCR1     (*(volatile uint32_t *)0x40000034)
#define TIM2_CCR2     (*(volatile uint32_t *)0x40000038)
#define TIM2_CCR3     (*(volatile uint32_t *)0x4000003C)
#define TIM2_CCR4     (*(volatile uint32_t *)0x40000040)

#define TIM3_CR1      (*(volatile uint32_t *)0x40000400)
#define TIM3_CCMR2    (*(volatile uint32_t *)0x4000041C)
#define TIM3_CCER     (*(volatile uint32_t *)0x40000420)
#define TIM3_PSC      (*(volatile uint32_t *)0x40000428)
#define TIM3_ARR      (*(volatile uint32_t *)0x4000042C)
#define TIM3_CCR3     (*(volatile uint32_t *)0x4000043C)
#define TIM3_CCR4     (*(volatile uint32_t *)0x40000440)

void Default_Handler(void) { while(1); }
void Reset_Handler(void);
__attribute__((section(".isr_vector")))
void (*const vector_table[])() = {
    (void (*)())0x20001000, Reset_Handler,
    Default_Handler, Default_Handler, Default_Handler, Default_Handler,
    Default_Handler, Default_Handler, Default_Handler, Default_Handler,
    Default_Handler, Default_Handler, Default_Handler,
    Default_Handler, Default_Handler, Default_Handler,
};

void Reset_Handler(void) {
    RCC_AHBENR |= (1 << 17) | (1 << 18);
    RCC_APB1ENR |= (1 << 0) | (1 << 1);

    /* PA5 output LED */
    uint32_t m = GPIOA_MODER;
    m &= ~(3u << 10);
    m |=  (1u << 10);
    /* PA0-PA3 = AF (TIM2 CH1-CH4) */
    m &= ~((3u << 0) | (3u << 2) | (3u << 4) | (3u << 6));
    m |=  ((2u << 0) | (2u << 2) | (2u << 4) | (2u << 6));
    GPIOA_MODER = m;
    GPIOA_OSPEEDR |= 0xFF;

    /* PA0-PA3 = AF2 (TIM2) */
    GPIOA_AFRL = (GPIOA_AFRL & ~0xFFFFu) | 0x2222u;

    /* PB0 = AF1 (TIM3_CH3 — Tail, PB0+PB1 joined on board) */
    /* PB1 = Input (must be input to avoid glitch on PB0) */
    GPIOB_MODER = (GPIOB_MODER & ~3u) | 2u;
    GPIOB_AFRL  = (GPIOB_AFRL & ~0xFu) | 1u;

    /* TIM2: 125Hz — Servo1(PA0), Servo2(PA1), Servo3(PA2), Throttle(PA3) */
    TIM2_PSC = 63;
    TIM2_ARR = 999;
    TIM2_CCMR1 = 0x6060;
    TIM2_CCMR2 = 0x6060;
    TIM2_CCER = 0x1111;
    TIM2_CCR1 = 125;
    TIM2_CCR2 = 125;
    TIM2_CCR3 = 125;
    TIM2_CCR4 = 125;
    TIM2_CR1 = 0x81;

    /* TIM3: 125Hz — Tail on PB0 (CH3, PB1=Input) */
    TIM3_PSC = 63;
    TIM3_ARR = 999;
    TIM3_CCMR2 = 0x0068;
    TIM3_CCER = 0x0100;
    TIM3_CCR3 = 125;
    TIM3_CR1 = 0x81;

    while (1) {
        GPIOA_ODR ^= (1u << 5);
        for (volatile int i = 0; i < 1000000; i++);
    }
}
