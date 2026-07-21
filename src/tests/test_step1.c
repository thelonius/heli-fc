#include <stdint.h>

#define RCC_BASE      0x40021000
#define RCC_AHBENR    (*(volatile uint32_t *)(RCC_BASE + 0x14))
#define RCC_APB1ENR   (*(volatile uint32_t *)(RCC_BASE + 0x1C))

#define GPIOA_BASE    0x48000000
#define GPIOA_MODER   (*(volatile uint32_t *)(GPIOA_BASE + 0x00))
#define GPIOA_ODR     (*(volatile uint32_t *)(GPIOA_BASE + 0x14))

#define GPIOB_BASE    0x48000400
#define GPIOB_MODER   (*(volatile uint32_t *)(GPIOB_BASE + 0x00))
#define GPIOB_AFRL    (*(volatile uint32_t *)(GPIOB_BASE + 0x20))

#define TIM3_BASE     0x40000400
#define TIM3_CR1      (*(volatile uint32_t *)(TIM3_BASE + 0x00))
#define TIM3_CCMR2    (*(volatile uint32_t *)(TIM3_BASE + 0x1C))
#define TIM3_CCER     (*(volatile uint32_t *)(TIM3_BASE + 0x20))
#define TIM3_PSC      (*(volatile uint32_t *)(TIM3_BASE + 0x28))
#define TIM3_ARR      (*(volatile uint32_t *)(TIM3_BASE + 0x2C))
#define TIM3_CCR3     (*(volatile uint32_t *)(TIM3_BASE + 0x3C))

void Default_Handler(void) { while(1); }
void Reset_Handler(void);

__attribute__((section(".isr_vector")))
void (*const vector_table[])() = {
    (void (*)())0x20001000,
    Reset_Handler,
    Default_Handler, Default_Handler, Default_Handler, Default_Handler,
    Default_Handler, Default_Handler, Default_Handler, Default_Handler,
    Default_Handler, Default_Handler, Default_Handler,
    Default_Handler, Default_Handler, Default_Handler,
};

void Reset_Handler(void) {
    RCC_AHBENR |= (1 << 17) | (1 << 18);
    RCC_APB1ENR |= (1 << 1);

    GPIOA_MODER &= ~(3u << (5 * 2));
    GPIOA_MODER |=  (1u << (5 * 2));

    GPIOB_MODER &= ~(3u << (0 * 2));
    GPIOB_MODER |=  (2u << (0 * 2));
    GPIOB_AFRL  &= ~(0xFu << 0);
    GPIOB_AFRL  |=  (1u << 0);

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
