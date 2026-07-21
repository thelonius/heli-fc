#include "stm32f031.h"
#include <stdint.h>

extern uint32_t _estack;
extern uint32_t _sbss;
extern uint32_t _ebss;

static volatile uint32_t tick = 0;

void SysTick_Handler(void) {
    tick++;
}

void Default_Handler(void) {
    while(1);
}

void Reset_Handler(void);

__attribute__((section(".isr_vector")))
void (*const vector_table[])() = {
    (void (*)())&_estack,
    Reset_Handler,
    Default_Handler, Default_Handler,
    Default_Handler, Default_Handler, Default_Handler, Default_Handler,
    Default_Handler, Default_Handler, Default_Handler,
    Default_Handler,
    Default_Handler, Default_Handler,
    Default_Handler,
    SysTick_Handler,
};

void Reset_Handler(void) {
    uint32_t *dst = &_sbss;
    while (dst < &_ebss) *dst++ = 0;

    RCC_AHBENR |= (1 << 17) | (1 << 18);
    RCC_APB1ENR |= (1 << 1);

    GPIOB_MODER &= ~(3u << (0 * 2));
    GPIOB_MODER |= (2u << (0 * 2));
    GPIOB_AFRL &= ~(0xFu << (0 * 4));
    GPIOB_AFRL |= (1u << (0 * 4));

    GPIOB_MODER &= ~(3u << (6 * 2));
    GPIOB_MODER |= (1u << (6 * 2));
    GPIOB_MODER &= ~(3u << (7 * 2));
    GPIOB_MODER |= (1u << (7 * 2));

    TIM3_PSC = 47;
    TIM3_ARR = 8000;
    TIM3_CCMR2 = 0x6800;
    TIM3_CCER = 0x0100;
    TIM3_CR1 = 0x81;
    TIM3_CCR3 = 1333;

    STK_RVR = 7999;
    STK_CSR = 7;

    while (1) {
        uint32_t target = tick + 500;
        while (tick < target);
        GPIOB_ODR ^= (1 << 6) | (1 << 7);
    }
}
