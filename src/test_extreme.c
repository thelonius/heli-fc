#include "stm32f031.h"

void Reset_Handler(void) {
    // No stack init, no data copy, just raw registers
    RCC_AHBENR |= RCC_AHBENR_GPIOAEN;
    GPIOA_MODER &= ~(3 << (5 * 2));
    GPIOA_MODER |= (1 << (5 * 2));

    while(1) {
        GPIOA_ODR ^= (1 << 5);
        for(volatile int i = 0; i < 100000; i++);
    }
}

__attribute__((section(".isr_vector")))
void (*const vector_table[])() = {
    (void (*)())0x20001000, // Stack pointer
    Reset_Handler,          // Reset
};
