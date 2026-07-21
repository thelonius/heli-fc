#include "stm32f031.h"
#include <stdint.h>

void Reset_Handler(void) {
    /* Minimal Setup */
    RCC_AHBENR |= RCC_AHBENR_GPIOAEN;
    
    /* PA5 as Output */
    GPIOA_MODER &= ~(3 << (5 * 2));
    GPIOA_MODER |= (1 << (5 * 2));

    while(1) {
        GPIOA_ODR ^= (1 << 5);
        for(volatile int i = 0; i < 100000; i++);
    }
}

__attribute__((section(".isr_vector")))
void (*const vector_table[])() = {
    (void (*)())0x20008000, // Stack pointer
    Reset_Handler,          // Reset
};
