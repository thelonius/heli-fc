#include <stdint.h>

// Base Addresses
#define RCC_BASE          0x40021000
#define GPIOA_BASE        0x48000000
#define GPIOB_BASE        0x48000400

#define RCC_AHBENR        (*(volatile uint32_t *)(RCC_BASE + 0x14))
#define GPIOA_MODER       (*(volatile uint32_t *)(GPIOA_BASE + 0x00))
#define GPIOA_ODR         (*(volatile uint32_t *)(GPIOA_BASE + 0x14))
#define GPIOB_MODER       (*(volatile uint32_t *)(GPIOB_BASE + 0x00))
#define GPIOB_ODR         (*(volatile uint32_t *)(GPIOB_BASE + 0x14))

void Reset_Handler(void) {
    // 1. Enable GPIOA and GPIOB clocks
    RCC_AHBENR |= (1 << 0) | (1 << 1);

    // 2. Set PB6, PB7, and PA5 as outputs
    GPIOB_MODER &= ~((3 << (6 * 2)) | (3 << (7 * 2)));
    GPIOB_MODER |=  ((1 << (6 * 2)) | (1 << (7 * 2)));
    
    GPIOA_MODER &= ~(3 << (5 * 2));
    GPIOA_MODER |=  (1 << (5 * 2));

    while (1) {
        // Toggle LEDs
        GPIOB_ODR ^= (1 << 6);
        GPIOB_ODR ^= (1 << 7);
        GPIOA_ODR ^= (1 << 5);

        // Very crude delay
        for (volatile int i = 0; i < 500000; i++);
    }
}

// Minimal Vector Table
__attribute__((section(".isr_vector")))
uint32_t vector_table[] = {
    0x20001000,       // Stack Pointer (top of 4KB SRAM)
    (uint32_t)Reset_Handler // Reset Vector
};
