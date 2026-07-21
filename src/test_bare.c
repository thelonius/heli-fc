/* Bare minimum: just toggle PA6 forever */
#include <stdint.h>

#define RCC_AHBENR   (*(volatile uint32_t *)0x40021014)
#define GPIOA_MODER  (*(volatile uint32_t *)0x48000000)
#define GPIOA_ODR    (*(volatile uint32_t *)0x48000014)
#define IWDG_KR      (*(volatile uint32_t *)0x40003000)

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
    IWDG_KR = 0xAAAA;
    RCC_AHBENR |= (1 << 17);
    GPIOA_MODER = (GPIOA_MODER & ~(3UL << 12)) | (1UL << 12);
    while (1) {
        IWDG_KR = 0xAAAA;
        GPIOA_ODR ^= (1UL << 6);
        for (volatile int i = 0; i < 500000; i++);
    }
}
