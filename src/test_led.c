/* Minimal test: just blink PA6 (blue) and PA7 (red) */
#include <stdint.h>

#define RCC_AHBENR   (*(volatile uint32_t *)0x40021014)
#define GPIOA_MODER  (*(volatile uint32_t *)0x48000000)
#define GPIOA_ODR    (*(volatile uint32_t *)0x48000014)
#define GPIOA_BSRR   (*(volatile uint32_t *)0x48000018)
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

static void delay(uint32_t n) {
    for (volatile uint32_t i = 0; i < n; i++);
}

void Reset_Handler(void) {
    IWDG_KR = 0xAAAA;
    RCC_AHBENR |= (1 << 17);  /* GPIOA enable */

    /* PA6, PA7 = output */
    GPIOA_MODER &= ~((3UL << 12) | (3UL << 14));
    GPIOA_MODER |=  ((1UL << 12) | (1UL << 14));

    /* Step 1: both ON */
    GPIOA_BSRR = (1 << 6) | (1 << 7);
    delay(2000000);

    /* Step 2: both OFF */
    GPIOA_BSRR = (1 << 22) | (1 << 23);
    delay(2000000);

    /* Step 3: blue ON, red OFF */
    GPIOA_BSRR = (1 << 6) | (1 << 23);
    delay(2000000);

    /* Step 4: blue OFF, red ON */
    GPIOA_BSRR = (1 << 22) | (1 << 7);
    delay(2000000);

    /* Step 5: blink blue 3 times */
    for (int i = 0; i < 3; i++) {
        GPIOA_BSRR = (1 << 6);
        delay(500000);
        GPIOA_BSRR = (1 << 22);
        delay(500000);
    }

    /* Step 6: blink red 3 times */
    for (int i = 0; i < 3; i++) {
        GPIOA_BSRR = (1 << 7);
        delay(500000);
        GPIOA_BSRR = (1 << 23);
        delay(500000);
    }

    /* Stay with both off */
    while (1) {
        IWDG_KR = 0xAAAA;
    }
}
