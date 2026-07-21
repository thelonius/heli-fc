/* Step-by-step SPI test */
#include <stdint.h>

#define RCC_AHBENR   (*(volatile uint32_t *)0x40021014)
#define GPIOA_MODER  (*(volatile uint32_t *)0x48000000)
#define GPIOA_IDR    (*(volatile uint32_t *)0x48000010)
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

static void delay(uint32_t n) {
    for (volatile uint32_t i = 0; i < n; i++) {
        IWDG_KR = 0xAAAA;
    }
}

static void blink_red(int n) {
    for (int i = 0; i < n; i++) {
        GPIOA_ODR |= (1UL << 7);
        delay(40000);
        GPIOA_ODR &= ~(1UL << 7);
        delay(40000);
    }
    delay(60000);
}

void Reset_Handler(void) {
    IWDG_KR = 0xAAAA;
    RCC_AHBENR |= (1 << 17);

    /* All pins default input */
    /* PA7 = output for red LED, PA6 = input for MISO */
    GPIOA_MODER = (GPIOA_MODER & ~(3UL << 14)) | (1UL << 14);
    /* PA4=CS=high, PA5=SCK=low */
    GPIOA_ODR |= (1UL << 4);

    delay(20000);
    blink_red(2);  /* "I'm alive" */

    /* PA4,PA5 = output */
    GPIOA_MODER |= (1UL << 8) | (1UL << 10);

    delay(20000);
    blink_red(3);  /* "GPIO configured" */

    /* CS low */
    GPIOA_ODR &= ~(1UL << 4);
    delay(10);
    blink_red(2);  /* "CS low" */

    /* Check if MISO goes low (CC2500 crystal running) */
    int so_low = !(GPIOA_IDR & (1UL << 6));
    if (so_low) blink_red(3);
    else         blink_red(1);

    /* CS high */
    GPIOA_ODR |= (1UL << 4);
    delay(10);
    blink_red(2);  /* "CS high" */

    /* Try SPI: send 0x30|0x80 = 0xB0 (read PARTNUM) */
    uint8_t rx = 0;
    GPIOA_ODR &= ~(1UL << 4);  /* CS low */
    delay(10);

    uint8_t tx = 0xB0;
    for (int i = 7; i >= 0; i--) {
        if (tx & (1 << i)) GPIOA_ODR |= (1UL << 7);
        else                GPIOA_ODR &= ~(1UL << 7);
        delay(5);
        GPIOA_ODR |= (1UL << 5);   /* SCK high */
        delay(5);
        if (GPIOA_IDR & (1UL << 6)) rx |= (1 << i);
        GPIOA_ODR &= ~(1UL << 5);  /* SCK low */
        delay(5);
    }

    GPIOA_ODR |= (1UL << 4);  /* CS high */

    /* Show rx value: blink red N times for each nibble */
    blink_red((rx >> 4) & 0xF);
    delay(40000);
    blink_red(rx & 0xF);

    /* Stay: red heartbeat */
    while (1) {
        IWDG_KR = 0xAAAA;
        GPIOA_ODR |= (1UL << 7);
        delay(50000);
        GPIOA_ODR &= ~(1UL << 7);
        delay(50000);
    }
}
