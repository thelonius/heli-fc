/* Simplest SPI test: just read one register, show result on LEDs */
#include <stdint.h>

#define RCC_AHBENR   (*(volatile uint32_t *)0x40021014)
#define GPIOA_MODER  (*(volatile uint32_t *)0x48000000)
#define GPIOA_IDR    (*(volatile uint32_t *)0x48000010)
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

static void delay(volatile uint32_t n) {
    while (n--) IWDG_KR = 0xAAAA;
}

static uint8_t spi_read_one(uint8_t addr) {
    /* PA6=input(MISO), PA7=output(MOSI) */
    GPIOA_MODER = (GPIOA_MODER & ~((3UL<<12)|(3UL<<14))) | (1UL<<14);
    /* CS low */
    GPIOA_ODR &= ~(1UL << 4);
    delay(10);
    /* Wait for SO low */
    int t = 500;
    while ((GPIOA_IDR & (1UL << 6)) && --t);
    /* Send header: addr|0x80 (read) */
    uint8_t tx = addr | 0x80;
    uint8_t rx = 0;
    for (int i = 7; i >= 0; i--) {
        if (tx & (1<<i)) GPIOA_ODR |= (1UL<<7); else GPIOA_ODR &= ~(1UL<<7);
        delay(2);
        GPIOA_ODR |= (1UL<<5);  /* SCK high */
        delay(2);
        if (GPIOA_IDR & (1UL<<6)) rx |= (1<<i);
        GPIOA_ODR &= ~(1UL<<5); /* SCK low */
        delay(2);
    }
    /* CS high */
    GPIOA_ODR |= (1UL << 4);
    /* Back to GPIO for LEDs */
    GPIOA_MODER = (GPIOA_MODER & ~((3UL<<12)|(3UL<<14))) | ((1UL<<12)|(1UL<<14));
    return rx;
}

void Reset_Handler(void) {
    RCC_AHBENR |= (1 << 17);
    /* PA4,PA5,PA6,PA7 = output initially */
    GPIOA_MODER |= (1UL<<8)|(1UL<<10)|(1UL<<12)|(1UL<<14);
    /* CS high, SCK low */
    GPIOA_ODR = (1UL << 4);

    delay(100000);

    /* Read PARTNUM (0x30) and VERSION (0x31) */
    uint8_t partnum = spi_read_one(0x30);
    uint8_t version = spi_read_one(0x31);

    /* Show on LEDs:
     * Blue ON  = partnum != 0 (SPI works)
     * Blue OFF = partnum == 0 (SPI broken)
     * Red ON   = version != 0
     * Red OFF  = version == 0
     */
    if (partnum) GPIOA_BSRR = (1UL << 6); else GPIOA_BSRR = (1UL << 22);
    if (version) GPIOA_BSRR = (1UL << 7); else GPIOA_BSRR = (1UL << 23);

    while (1) IWDG_KR = 0xAAAA;
}
