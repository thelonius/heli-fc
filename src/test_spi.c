/* Test SPI bit-bang + CC2500 read */
#include <stdint.h>

#define RCC_AHBENR   (*(volatile uint32_t *)0x40021014)
#define GPIOA_MODER  (*(volatile uint32_t *)0x48000000)
#define GPIOA_IDR    (*(volatile uint32_t *)0x48000010)
#define GPIOA_ODR    (*(volatile uint32_t *)0x48000014)
#define GPIOA_BSRR   (*(volatile uint32_t *)0x48000018)
#define IWDG_KR      (*(volatile uint32_t *)0x40003000)

#define CS_PIN   4
#define SCK_PIN  5
#define MISO_PIN 6
#define MOSI_PIN 7

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

static void led_on(uint8_t pin) { GPIOA_BSRR = (1UL << pin); }
static void led_off(uint8_t pin) { GPIOA_BSRR = (1UL << (pin + 16)); }

static void blink(uint8_t pin, int count) {
    for (int i = 0; i < count; i++) {
        led_on(pin);
        delay(300000);
        led_off(pin);
        delay(300000);
    }
}

static uint8_t spi_transfer(uint8_t data) {
    uint8_t received = 0;
    for (int i = 7; i >= 0; i--) {
        if (data & (1 << i)) GPIOA_ODR |= (1UL << MOSI_PIN);
        else                  GPIOA_ODR &= ~(1UL << MOSI_PIN);
        for (volatile int d = 0; d < 5; d++);
        GPIOA_ODR |= (1UL << SCK_PIN);
        for (volatile int d = 0; d < 5; d++);
        if (GPIOA_IDR & (1UL << MISO_PIN)) received |= (1 << i);
        GPIOA_ODR &= ~(1UL << SCK_PIN);
        for (volatile int d = 0; d < 5; d++);
    }
    return received;
}

static void spi_to_gpio(void) {
    /* PA6=output, PA7=output for LED */
    GPIOA_MODER &= ~((3UL << 12) | (3UL << 14));
    GPIOA_MODER |=  ((1UL << 12) | (1UL << 14));
}

static void spi_to_bus(void) {
    /* PA6=input(MISO), PA7=output(MOSI) */
    GPIOA_MODER &= ~((3UL << 12) | (3UL << 14));
    GPIOA_MODER |=  (1UL << 14);
}

/* CC2500: wait for SO low after CS low */
static void wait_so_low(void) {
    int timeout = 1000;
    while ((GPIOA_IDR & (1UL << MISO_PIN)) && --timeout);
}

/* Read one CC2500 register */
static uint8_t cc2500_read_reg(uint8_t addr) {
    spi_to_bus();
    GPIOA_ODR &= ~(1UL << CS_PIN);
    wait_so_low();
    spi_transfer(addr | 0x80);  /* read, single */
    uint8_t val = spi_transfer(0x00);
    GPIOA_ODR |= (1UL << CS_PIN);
    spi_to_gpio();
    return val;
}

void Reset_Handler(void) {
    IWDG_KR = 0xAAAA;
    RCC_AHBENR |= (1 << 17);  /* GPIOA enable */

    /* PA4,PA5,PA7 = output; PA6 = input initially */
    GPIOA_MODER &= ~((3UL<<8)|(3UL<<10)|(3UL<<12)|(3UL<<14));
    GPIOA_MODER |=  ((1UL<<8)|(1UL<<10)|(1UL<<14));
    /* CS high (idle) */
    GPIOA_ODR |= (1UL << CS_PIN);

    delay(100000);

    /* Step 1: Read PARTNUM (reg 0x30) and VERSION (reg 0x31) */
    uint8_t partnum = cc2500_read_reg(0x30);
    uint8_t version = cc2500_read_reg(0x31);

    /* Step 2: Show results via blink pattern */
    /* Blue blinks = partnum high nibble */
    blink(MISO_PIN, (partnum >> 4) & 0xF);
    delay(500000);
    /* Red blinks = partnum low nibble */
    blink(MOSI_PIN, partnum & 0xF);
    delay(500000);
    /* Blue blinks = version high nibble */
    blink(MISO_PIN, (version >> 4) & 0xF);
    delay(500000);
    /* Red blinks = version low nibble */
    blink(MOSI_PIN, version & 0xF);
    delay(1000000);

    /* Step 3: If partnum==0 and version==0, SPI not working → fast blink both */
    if (partnum == 0 && version == 0) {
        while (1) {
            led_on(MISO_PIN); led_on(MOSI_PIN);
            delay(200000);
            led_off(MISO_PIN); led_off(MOSI_PIN);
            delay(200000);
        }
    }

    /* Step 4: SPI works → slow heartbeat on red, blue stays on */
    led_on(MISO_PIN);
    while (1) {
        IWDG_KR = 0xAAAA;
        led_on(MOSI_PIN);
        delay(500000);
        led_off(MOSI_PIN);
        delay(500000);
    }
}
