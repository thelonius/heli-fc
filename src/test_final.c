/* LED + SPI only, no timers. Shows MARCSTATE via blink count. */
#include <stdint.h>

#define RCC_AHBENR   (*(volatile uint32_t *)0x40021014)
#define GPIOA_MODER  (*(volatile uint32_t *)0x48000000)
#define GPIOA_IDR    (*(volatile uint32_t *)0x48000010)
#define GPIOA_ODR    (*(volatile uint32_t *)0x48000014)
#define IWDG_KR      (*(volatile uint32_t *)0x40003000)

#define CS 4
#define SCK 5
#define MISO 6
#define MOSI 7

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

static void wdg(void) { IWDG_KR = 0xAAAA; }
static void dly(volatile uint32_t n) { while (n--) wdg(); }

static void red_on(void)  { GPIOA_ODR |= (1UL << 7); }
static void red_off(void) { GPIOA_ODR &= ~(1UL << 7); }
static void blue_on(void)  { GPIOA_ODR |= (1UL << 6); }
static void blue_off(void) { GPIOA_ODR &= ~(1UL << 6); }

static void blink_red(int n) {
    for (int i = 0; i < n; i++) { red_on(); dly(30000); red_off(); dly(30000); }
    dly(50000);
}

static uint8_t xfer(uint8_t d) {
    uint8_t r = 0;
    for (int i = 7; i >= 0; i--) {
        if (d & (1<<i)) GPIOA_ODR |= (1UL<<MOSI); else GPIOA_ODR &= ~(1UL<<MOSI);
        dly(1);
        GPIOA_ODR |= (1UL<<SCK);
        dly(1);
        if (GPIOA_IDR & (1UL<<MISO)) r |= (1<<i);
        GPIOA_ODR &= ~(1UL<<SCK);
        dly(1);
    }
    return r;
}

static void to_spi(void) {
    GPIOA_MODER &= ~((3UL<<12)|(3UL<<14));
    GPIOA_MODER |= (1UL<<14);
}
static void to_led(void) {
    GPIOA_MODER &= ~((3UL<<12)|(3UL<<14));
    GPIOA_MODER |= ((1UL<<12)|(1UL<<14));
}
static void cs_low(void) {
    GPIOA_ODR &= ~(1UL<<CS);
    int t=500; while((GPIOA_IDR&(1UL<<MISO))&&--t);
}
static void cs_high(void) { GPIOA_ODR |= (1UL<<CS); }

static void cc_write(uint8_t addr, uint8_t val) {
    to_spi(); cs_low(); xfer(addr); xfer(val); cs_high(); to_led();
}
static void cc_cmd(uint8_t cmd) {
    to_spi(); cs_low(); xfer(cmd); cs_high(); to_led();
}
static uint8_t cc_read(uint8_t addr) {
    to_spi(); cs_low(); xfer(addr|0x80); uint8_t v=xfer(0); cs_high(); to_led(); return v;
}

void Reset_Handler(void) {
    RCC_AHBENR |= (1<<17);
    wdg();

    /* PA4,PA5=output; PA6=input(MISO); PA7=output(MOSI) */
    GPIOA_MODER |= (1UL<<8)|(1UL<<10)|(1UL<<14);
    GPIOA_ODR |= (1UL<<CS);
    to_led();

    dly(30000);
    blink_red(2);

    /* Write CC2500 config */
    static const uint8_t cfg[][2] = {
        {0, 7}, {1, 0x2e}, {2, 7}, {3, 0xf}, {4, 0xd3}, {5, 0x91}, {6, 0xd}, {7, 4},
        {8, 0xc}, {9, 0x81}, {10, 0}, {0xb, 6}, {0xc, 0}, {0x10, 0x2c}, {0x11, 0x43},
        {0x12, 3}, {0x13, 0x23}, {0x14, 0x7a}, {0x15, 0x44}, {0x16, 7}, {0x17, 0xc},
        {0x18, 8}, {0x19, 0x1d}, {0x1a, 0x1c}, {0x1b, 0x43}, {0x1c, 0x40}, {0x1d, 0x91},
        {0x1e, 0x57}, {0x1f, 0x6b}, {0x20, 0xf8}, {0x21, 0xb6}, {0x22, 0x10}, {0x23, 0xea},
        {0x24, 10}, {0x25, 0x11}, {0x26, 0x11}, {0x2c, 0x88}, {0x2d, 0x31}, {0x2e, 0xb}
    };
    for (unsigned i = 0; i < sizeof(cfg)/sizeof(cfg[0]); i++) cc_write(cfg[i][0], cfg[i][1]);
    blink_red(3);

    /* SRX */
    cc_cmd(0x34);
    dly(30000);
    blink_red(2);

    /* Read MARCSTATE */
    uint8_t marc = cc_read(0x35 | 0x40);
    uint8_t rxbytes = cc_read(0x3B | 0x40) & 0x7F;

    /* Show MARCSTATE: blue blinks N times where N = state */
    for (int i = 0; i < (marc & 0x1F); i++) {
        blue_on(); dly(20000); blue_off(); dly(20000);
    }
    dly(50000);

    /* Show RXBYTES: red blinks N times */
    for (int i = 0; i < rxbytes; i++) {
        red_on(); dly(20000); red_off(); dly(20000);
    }
    dly(50000);

    /* Final: blue=marcstate, red=rxbytes steady */
    if (marc & 0x1F) blue_on(); else blue_off();
    if (rxbytes) red_on(); else red_off();

    while (1) wdg();
}
