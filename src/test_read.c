/* Check MARCSTATE + RXBYTES */
#include <stdint.h>

#define RCC_AHBENR   (*(volatile uint32_t *)0x40021014)
#define GPIOA_MODER  (*(volatile uint32_t *)0x48000000)
#define GPIOA_AFRL   (*(volatile uint32_t *)0x48000020)
#define GPIOA_IDR    (*(volatile uint32_t *)0x48000010)
#define GPIOA_ODR    (*(volatile uint32_t *)0x48000014)
#define GPIOA_BSRR   (*(volatile uint32_t *)0x48000018)
#define GPIOB_MODER  (*(volatile uint32_t *)0x48000400)
#define GPIOB_AFRL   (*(volatile uint32_t *)0x48000420)
#define TIM2_PSC     (*(volatile uint32_t *)0x40000028)
#define TIM2_ARR     (*(volatile uint32_t *)0x4000002C)
#define TIM2_CCMR1   (*(volatile uint32_t *)0x40000018)
#define TIM2_CCMR2   (*(volatile uint32_t *)0x4000001C)
#define TIM2_CCER    (*(volatile uint32_t *)0x40000020)
#define TIM2_CCR1    (*(volatile uint32_t *)0x40000034)
#define TIM2_CCR2    (*(volatile uint32_t *)0x40000038)
#define TIM2_CCR3    (*(volatile uint32_t *)0x4000003C)
#define TIM2_CCR4    (*(volatile uint32_t *)0x40000040)
#define TIM2_CR1     (*(volatile uint32_t *)0x40000000)
#define TIM3_PSC     (*(volatile uint32_t *)0x40000428)
#define TIM3_ARR     (*(volatile uint32_t *)0x4000042C)
#define TIM3_CCMR2   (*(volatile uint32_t *)0x4000041C)
#define TIM3_CCER    (*(volatile uint32_t *)0x40000420)
#define TIM3_CCR3    (*(volatile uint32_t *)0x4000043C)
#define TIM3_CR1     (*(volatile uint32_t *)0x40000400)
#define IWDG_KR      (*(volatile uint32_t *)0x40003000)

#define CS   4
#define SCK  5
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
static void delay(volatile uint32_t n) { while (n--) wdg(); }

static uint8_t xfer(uint8_t d) {
    uint8_t r = 0;
    for (int i = 7; i >= 0; i--) {
        if (d & (1<<i)) GPIOA_ODR |= (1UL<<MOSI); else GPIOA_ODR &= ~(1UL<<MOSI);
        delay(1);
        GPIOA_ODR |= (1UL<<SCK);
        delay(1);
        if (GPIOA_IDR & (1UL<<MISO)) r |= (1<<i);
        GPIOA_ODR &= ~(1UL<<SCK);
        delay(1);
    }
    return r;
}

static void to_spi(void) { GPIOA_MODER &= ~((3UL<<12)|(3UL<<14)); GPIOA_MODER |= (1UL<<14); }
static void to_led(void) { GPIOA_MODER &= ~((3UL<<12)|(3UL<<14)); GPIOA_MODER |= ((1UL<<12)|(1UL<<14)); }
static void cs_low(void) { GPIOA_ODR &= ~(1UL<<CS); int t=500; while((GPIOA_IDR&(1UL<<MISO))&&--t); }
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

    /* PA0-PA3 = AF1 (TIM2), PA4,5=output, PA6=input, PA7=output */
    uint32_t m = GPIOA_MODER;
    m &= ~((3<<0)|(3<<2)|(3<<4)|(3<<6));
    m |= ((2<<0)|(2<<2)|(2<<4)|(2<<6));
    GPIOA_MODER = m;
    GPIOA_AFRL = (GPIOA_AFRL & ~0xFFFFu) | 0x1111u;
    GPIOA_MODER |= (1UL<<8)|(1UL<<10)|(1UL<<14);
    GPIOA_ODR |= (1UL<<CS);

    /* PB0 = AF1 (TIM3 CH3) */
    GPIOB_MODER = (GPIOB_MODER & ~3u) | 2u;
    GPIOB_AFRL = (GPIOB_AFRL & ~0xFu) | 1u;

    /* TIM2: 125Hz */
    TIM2_PSC = 63; TIM2_ARR = 999;
    TIM2_CCMR1 = 0x6060; TIM2_CCMR2 = 0x6060;
    TIM2_CCER = 0x1111;
    TIM2_CCR1 = 1500; TIM2_CCR2 = 1500; TIM2_CCR3 = 1500; TIM2_CCR4 = 1500;
    TIM2_CR1 = 0x81;

    /* TIM3: 125Hz */
    TIM3_PSC = 63; TIM3_ARR = 999;
    TIM3_CCMR2 = 0x0068; TIM3_CCER = 0x0100;
    TIM3_CCR3 = 1000;
    TIM3_CR1 = 0x81;

    delay(50000);

    /* Write CC2500 config */
    static const uint8_t cfg[][2] = {
        {0, 7}, {1, 0x2e}, {2, 7}, {3, 0xf}, {4, 0xd3}, {5, 0x91}, {6, 0xd}, {7, 4},
        {8, 0xc}, {9, 0x81}, {10, 0}, {0xb, 6}, {0xc, 0}, {0x10, 0x2c}, {0x11, 0x43},
        {0x12, 3}, {0x13, 0x23}, {0x14, 0x7a}, {0x15, 0x44}, {0x16, 7}, {0x17, 0xc},
        {0x18, 8}, {0x19, 0x1d}, {0x1a, 0x1c}, {0x1b, 0x43}, {0x1c, 0x40}, {0x1d, 0x91},
        {0x1e, 0x57}, {0x1f, 0x6b}, {0x20, 0xf8}, {0x21, 0xb6}, {0x22, 0x10}, {0x23, 0xea},
        {0x24, 10}, {0x25, 0x11}, {0x26, 0x11}, {0x2c, 0x88}, {0x2d, 0x31}, {0x2e, 0xb}
    };
    for (int i = 0; i < sizeof(cfg)/2; i++) cc_write(cfg[i][0], cfg[i][1]);

    /* SRX: enable receive */
    cc_cmd(0x34);
    delay(50000);

    /* Read MARCSTATE (0x35) and RXBYTES (0x3B) */
    uint8_t marc = cc_read(0x35 | 0x40);
    uint8_t rxbytes = cc_read(0x3B | 0x40) & 0x7F;

    /* PA0 = MARCSTATE × 100 */
    TIM2_CCR1 = (marc & 0x1F) * 100;
    /* PA1 = RXBYTES × 100 */
    TIM2_CCR2 = rxbytes * 100;
    /* PA2 = MARCSTATE raw × 50 */
    TIM2_CCR3 = marc * 50;
    /* PB0 = RXBYTES × 100 */
    TIM3_CCR3 = rxbytes * 100;

    while (1) wdg();
}
