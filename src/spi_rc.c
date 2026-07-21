#include "spi_rc.h"

/* This driver runs in the TIM14 radio-poll ISR, so every SPI byte's cost is
 * loop-rate-limiting. Force-inline the bit ops and drop the half-bit pad to 0:
 * at 8MHz the BSRR writes + loop overhead alone clock SCLK well under the
 * CC2500's ~6.5MHz max, so no explicit delay is needed. (Same reasoning as the
 * IMU I2C speedup.) Raise SPI_DELAY_ITERS only if a scope shows SPI errors. */
#define SPI_INLINE static inline __attribute__((always_inline))
#define SPI_DELAY_ITERS 4
SPI_INLINE void spi_delay(void) {
    for (volatile int i = 0; i < SPI_DELAY_ITERS; i++);
}

SPI_INLINE void cs_low(void)  { GPIOA_BSRR = (uint32_t)SPI_CS_PIN << 16; }
SPI_INLINE void cs_high(void) { GPIOA_BSRR = SPI_CS_PIN; }

/* After CS goes low the CC2500 holds SO high until its crystal is running.
 * Keep the full 500 timeout: it's needed for the crystal to settle right after
 * CC2500_Reset(), and cutting it to 64 was measured to leave the chip read back
 * as all-zeros (partnum 0x00 instead of 0x80) — the wake wasn't done yet. In
 * RX/IDLE the crystal is already on, so this returns on the first read and
 * costs nothing during steady polling. */
SPI_INLINE void spi_wait_so_low(void) {
    int timeout = 500;
    while ((GPIOB_IDR & SPI_MISO_PIN) && --timeout);
}

void SPI_RC_Init(void) {
    RCC_AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;

    /* PA8 = CS (Output, idle high) */
    GPIOA_MODER &= ~(3u << (8 * 2));
    GPIOA_MODER |= (1u << (8 * 2));
    cs_high();

    /* PB3 = MISO/SO (Input) */
    GPIOB_MODER &= ~(3u << (3 * 2));

    /* PB4 = SCK (Output), PB5 = MOSI (Output) */
    GPIOB_MODER &= ~((3u << (4 * 2)) | (3u << (5 * 2)));
    GPIOB_MODER |= ((1u << (4 * 2)) | (1u << (5 * 2)));

    GPIOB_BSRR = (uint32_t)(SPI_SCK_PIN | SPI_MOSI_PIN) << 16;
}

/* Atomic BSRR, not GPIOB_ODR read-modify-write: this runs in the TIM14 ISR
 * while the IMU I2C bit-bang runs in the main loop on the SAME port (GPIOB).
 * An RMW here would clobber the other path's concurrent bit change — that race
 * corrupted both buses and read the CC2500 back as garbage (0x72/0xf8 instead
 * of 0x80/0x03). BSRR touches only the named bits, so the disjoint SPI (PB4/5)
 * and I2C (PB6/7) pins never interfere. */
SPI_INLINE uint8_t spi_transfer(uint8_t data) {
    uint8_t received = 0;
    for (int i = 7; i >= 0; i--) {
        if (data & (1 << i)) GPIOB_BSRR = SPI_MOSI_PIN;
        else                 GPIOB_BSRR = (uint32_t)SPI_MOSI_PIN << 16;

        spi_delay();
        GPIOB_BSRR = SPI_SCK_PIN;
        spi_delay();

        if (GPIOB_IDR & SPI_MISO_PIN) received |= (1 << i);

        GPIOB_BSRR = (uint32_t)SPI_SCK_PIN << 16;
        spi_delay();
    }
    return received;
}

uint8_t CC2500_Strobe(uint8_t cmd) {
    cs_low();
    spi_wait_so_low();
    uint8_t status = spi_transfer(cmd);
    cs_high();
    return status;
}

void CC2500_WriteReg(uint8_t addr, uint8_t value) {
    cs_low();
    spi_wait_so_low();
    spi_transfer(addr);
    spi_transfer(value);
    cs_high();
}

uint8_t CC2500_ReadReg(uint8_t addr) {
    cs_low();
    spi_wait_so_low();
    spi_transfer(addr | 0x80);
    uint8_t value = spi_transfer(0x00);
    cs_high();
    return value;
}

/* Status registers 0x30-0x3D require the burst bit */
uint8_t CC2500_ReadStatusReg(uint8_t addr) {
    cs_low();
    spi_wait_so_low();
    spi_transfer(addr | 0xC0);
    uint8_t value = spi_transfer(0x00);
    cs_high();
    return value;
}

/* CC2500 erratum: RXBYTES/TXBYTES may be corrupt if read while the FIFO count
 * is updating. Read until two consecutive reads agree. While a packet is
 * streaming IN, consecutive reads legitimately never agree (the count really
 * is climbing) — that used to burn all 8 re-reads (~350us) per call. But the
 * only question the caller asks is "is a full packet there": once two reads
 * both say >= 15 the answer is yes regardless of the exact count, so accept
 * the smaller one immediately. */
uint8_t CC2500_ReadRxBytes(void) {
    uint8_t a = CC2500_ReadStatusReg(CC2500_RXBYTES);
    uint8_t b;
    for (int i = 0; i < 8; i++) {
        b = CC2500_ReadStatusReg(CC2500_RXBYTES);
        if (a == b) return a;
        if ((a & 0x7F) >= 15 && (b & 0x7F) >= 15)
            return ((a & 0x7F) < (b & 0x7F)) ? a : b;
        a = b;
    }
    return a;
}

void CC2500_WriteBurst(uint8_t addr, const uint8_t *data, uint8_t len) {
    cs_low();
    spi_wait_so_low();
    spi_transfer(addr | 0x40);
    for (uint8_t i = 0; i < len; i++) spi_transfer(data[i]);
    cs_high();
}

uint8_t CC2500_ReadFIFO(uint8_t *buf, uint8_t len) {
    cs_low();
    spi_wait_so_low();
    uint8_t status = spi_transfer(0xFF); /* RX FIFO burst read */
    for (uint8_t i = 0; i < len; i++) buf[i] = spi_transfer(0x00);
    cs_high();
    return status;
}

static void busy_delay(volatile uint32_t n) {
    while (n--);
}

void CC2500_Reset(void) {
    cs_high();
    busy_delay(1000);
    CC2500_Strobe(CC2500_SRES);
    busy_delay(8000); /* ~1ms at 8MHz for crystal + reset */
    for (int i = 0; i < 1000; i++) {
        if (!(CC2500_Strobe(CC2500_SNOP) & CC2500_STATUS_CHIP_RDYn)) break;
    }
}

uint8_t CC2500_WaitState(uint8_t state) {
    for (int i = 0; i < 2000; i++) {
        IWDG_KR = IWDG_REFRESH;
        if ((CC2500_Strobe(CC2500_SNOP) & CC2500_STATUS_STATE_MASK) == state)
            return 1;
    }
    return 0;
}

void LED_Init(void) {
    RCC_AHBENR |= RCC_AHBENR_GPIOAEN;
    GPIOA_MODER &= ~((3u << (6 * 2)) | (3u << (7 * 2)));
    GPIOA_MODER |= ((1u << (6 * 2)) | (1u << (7 * 2)));
    LED_SetBlue(0);
    LED_SetRed(0);
}

void LED_SetBlue(uint8_t on) {
    if (on) GPIOA_BSRR = (1u << 6);
    else    GPIOA_BSRR = (1u << (6 + 16));
}

void LED_SetRed(uint8_t on) {
    if (on) GPIOA_BSRR = (1u << 7);
    else    GPIOA_BSRR = (1u << (7 + 16));
}
