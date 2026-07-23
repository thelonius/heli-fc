#ifndef SPI_RC_H
#define SPI_RC_H

#include <stdint.h>
#include "stm32f031.h"

/* SPI pins — confirmed empirically over SWD by reading back PARTNUM=0x80,
 * VERSION=0x03 (2026-07-03). Do not change without re-verifying the same way.
 * PA15 was guessed from a decompiled EXTI handler as the GDO pin but reads
 * stuck high with the transmitter live (floating debug pin, not real GDO) —
 * do not use it. Poll CC2500_RXBYTES over SPI instead. */
#define SPI_CS_PIN    (1u << 8)  /* PA8  = CSn */
#define SPI_SCK_PIN   (1u << 4)  /* PB4  = SCLK */
#define SPI_MOSI_PIN  (1u << 5)  /* PB5  = SI */
#define SPI_MISO_PIN  (1u << 3)  /* PB3  = SO */

/* CC2500 registers */
#define CC2500_FSCTRL0   0x0C
#define CC2500_CHANNR    0x0A
#define CC2500_FSCAL1    0x25
#define CC2500_PARTNUM   0x30    /* status reg, read with 0xC0 */
#define CC2500_VERSION   0x31
#define CC2500_FREQEST   0x32    /* status reg: signed carrier offset estimate */
#define CC2500_RXBYTES   0x3B    /* status reg: bit7=overflow, bits6-0=count */

/* CC2500 command strobes */
#define CC2500_SRES      0x30
#define CC2500_SCAL      0x33
#define CC2500_SRX       0x34
#define CC2500_SIDLE     0x36
#define CC2500_SFRX      0x3A
#define CC2500_SNOP      0x3D

/* Chip status byte */
#define CC2500_STATUS_CHIP_RDYn   0x80
#define CC2500_STATUS_STATE_MASK  0x70
#define CC2500_STATE_IDLE         0x00
#define CC2500_STATE_RX           0x10
#define CC2500_STATE_RX_OVERFLOW  0x60
#define CC2500_STATUS_FIFO_MASK   0x0F

void SPI_RC_Init(void);
uint8_t CC2500_Strobe(uint8_t cmd);
void CC2500_WriteReg(uint8_t addr, uint8_t value);
uint8_t CC2500_ReadReg(uint8_t addr);
uint8_t CC2500_ReadStatusReg(uint8_t addr);
uint8_t CC2500_ReadRxBytes(void);  /* RXBYTES with the read-twice silicon erratum workaround */
void CC2500_WriteBurst(uint8_t addr, const uint8_t *data, uint8_t len);
uint8_t CC2500_ReadFIFO(uint8_t *buf, uint8_t len); /* returns chip status byte */
uint8_t CC2500_Reset(void);                         /* 1 = chip reported ready */
uint8_t CC2500_WaitState(uint8_t state);            /* 1 = reached, 0 = timeout */

/* LEDs: PA6 = blue, PA7 = red (per commit 7629395) */
void LED_Init(void);
void LED_SetBlue(uint8_t on);
void LED_SetRed(uint8_t on);

#endif /* SPI_RC_H */
