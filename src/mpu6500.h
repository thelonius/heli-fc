#ifndef MPU6500_H
#define MPU6500_H

#include <stdint.h>
#include "stm32f031.h"

/* InvenSense 6-axis IMU bit-bang I2C driver.
 *
 * NOTE ON NAMING: every symbol here is prefixed MPU6500 for historical reasons,
 * but this board's actual chip is an ICM-20689 (WHO_AM_I = 0x98, confirmed on
 * hardware — see MPU_WHO_AM_I_* below). The ICM-20689 is a pin- and register-
 * compatible successor to the MPU-6500, so the register map, init sequence, and
 * datasheet references in this file all still apply unchanged; only the
 * accepted WHO_AM_I value differs. The prefix was left as-is to avoid churning
 * 100+ call sites and the SWD debug addresses — read "MPU6500" as "the IMU".
 *
 * This chip is wired for I2C, not SPI: continuity-testing the board found
 * nCS (chip pin 22) tied directly to VDD. Per the MPU-6500 datasheet
 * (InvenSense PS-MPU-6500A-01 rev 1.1, section 4.9 / Figure 6), nCS held
 * high permanently is exactly how I2C mode is selected — SPI mode requires
 * the host to actively drive nCS. In I2C mode, chip pin 23 (SCL/SCLK) and
 * pin 24 (SDA/SDI) become the standard open-drain I2C clock/data lines
 * (SDA is bidirectional, unlike the push-pull MOSI this driver originally
 * assumed for SPI), and chip pin 9 (AD0/SDO in the SPI diagram) becomes AD0,
 * a strap pin setting the I2C address's LSB (0x68 if grounded, 0x69 if
 * tied to VDD) — not a data line the MCU drives.
 *
 * Confirmed 2026-07-04 by continuity-testing the physical board:
 *   chip pin 24 (SDA) -> MCU PB7
 *   chip pin 23 (SCL) -> MCU PB6
 * (Both on GPIOB, unlike the original SPI-era placeholder guesses on GPIOA.)
 *
 * The I2C address (0x68 vs 0x69) depends on the AD0 strap, which hasn't been
 * checked on the board — MPU6500_Init() tries both and uses whichever answers
 * a valid WHO_AM_I (this board's ICM-20689 returns 0x98; a genuine MPU6500
 * would return 0x70), so this doesn't need a separate hardware check. */
#define MPU_SCL_PIN   (1u << 6)   /* PB6 = SCL  (CONFIRMED: chip pin 23) */
#define MPU_SDA_PIN   (1u << 7)   /* PB7 = SDA  (CONFIRMED: chip pin 24) */

#define MPU_I2C_ADDR_LOW   0x68  /* AD0 strapped to GND */
#define MPU_I2C_ADDR_HIGH  0x69  /* AD0 strapped to VDD */

/* IMU registers actually used — identical on MPU-6500 and ICM-20689 (values
 * taken from heli_ghidra_output/heli_decompiled.c IMU_ConfigureAllRegs /
 * IMU_ReadGyro — that part of the decompile is legitimate: 0x6B/0x19/0x1A/0x1B/
 * 0x1C/0x23/0x37/0x38/0x6A/0x6C and reading gyro from 0x43 are real register
 * addresses per the datasheet, unlike the mislabeled pin/init function). */
#define MPU_REG_SMPLRT_DIV     0x19
#define MPU_REG_CONFIG         0x1A
#define MPU_REG_GYRO_CONFIG    0x1B
#define MPU_REG_ACCEL_CONFIG   0x1C
#define MPU_REG_ACCEL_CONFIG2  0x1D  /* accel DLPF bandwidth */
#define MPU_REG_FIFO_EN        0x23
#define MPU_REG_INT_PIN_CFG    0x37
#define MPU_REG_INT_ENABLE     0x38
#define MPU_REG_ACCEL_XOUT_H   0x3B
#define MPU_REG_GYRO_XOUT_H    0x43
#define MPU_REG_USER_CTRL      0x6A
#define MPU_REG_PWR_MGMT_1     0x6B
#define MPU_REG_PWR_MGMT_2     0x6C
#define MPU_REG_WHO_AM_I       0x75
#define MPU_WHO_AM_I_MPU6500   0x70
#define MPU_WHO_AM_I_ICM20689  0x98
/* Confirmed 2026-07-04: this board's chip answers 0x98 — it's actually an
 * ICM-20689, not an MPU6500. Same pinout, register map, and I2C protocol
 * (InvenSense's ICM-20689 is designed as a pin/register-compatible successor
 * to the MPU6500), so the driver code doesn't need to change, only which
 * WHO_AM_I value counts as success. */
#define MPU_IS_WHO_AM_I(v) ((v) == MPU_WHO_AM_I_MPU6500 || (v) == MPU_WHO_AM_I_ICM20689)

/* Scale factors for the config below: GYRO_CONFIG=0x18 (FS_SEL=3, ±2000dps),
 * ACCEL_CONFIG=0x18 (FS_SEL=3, ±16g). Same LSB/dps and LSB/g on MPU-6500 and
 * ICM-20689 (both InvenSense datasheets, table 2-1). */
#define MPU_GYRO_LSB_PER_DPS   16.4f
#define MPU_ACCEL_LSB_PER_G    2048.0f

typedef struct {
    int16_t accel_x, accel_y, accel_z; /* raw, LSB */
    int16_t gyro_x, gyro_y, gyro_z;    /* raw, LSB */
} MPU6500_Raw_t;

typedef struct {
    float accel_x_g, accel_y_g, accel_z_g;     /* g */
    float gyro_x_dps, gyro_y_dps, gyro_z_dps;  /* deg/s, bias-corrected */
} MPU6500_Sample_t;

/* gyro_bias[] in raw LSB, subtracted from every raw gyro reading before
 * scaling. Filled in by MPU6500_CalibrateGyro(); assumes the airframe is
 * held still and level while calibrating. */
typedef struct {
    int32_t gyro_bias_x, gyro_bias_y, gyro_bias_z;
} MPU6500_Cal_t;

extern MPU6500_Cal_t g_mpu_cal;
extern uint8_t g_mpu_whoami;   /* last WHO_AM_I read; this board = 0x98 (ICM-20689) */
extern uint8_t g_mpu_ok;       /* 1 once WHO_AM_I matched and config verified */
extern uint8_t g_mpu_addr;     /* I2C address that answered: 0x68 or 0x69 */

void MPU6500_Init(void);                         /* GPIO setup + address probe + reset + config + verify */
uint8_t MPU6500_ReadReg(uint8_t addr);
void MPU6500_WriteReg(uint8_t addr, uint8_t value);
void MPU6500_ReadRaw(MPU6500_Raw_t *out);         /* one burst read of accel+gyro (14 bytes) */
/* Stillness-validated (2026-07-20, stock's scheme): `samples` = CONSECUTIVE
 * still reads required; motion discards the accumulator and restarts. Slow
 * red blink = measuring, fast flicker = disturbed. See the .c for the story. */
void MPU6500_CalibrateGyro(uint16_t samples);
extern volatile uint16_t g_gyro_cal_restarts;  /* motion events during cal (SWD) */
extern volatile uint8_t  g_gyro_cal_timeout;   /* 1 = never settled, EMA fallback used */
void MPU6500_GetSample(MPU6500_Sample_t *out);    /* raw read + bias correction + scaling */
void MPU6500_GetGyroSample(MPU6500_Sample_t *out); /* gyro only (6B burst): fills gyro_*_dps, accel fields untouched */

#endif /* MPU6500_H */
