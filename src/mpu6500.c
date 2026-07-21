#include "mpu6500.h"

MPU6500_Cal_t g_mpu_cal;
uint8_t g_mpu_whoami;
uint8_t g_mpu_ok;
uint8_t g_mpu_addr;

/* Half-bit I2C pad. At 8MHz the inlined bit-bang GPIO ops alone hold SCL far
 * below the ICM-20689's 400kHz max, so no explicit delay is needed. History:
 * an old value of 40 (then 4) ran the bus at ~30kHz and made a 14-byte burst
 * cost ~4.2ms — long enough to stall the main loop and stutter the servos
 * every IMU tick (the "freezes started when the IMU was connected" symptom).
 * Bench-measured 2026-07-06: with the helpers force-inlined and this at 0 the
 * 14-byte burst is ~1.08ms and SCL sits at ~117kHz (3.4x margin under 400kHz);
 * WHO_AM_I still reads 0x98 and samples stay valid. Raise this only if a scope
 * shows setup/hold violations, or if the MCU clock is ever increased. */
#define I2C_DELAY_ITERS 8
static inline __attribute__((always_inline)) void i2c_delay(void) {
    for (volatile int i = 0; i < I2C_DELAY_ITERS; i++);
}

/* Open-drain I2C bit-bang: "low" actively drives the pin, "high" releases
 * it (lets the pull-up — external or the STM32's internal ~40k, enabled
 * below — bring it up, or lets the slave hold it down for ACK/read data).
 * Never drive high directly; that would fight another device on the bus. */
/* Force-inlined: these are called per-bit in the read/write loops, and at -Os
 * GCC otherwise keeps them as real calls — the call/return overhead per bit was
 * the dominant cost of the ~4ms burst, not the i2c_delay. */
/* Atomic BSRR, not GPIOB_ODR read-modify-write: the CC2500 SPI bit-bang runs in
 * the TIM14 ISR and can preempt this I2C mid-transfer on the SAME port (GPIOB).
 * An RMW would lose the other path's concurrent bit change; BSRR only touches
 * the named pin, so SPI (PB4/5) and I2C (PB6/7) coexist. */
#define I2C_INLINE static inline __attribute__((always_inline))
I2C_INLINE void scl_low(void)  { GPIOB_BSRR = (uint32_t)MPU_SCL_PIN << 16; }
I2C_INLINE void scl_release(void) { GPIOB_BSRR = MPU_SCL_PIN; }
I2C_INLINE void sda_low(void)  { GPIOB_BSRR = (uint32_t)MPU_SDA_PIN << 16; }
I2C_INLINE void sda_release(void) { GPIOB_BSRR = MPU_SDA_PIN; }
I2C_INLINE uint8_t sda_read(void) { return (GPIOB_IDR & MPU_SDA_PIN) ? 1 : 0; }

static void i2c_init_gpio(void) {
    RCC_AHBENR |= RCC_AHBENR_GPIOBEN;

    /* Open-drain output, pull-up enabled, both idle released (high). */
    GPIOB_MODER &= ~((3u << (6 * 2)) | (3u << (7 * 2)));
    GPIOB_MODER |= ((1u << (6 * 2)) | (1u << (7 * 2)));
    GPIOB_OTYPER |= MPU_SCL_PIN | MPU_SDA_PIN;
    GPIOB_PUPDR &= ~((3u << (6 * 2)) | (3u << (7 * 2)));
    GPIOB_PUPDR |= ((1u << (6 * 2)) | (1u << (7 * 2)));
    scl_release();
    sda_release();
}

static void i2c_start(void) {
    sda_release(); scl_release(); i2c_delay();
    sda_low(); i2c_delay();
    scl_low(); i2c_delay();
}

static void i2c_restart(void) {
    sda_release(); i2c_delay();
    scl_release(); i2c_delay();
    sda_low(); i2c_delay();
    scl_low(); i2c_delay();
}

static void i2c_stop(void) {
    sda_low(); i2c_delay();
    scl_release(); i2c_delay();
    sda_release(); i2c_delay();
}

/* Returns 1 if the slave ACKed (pulled SDA low), 0 on NACK. */
static uint8_t i2c_write_byte(uint8_t byte) {
    for (int i = 7; i >= 0; i--) {
        if (byte & (1 << i)) sda_release(); else sda_low();
        i2c_delay();
        scl_release();
        i2c_delay();
        scl_low();
    }
    sda_release(); /* release for the slave to drive ACK */
    i2c_delay();
    scl_release();
    i2c_delay();
    uint8_t ack = (sda_read() == 0);
    scl_low();
    i2c_delay();
    return ack;
}

static uint8_t i2c_read_byte(uint8_t ack) {
    uint8_t value = 0;
    sda_release();
    for (int i = 7; i >= 0; i--) {
        i2c_delay();
        scl_release();
        i2c_delay();
        if (sda_read()) value |= (1 << i);
        scl_low();
    }
    if (ack) sda_low(); else sda_release();
    i2c_delay();
    scl_release();
    i2c_delay();
    scl_low();
    sda_release();
    return value;
}

/* addr is the 7-bit I2C address; returns 1 if the device ACKed the address
 * byte (i.e. it's actually present at that address). */
static uint8_t i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t value) {
    i2c_start();
    uint8_t ok = i2c_write_byte((uint8_t)(addr << 1));
    ok &= i2c_write_byte(reg);
    ok &= i2c_write_byte(value);
    i2c_stop();
    return ok;
}

static uint8_t i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t *out) {
    i2c_start();
    uint8_t ok = i2c_write_byte((uint8_t)(addr << 1));
    ok &= i2c_write_byte(reg);
    i2c_restart();
    ok &= i2c_write_byte((uint8_t)((addr << 1) | 1));
    *out = i2c_read_byte(0); /* single byte: NACK to tell the slave to stop */
    i2c_stop();
    return ok;
}

static uint8_t i2c_read_burst(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len) {
    i2c_start();
    uint8_t ok = i2c_write_byte((uint8_t)(addr << 1));
    ok &= i2c_write_byte(reg);
    i2c_restart();
    ok &= i2c_write_byte((uint8_t)((addr << 1) | 1));
    for (uint8_t i = 0; i < len; i++) buf[i] = i2c_read_byte(i < len - 1);
    i2c_stop();
    return ok;
}

uint8_t MPU6500_ReadReg(uint8_t addr) {
    uint8_t value = 0;
    i2c_read_reg(g_mpu_addr, addr, &value);
    return value;
}

void MPU6500_WriteReg(uint8_t addr, uint8_t value) {
    i2c_write_reg(g_mpu_addr, addr, value);
}

void MPU6500_ReadRaw(MPU6500_Raw_t *out) {
    uint8_t buf[14];
    i2c_read_burst(g_mpu_addr, MPU_REG_ACCEL_XOUT_H, buf, 14);

    out->accel_x = (int16_t)((buf[0] << 8) | buf[1]);
    out->accel_y = (int16_t)((buf[2] << 8) | buf[3]);
    out->accel_z = (int16_t)((buf[4] << 8) | buf[5]);
    /* buf[6..7] = TEMP_OUT, unused */
    out->gyro_x  = (int16_t)((buf[8] << 8) | buf[9]);
    out->gyro_y  = (int16_t)((buf[10] << 8) | buf[11]);
    out->gyro_z  = (int16_t)((buf[12] << 8) | buf[13]);
}

static void busy_delay(volatile uint32_t n) {
    while (n--);
}

/* Try both possible 7-bit addresses (AD0 strap low=0x68, high=0x69) by
 * reading WHO_AM_I; use whichever answers a known value (0x70 MPU6500 or
 * 0x98 ICM-20689 — this board's chip is the latter, confirmed 2026-07-04).
 * Avoids needing to know the AD0 strap from a hardware check. */
static uint8_t probe_address(void) {
    uint8_t who;
    if (i2c_read_reg(MPU_I2C_ADDR_LOW, MPU_REG_WHO_AM_I, &who) && MPU_IS_WHO_AM_I(who))
        return MPU_I2C_ADDR_LOW;
    if (i2c_read_reg(MPU_I2C_ADDR_HIGH, MPU_REG_WHO_AM_I, &who) && MPU_IS_WHO_AM_I(who))
        return MPU_I2C_ADDR_HIGH;
    return 0; /* neither answered */
}

void MPU6500_Init(void) {
    i2c_init_gpio();
    busy_delay(48000); /* ~3ms settle */

    /* RETRY the probe (2026-07-13). A single probe intermittently returned 0 —
     * the ICM-20689 isn't always ready on I2C the instant we ask after a cold
     * boot, and the old busy_delay(8000) settle was calibrated for 8MHz so at
     * 48MHz it's ~6x too short. A failed probe left g_mpu_ok=0 and stabilization
     * SILENTLY OFF (found when a later manual reset, giving the chip more time,
     * brought it up clean). Retry ~90ms total so a slow wake-up can't disable
     * stab; refresh the watchdog across the wait. */
    g_mpu_addr = 0;
    for (int attempt = 0; attempt < 30 && g_mpu_addr == 0; attempt++) {
        IWDG_KR = IWDG_REFRESH;
        g_mpu_addr = probe_address();
        if (g_mpu_addr == 0) busy_delay(48000); /* ~3ms between tries */
    }
    if (g_mpu_addr == 0) {
        g_mpu_whoami = 0;
        g_mpu_ok = 0;
        return;
    }

    /* Reset (PWR_MGMT_1.DEVICE_RESET), then bring up clock + config.
     * gyro PLL clock source, 1kHz internal rate (no divider), FS ±2000dps /
     * ±16g, FIFO/interrupts/I2C-master all disabled.
     *
     * DLPF TIGHTENED 2026-07-12 for main-rotor vibration (pilot: servo dither
     * grows with head speed, damps when the board is hand-held = vibration
     * into the IMU). We sample gyro at 100Hz (Nyquist 50Hz) and accel at
     * ~12.5Hz (Nyquist ~6Hz), so the stock-wide filters (gyro 188Hz, accel
     * ~460Hz) let the rotor fundamental (~33-48Hz) and its harmonics alias
     * straight into the loop:
     *   CONFIG=0x04         -> gyro DLPF 20Hz (was 0x01 = 188Hz)
     *   ACCEL_CONFIG2=0x06  -> accel DLPF 5Hz (was default ~460Hz)
     * 20Hz keeps ample control bandwidth for a hover heli (<10Hz dynamics);
     * relax toward 0x03 (41Hz) only if it feels laggy. */
    MPU6500_WriteReg(MPU_REG_PWR_MGMT_1, 0x80);
    /* DEVICE_RESET needs ~100ms to complete (datasheet); the old busy_delay(8000)
     * was ~0.15ms at 48MHz. Refresh the watchdog across a real settle. */
    for (int i = 0; i < 20; i++) { IWDG_KR = IWDG_REFRESH; busy_delay(48000); } /* ~60ms */

    MPU6500_WriteReg(MPU_REG_PWR_MGMT_1, 0x01);
    MPU6500_WriteReg(MPU_REG_SMPLRT_DIV, 0x00);
    MPU6500_WriteReg(MPU_REG_CONFIG, 0x04);
    MPU6500_WriteReg(MPU_REG_GYRO_CONFIG, 0x18);
    MPU6500_WriteReg(MPU_REG_ACCEL_CONFIG, 0x18);
    MPU6500_WriteReg(MPU_REG_ACCEL_CONFIG2, 0x06);
    MPU6500_WriteReg(MPU_REG_FIFO_EN, 0x00);
    MPU6500_WriteReg(MPU_REG_INT_PIN_CFG, 0x00);
    MPU6500_WriteReg(MPU_REG_INT_ENABLE, 0x00);
    MPU6500_WriteReg(MPU_REG_USER_CTRL, 0x00);
    MPU6500_WriteReg(MPU_REG_PWR_MGMT_2, 0x00);

    g_mpu_whoami = MPU6500_ReadReg(MPU_REG_WHO_AM_I);
    g_mpu_ok = MPU_IS_WHO_AM_I(g_mpu_whoami);
}

void MPU6500_CalibrateGyro(uint16_t samples) {
    int64_t sum_x = 0, sum_y = 0, sum_z = 0;
    MPU6500_Raw_t raw;

    for (uint16_t i = 0; i < samples; i++) {
        IWDG_KR = IWDG_REFRESH;
        MPU6500_ReadRaw(&raw);
        sum_x += raw.gyro_x;
        sum_y += raw.gyro_y;
        sum_z += raw.gyro_z;
        busy_delay(8000); /* ~1ms between samples at 8MHz */
    }

    g_mpu_cal.gyro_bias_x = (int32_t)(sum_x / samples);
    g_mpu_cal.gyro_bias_y = (int32_t)(sum_y / samples);
    g_mpu_cal.gyro_bias_z = (int32_t)(sum_z / samples);
}

void MPU6500_GetSample(MPU6500_Sample_t *out) {
    MPU6500_Raw_t raw;
    MPU6500_ReadRaw(&raw);

    out->accel_x_g = (float)raw.accel_x / MPU_ACCEL_LSB_PER_G;
    out->accel_y_g = (float)raw.accel_y / MPU_ACCEL_LSB_PER_G;
    out->accel_z_g = (float)raw.accel_z / MPU_ACCEL_LSB_PER_G;

    out->gyro_x_dps = (float)(raw.gyro_x - g_mpu_cal.gyro_bias_x) / MPU_GYRO_LSB_PER_DPS;
    out->gyro_y_dps = (float)(raw.gyro_y - g_mpu_cal.gyro_bias_y) / MPU_GYRO_LSB_PER_DPS;
    out->gyro_z_dps = (float)(raw.gyro_z - g_mpu_cal.gyro_bias_z) / MPU_GYRO_LSB_PER_DPS;
}

/* Gyro-only read: 6-byte burst from GYRO_XOUT_H, ~half the bit-bang time of a
 * full 14-byte accel+gyro read. Used on the common IMU tick to keep the loop
 * responsive; accel is read less often via MPU6500_GetSample. */
void MPU6500_GetGyroSample(MPU6500_Sample_t *out) {
    uint8_t buf[6];
    i2c_read_burst(g_mpu_addr, MPU_REG_GYRO_XOUT_H, buf, 6);

    int16_t gx = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t gy = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t gz = (int16_t)((buf[4] << 8) | buf[5]);
    out->gyro_x_dps = (float)(gx - g_mpu_cal.gyro_bias_x) / MPU_GYRO_LSB_PER_DPS;
    out->gyro_y_dps = (float)(gy - g_mpu_cal.gyro_bias_y) / MPU_GYRO_LSB_PER_DPS;
    out->gyro_z_dps = (float)(gz - g_mpu_cal.gyro_bias_z) / MPU_GYRO_LSB_PER_DPS;
}
