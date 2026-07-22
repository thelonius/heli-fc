#ifndef ORIENTATION_H
#define ORIENTATION_H

#include "mpu6500.h"

/* Complementary filter: pitch/roll estimated from the gyro (integrated,
 * responsive but drifts) blended with the accelerometer (absolute, but noisy
 * and only valid near 1g / low acceleration). This is a deliberate
 * simplification versus the original firmware's full quaternion fusion
 * (see heli_ghidra_output/STRUCT_DEFS.h PTR_QUATERNION) — the original's
 * exact gain constants weren't recoverable from the decompile (opaque
 * float symbols CONST_PITCH_GAIN/CONST_ROLL_GAIN with no literal dumped),
 * and a small-angle complementary filter is the standard, well-understood
 * choice for a simple hover-class stabilizer. Revisit if attitude accuracy
 * at high bank angles turns out to matter. */

typedef struct {
    float pitch_deg;   /* + = nose up */
    float roll_deg;    /* + = right side down */
    float yaw_rate_dps; /* raw gyro Z, no absolute yaw reference available */
    /* Body angular RATES, deg/s, signed to match the angle conventions above
     * (roll_rate = +gyro_x, pitch_rate = -gyro_y — same signs the integrator
     * uses below). These are the feedback for RATE-mode cyclic (flybarless
     * "normal"): the stick sets a desired rate, these measure the actual one. */
    float roll_rate_dps;   /* + = rolling right-side-down */
    float pitch_rate_dps;  /* + = pitching nose-up */
} Orientation_t;

extern Orientation_t g_orientation;

/* Ground gyro auto-trim, reported outward so the pilot gets a LED and SWD gets
 * a post-mortem. The trim is what re-zeroes the gyro before every takeoff, so
 * "did it actually run" is flight-safety information, not debug trivia — it was
 * invisible until 2026-07-22, when g_gyro_trim_count turned out to be 0 across a
 * whole session and nobody could have known from outside the board. */
#define GT_NONE    0   /* no window has finished yet (first ~1s after power-up) */
#define GT_MOVING  1   /* windows keep getting rejected, NO zero captured — do not fly */
#define GT_OK      2   /* last window accepted; g_gyro_trim_dps is fresh */
#define GT_STALE   3   /* a zero is held, but the latest window saw motion */
extern volatile uint8_t  g_gyro_trim_state;   /* GT_* */
extern volatile uint16_t g_gyro_trim_count;   /* accepted windows (sticky: ever calibrated) */
extern volatile uint16_t g_gyro_trim_rej;     /* rejected windows */
extern volatile float    g_gyro_trim_dps[3];

void Orientation_Init(void);
/* dt_s = seconds since the previous call. use_accel=1 when sample carries a
 * fresh accelerometer read (full MPU6500_GetSample); 0 for a gyro-only tick
 * (MPU6500_GetGyroSample) — the accel tilt correction is then skipped. */
/* on_ground = throttle at idle with live signal (main.c decides): enables the
 * ground yaw auto-trim window; pass 0 in flight so the trim stays frozen. */
void Orientation_Update(const MPU6500_Sample_t *sample, float dt_s, uint8_t use_accel,
                        uint8_t on_ground);

#endif /* ORIENTATION_H */
