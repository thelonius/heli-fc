#include "orientation.h"

Orientation_t g_orientation;

/* ACCEL-HEALTH TRIPWIRE (2026-07-19, after the stuck-estimate incident above —
 * see the LEVEL OFFSETS comment). The trust gate rejects accel samples
 * silently, so a filter running gyro-only for a whole flight was
 * indistinguishable from a healthy one. These counters make it a one-read
 * post-flight check (same methodology as statLost: read over SWD after
 * landing, power still on):
 *   g_accel_ok / g_accel_rej: accepted / rejected accel ticks, cumulative.
 *     Rejection ratio >> a few % at rest or in steady hover = accel data bad.
 *   g_accel_rej_run: CURRENT consecutive rejections — a large value right
 *     after landing means the filter is coasting on gyro alone right now.
 *   g_accel_last_magsq: |a|^2 of the LAST accel sample (healthy at rest
 *     ~1.0); far from 1.0 while parked = chip serving garbage -> the "biased
 *     chip data" branch of the incident, vs gate-rejection = the "gyro-only"
 *     branch. */
volatile uint32_t g_accel_ok, g_accel_rej;
volatile uint16_t g_accel_rej_run;
volatile float g_accel_last_magsq;

/* Ground yaw auto-trim state (see the block at the end of Orientation_Update).
 * g_gyro_trim_dps[x,y,z] is SUBTRACTED from the raw gyro before ANY consumer
 * (angle integration, rate feedback, yaw loop) sees it;
 * g_gyro_trim_count = how many 1s-windows have refreshed it (SWD check:
 * should tick up while sitting at idle with signal, freeze in flight). */
volatile float g_gyro_trim_dps[3];      /* [x,y,z], SUBTRACTED from raw gyro */
volatile uint16_t g_gyro_trim_count;    /* windows that refreshed the trim */
static float s_gt_sum[3], s_gt_min[3], s_gt_max[3];
static uint16_t s_gt_n;

#define RAD_TO_DEG 57.29578f

/* Blend weight for the gyro-integrated angle vs. the accel estimate.
 * Standard complementary-filter starting point; retune once the airframe
 * is on the bench (a heavier gyro weight drifts slower but tracks a real
 * static tilt more sluggishly). */
#define GYRO_WEIGHT   0.98f
#define ACCEL_WEIGHT  (1.0f - GYRO_WEIGHT)

/* Only trust the accelerometer's tilt estimate when the sensed magnitude is
 * close to 1g — i.e. the airframe isn't accelerating or vibrating hard.
 * Checked on the squared magnitude to avoid needing sqrtf (no libm on this
 * Cortex-M0 build). 0.8g..1.2g -> squared bounds 0.64..1.44. */
#define ACCEL_TRUST_MIN_SQ  0.64f
#define ACCEL_TRUST_MAX_SQ  1.44f

/* LEVEL OFFSETS — ZERO, and the war story of why (2026-07-19).
 * A bench "calibration" measured pitch +4.62 / roll -10.23 with the airframe
 * level and those values went into these macros for one build. WRONG: after
 * the flash (= MCU reset = MPU DEVICE_RESET in MPU6500_Init) the SAME
 * untouched airframe read raw ~0/-0.3. The +4.6/-10.2 was not mount bias but
 * a STUCK ESTIMATE that had survived since flight 4 (its in-flight hold mean,
 * roll -10.37, matches) and that a chip+filter re-init clears. At rest a
 * healthy filter re-converges to accel in ~4s (ACCEL_WEIGHT 0.02 @ ~12.5Hz);
 * this one sat wrong for MINUTES — either the chip itself served biased data
 * (register/internal state cured by DEVICE_RESET) or accel was continuously
 * rejected by the trust gate below and the angle ran gyro-only, frozen on the
 * flight-era estimate. The reflash destroyed the evidence; the g_accel_*
 * tripwire counters below exist to decide it at the NEXT occurrence.
 * Real mount bias measured after the reset: ~0/-0.3 deg — not worth an
 * offset. Do NOT put bench numbers here without the 180-degree-rotation test
 * separating mount bias from surface tilt AND a reboot to rule out a stuck
 * estimate. */
#define ORIENT_PITCH_LEVEL_OFFSET_DEG  0.0f
#define ORIENT_ROLL_LEVEL_OFFSET_DEG   0.0f

void Orientation_Init(void) {
    g_orientation.pitch_deg = 0.0f;
    g_orientation.roll_deg = 0.0f;
    g_orientation.yaw_rate_dps = 0.0f;
}

void Orientation_Update(const MPU6500_Sample_t *sample, float dt_s, uint8_t use_accel,
                        uint8_t on_ground) {
    /* Integrate gyro rates into the running angle estimate.
     * Pitch sign confirmed inverted vs. gyro_y_dps's raw direction by a
     * bench test 2026-07-04 (tilting the nose up drove pitch_deg negative,
     * but the convention is +=nose up) — negated here. Roll matched
     * gyro_x_dps's raw direction directly (right-side-down gave positive
     * roll_deg as expected), so it's untouched. */
    /* Ground trim applied to ALL THREE axes before anything uses them — see
     * the trim block at the end of this function. */
    float gx = sample->gyro_x_dps - g_gyro_trim_dps[0];
    float gy = sample->gyro_y_dps - g_gyro_trim_dps[1];
    float gz = sample->gyro_z_dps - g_gyro_trim_dps[2];

    float pitch = g_orientation.pitch_deg - gy * dt_s;
    float roll  = g_orientation.roll_deg  + gx * dt_s;

    /* Accel-based tilt estimate, small-angle approximation (asin(x)~=x for
     * small x in radians): near level flight, each horizontal accel
     * component divided by the vertical (Z) component approximates the tilt
     * angle in radians. This is intentionally NOT a full atan2 — degrades
     * above roughly +-30 deg of bank, acceptable for a hover-class stabilizer.
     * Signs matched to the gyro terms above (same bench test). */
    /* Accel tilt correction only on ticks where the accelerometer was actually
     * read (use_accel); gyro-only ticks just keep integrating. */
    if (use_accel) {
        float ax = sample->accel_x_g, ay = sample->accel_y_g, az = sample->accel_z_g;
        float mag_sq = ax * ax + ay * ay + az * az;
        g_accel_last_magsq = mag_sq;

        if (mag_sq > ACCEL_TRUST_MIN_SQ && mag_sq < ACCEL_TRUST_MAX_SQ && az != 0.0f) {
            g_accel_ok++;
            g_accel_rej_run = 0;
            float pitch_acc = (ax / az) * RAD_TO_DEG - ORIENT_PITCH_LEVEL_OFFSET_DEG;
            float roll_acc  = (ay / az) * RAD_TO_DEG - ORIENT_ROLL_LEVEL_OFFSET_DEG;
            pitch = GYRO_WEIGHT * pitch + ACCEL_WEIGHT * pitch_acc;
            roll  = GYRO_WEIGHT * roll  + ACCEL_WEIGHT * roll_acc;
        } else {
            /* accelerating/vibrating too hard to trust — gyro-only this tick */
            g_accel_rej++;
            if (g_accel_rej_run < 0xffff) g_accel_rej_run++;
        }
    }

    g_orientation.pitch_deg = pitch;
    g_orientation.roll_deg = roll;
    g_orientation.yaw_rate_dps = gz;
    /* Body rates for RATE-mode cyclic, signed like the angle integration above
     * (pitch_deg -= gyro_y, roll_deg += gyro_x). Updated every tick, including
     * gyro-only ticks (the gyro is read on all of them). */
    g_orientation.roll_rate_dps  =  gx;
    g_orientation.pitch_rate_dps = -gy;

    /* GROUND GYRO AUTO-TRIM, ALL THREE AXES.
     *
     * Started 2026-07-19 as Z-only, after flight 5's slow tail walk: gyro Z
     * read a steady -3.65 dps with the airframe motionless, and a rate loop
     * nulls the MEASURED rate, so the machine physically rotated instead.
     * The comment then said roll/pitch needed no trim because "the accel
     * correction absorbs it". FLIGHT 12 PROVED THAT WRONG, and expensively:
     * the pilot reported a strong roll drift, and afterwards the bench read
     * gyro X = +6.71 dps at rest with the estimate parked at +25.6 deg on a
     * level machine. The accel does NOT absorb a gyro bias, it BALANCES it:
     * equilibrium sits where the accel pull equals the gyro push, i.e.
     *      angle error = bias x tau,  tau = 1/(ACCEL_WEIGHT x accel rate)
     *                  = 6.71 x 4s   = 26.8 deg   (measured: 25.6)
     * So this filter multiplies gyro bias by FOUR SECONDS and reports it as
     * attitude. One dps of bias is four degrees of lie, and the loop then
     * banks the machine to make its own lie read zero.
     *
     * Root cause of that 6.71: the BOOT calibration captured motion, not
     * bias. g_mpu_cal held x = -118 LSB = -7.20 dps while the sensor's true
     * resting bias is ~-0.5 dps — the airframe was being moved when it was
     * powered up, and that rotation was frozen in as "zero" for the whole
     * session. Re-zeroing whenever we KNOW the machine is stationary fixes
     * exactly that, and it also catches the slow walk the Z axis showed.
     *
     * Stillness is judged by SPREAD, not magnitude — the bias itself may be
     * large, but a sitting airframe's spread stays at noise level while
     * hand-carrying blows past it. All three axes must be still, otherwise a
     * rotation about one axis could be captured as another's bias. Frozen in
     * flight (no reference there), clamped because a trim past +-10 dps means
     * "not actually still" rather than bias. Also the reason this is worth
     * more than a better boot calibration: it re-runs before EVERY takeoff. */
    if (on_ground) {
        float g[3] = { sample->gyro_x_dps, sample->gyro_y_dps,
                       sample->gyro_z_dps };            /* raw, pre-trim */
        if (s_gt_n == 0) {
            for (int i = 0; i < 3; i++) {
                s_gt_min[i] = g[i]; s_gt_max[i] = g[i]; s_gt_sum[i] = 0.0f;
            }
        }
        for (int i = 0; i < 3; i++) {
            if (g[i] < s_gt_min[i]) s_gt_min[i] = g[i];
            if (g[i] > s_gt_max[i]) s_gt_max[i] = g[i];
            s_gt_sum[i] += g[i];
        }
        if (++s_gt_n >= 100) {                          /* ~1s at the 100Hz tick */
            uint8_t still = 1;
            float m[3];
            for (int i = 0; i < 3; i++) {
                m[i] = s_gt_sum[i] / (float)s_gt_n;
                if (s_gt_max[i] - s_gt_min[i] >= 2.0f) still = 0;
                if (m[i] <= -10.0f || m[i] >= 10.0f)    still = 0;
            }
            if (still) {
                for (int i = 0; i < 3; i++) g_gyro_trim_dps[i] = m[i];
                g_gyro_trim_count++;
            }
            s_gt_n = 0;
        }
    } else {
        s_gt_n = 0;   /* airborne: freeze, restart the window on next ground */
    }
}
