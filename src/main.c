#include "stm32f031.h"
#include "spi_rc.h"
#include "sfhss.h"
#include "mpu6500.h"
#include "orientation.h"
#include "stabilize.h"

extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss;

/* SysTick is 1MHz/24-bit (see systick_init below); all time math here is
 * modulo 2^24 (~16.7s wraparound), same convention as sfhss.c. */
#define TIME_WRAP 1000000u   /* micros() wraps every 1,000,000 us (see clock_init) */
__attribute__((unused)) static uint32_t elapsed_us(uint32_t since, uint32_t now) {
    int32_t d = (int32_t)now - (int32_t)since;
    if (d < 0) d += TIME_WRAP;
    return (uint32_t)d;
}

/* IMU sampling is rate-limited and kept OFF the hot SFHSS polling path: the
 * radio's RX FIFO overflows if bit-bang traffic (the IMU read is a 14-byte
 * I2C burst) delays draining it, which is exactly the bug an earlier session
 * spent a long time chasing. Each IMU read still costs real busy-wait time
 * (see i2c_delay() in mpu6500.c) between SFHSS_Poll() calls. */
/* 100Hz. Must comfortably exceed the IMU read+process block time (measured
 * ~4.8ms for a full accel+gyro burst, ~2.6ms gyro-only) or the read runs
 * back-to-back every loop and starves SFHSS_Poll — that was the refresh-rate
 * regression. */
#define IMU_PERIOD_US 10000u
/* Read the accelerometer (full 14-byte burst) only 1 tick in N; read the gyro
 * (6-byte burst) every tick. Gravity-based tilt drift-correction needs no more
 * than ~12Hz, and this keeps the common-case blind window to the gyro read. */
#define IMU_ACCEL_EVERY 8u

/* IMU polling. The 2026-07-04 experiment that set this to 0 (to test whether
 * the IMU I2C burst was degrading radio reception) concluded 2026-07-05: the
 * radio degradation was the S-FHSS pair-model bug (commit 15cedb2), NOT the
 * IMU — the IMU was exonerated with polling compiled out. Re-enabled now so
 * the gyro/accel path feeds stabilization. */
#define IMU_POLL_ENABLE 1

/* Stabilization engage RAMP. On (re)engage the correction is scaled 0->1 over
 * this many seconds so the swash EASES to the attitude hold instead of snapping
 * to it. Without it, anything that engages stab abruptly kicks the swash to the
 * current tilt's full correction in one tick — most visibly when idle-up forces
 * the throttle channel to full (crossing throttle_is_idle) and engages stab the
 * instant the switch is flipped (bench-seen 2026-07-12: ~25us swash jump). */
#define STAB_RAMP_S  0.7f

/* BENCH FLAG: when 1, the swash servos recentre whenever the motor is idle
 * (throttle down) — the requested parked-safety behavior. Set to 0 for bench
 * bring-up so the servos keep responding to roll/pitch/collective with the
 * throttle at minimum (main motor stays off at 1000us), which is the only way
 * to verify the CCPM mix directions without spinning the rotor. Set back to 1
 * once the mix is verified. */
#define SWASH_RECENTER_ON_IDLE 0

/* BENCH: pin->servo identification. When 1, ignore RC entirely and drive one
 * swash output (selected via g_bench_sel over SWD: 0=PA0, 1=PA1, 2=PA2) to
 * 1900us while the other two hold 1500us; both motors stay off (1000us). Set
 * g_bench_sel with `mww <addr> N`. Learns the real pin->servo wiring that the
 * CCPM roles depend on. Set back to 0 for flight. */
#define BENCH_SERVO_ID 0

/* Motor arming interlock. 1 = motors stay off after boot/signal-loss until the
 * throttle stick is seen at idle in normal mode (the CP arming ritual). 0 =
 * stock-receiver behavior, per the pilot's 2026-07-12 call: signal present =
 * armed ("если передатчик включён — значит он включён"). Signal-loss failsafe
 * still cuts the motors either way. NOTE with 0: powering up (or flashing)
 * with the TX already in idle-up spins the main motor immediately (the TX
 * pins the throttle channel to full) — accepted stock semantics. */
#define ARMING_INTERLOCK 0

/* BENCH: tail-ESC STEP-RESPONSE test (rotor blades irrelevant, MAIN MOTOR
 * FORCED OFF for the duration). Measures the ESC+motor+prop lag that sets the
 * tail-gain ceiling (see docs/tuning_framework.md). Trigger over SWD:
 *   mww <&g_step_go> 1        (addresses: nm firmware.elf | grep g_step)
 * Sequence (3s): tail 1150 (settle 1s) -> 1400 (1s, up-step) -> 1150
 * (1s, down-step) -> off. gyro-Z and the commanded tail are logged at the
 * 100Hz IMU tick into g_step_log/g_step_tail (256 entries); read them over
 * SWD after the run and fit dead time + tau on the host. The heli must stand
 * free so tail thrust can actually yaw it. Set 0 to compile out. Lag measured
 * 2026-07-13 (~80-100ms) — off for flight builds; flip to 1 to re-measure
 * after the BLHeli_S swap. */
#define TAIL_STEP_TEST 0

/* BENCH: tail RELAY AUTOTUNE experiment (Åström–Hägglund). MAIN MOTOR FORCED
 * OFF — that's also why SWD works here (the ST-Link only dies from main-rotor
 * RF pickup). The heli must be free to yaw: hang it by the rotor head on a
 * cord, or a turntable. Trigger over SWD:
 *   mww <&g_relay_go> 1        (addresses: nm firmware.elf | grep g_relay)
 * Sequence: tail settles at RELAY_BASE_US for 1.5s while a slow LPF learns
 * the mean yaw rate (a one-way tail has net thrust, so the frame pirouettes
 * steadily — the relay oscillates AROUND that drift, switching on the
 * mean-crossing, not zero). Then bang-bang: rate above mean + hyst -> thrust
 * LOW, below mean - hyst -> thrust HIGH (negative feedback: +tail_us = nose
 * right = +yaw_rate, confirmed 2026-07-05). 4s of rate + relay state logged
 * at the 100Hz IMU tick; dump with tools/relay_dump.sh, then
 * tools/relay_fit.py computes Ku=4d/(pi*a), Tu, and suggested KP/KI.
 * Set 0 to compile out of flight builds. */
#define TAIL_RELAY_TEST 0
#define RELAY_BASE_US   1150     /* same operating point as TAIL_STEP_TEST */
#define RELAY_D_US      60       /* relay half-amplitude d, us */
#define RELAY_HYST_DPS  3.0f     /* noise guard on the switching line */
#define RELAY_N         400      /* samples @100Hz = 4s of limit cycle */

#if STAB_BLACKBOX && (TAIL_STEP_TEST || TAIL_RELAY_TEST)
#error "RAM: bench log arrays + the yaw blackbox don't fit 4K together — set STAB_BLACKBOX 0 in stabilize.h for bench builds"
#endif

/* FLIGHT: high-rate GYRO RATE LOG — the ~4Hz-oscillation hunt (2026-07-19,
 * flight 5: pilot feels small fast oscillations, can't tell tail from cyclic;
 * the 4Hz trim blackbox is Nyquist-blind to them). All three body rates at
 * 50Hz into a ring holding the last ~7.5s of ENGAGED time: hover until the
 * oscillation is felt, chop throttle, land — the ring freezes with it inside
 * (same freeze semantics as the trim blackbox). Dump with tools/rate_dump.sh,
 * spectrum via tools/rate_fft.py: the loudest axis names the culprit (yaw =
 * tail loop/ESC, roll/pitch = cyclic), the exact frequency pins it against
 * the known modes (3-4Hz tail-gain, 6-8Hz airframe). RAM 6B x 373 = 2238:
 * REPLACES the trim blackbox for this build (one 2.2K ring fits, not two). */
#define RATE_LOG 0
#define RATE_LOG_N 373
#define RATE_LOG_DECIM 2    /* 100Hz IMU tick / 2 = 50Hz */

#if RATE_LOG && (STAB_BLACKBOX || TAIL_STEP_TEST || TAIL_RELAY_TEST)
#error "RAM: the rate log replaces the trim blackbox/bench arrays — set STAB_BLACKBOX 0 (stabilize.h) and bench modes 0"
#endif

/* BENCH: swash-correction SIGN test (rotor OFF). When 1, stabilization engages
 * on signal alone — the throttle-above-idle gate is dropped — so the swash
 * reacts to airframe tilt with the MOTOR STILL OFF (arming interlock keeps it
 * at PWM_MIN with the stick down). Tilt the frame by hand and watch the swash:
 * it must move to push the swash back toward level. If an axis drives DEEPER
 * into the tilt, flip that axis's STAB_*_CORR_SIGN in stabilize.h. Requires
 * STAB_ENABLE=1 and SWASH_RECENTER_ON_IDLE=0. Set back to 0 for flight.
 * Test PASSED 2026-07-12: both axes compensate — signs stay +1/+1.
 * Back to 0 for flight 2026-07-13: rate-mode validated with the motor running;
 * at idle stab now disengages and its integral resets (the stock "throttle-min
 * zeroes the accumulated cyclic" behavior). */
#define STAB_BENCH_TILT 0

/* System runs on 48MHz (PLL, see clock_init) — same clock the original FC used.
 * Timers get 48MHz on APB. PSC=47 → 48MHz/(47+1) = 1MHz timer clock → 1 tick =
 * 1us, so CCR is written directly in microseconds. ARR=8000 → 8000us period =
 * 125Hz frame rate, what these servos and ESCs expect. */
#define PWM_PSC         47             /* 48MHz/(47+1) = 1MHz timer tick */
#define PWM_ARR         8000            /* 8000us period → 125Hz */
#define PWM_CENTER      1520            /* RC center pulse width, us */
#define PWM_MIN         1000            /* min pulse width, us */
#define PWM_MAX         2000            /* max pulse width, us */

/* ==================== INPUT CHANNELS ====================
 * Fast first-frame axes only (type-1, ~145/s). g_sfhss.data[] index per
 * function, confirmed by SWD (see memory heli-channel-map / heli-sfhss-true-
 * protocol). The second frame (ch5-8) is a SLOW aux bank (mode switches +
 * a collective trim, sent ~on-change) — NOT usable as a flight axis, so
 * collective is synthesized on-chip from the throttle stick below, exactly
 * as a heli TX does internally. data[6],[7] pegged; throttle-cut not on any
 * received channel (arming interlock carries the motor-off safety). */
#define CH_ROLL        0   /* aileron, spring-center 1520 */
#define CH_PITCH       1   /* elevator, spring-center 1520 */
#define CH_THROTTLE    2   /* left stick, REVERSED: bottom=1930us, top=1109us.
                            * The TX applies its mode switches ONTO THIS
                            * CHANNEL (bench-proven 2026-07-12): idle-up ON
                            * forces it to full (1110) with the stick anywhere;
                            * cut forces it to idle (1930). The firmware needs
                            * NO mode logic of its own — just follow the
                            * channel; the stick meanwhile remains live pitch
                            * via CH_COLLECTIVE. */
#define CH_RUDDER      3   /* yaw, spring-center 1520 */
#define CH_MODE        4   /* NOT a mode switch on this TX: observed static
                            * 1410 while every switch was toggled (2026-07-12).
                            * The old "<1520 = idle-up" reading (from the stock
                            * decompile) misclassified it and permanently
                            * blocked arming. Kept as a diagnostic name only. */
#define CH_COLLECTIVE  5   /* collective pitch, own channel (reversed ~1725..1328) */

#define CYCLIC_CENTER  1520  /* roll/pitch/rudder spring center, us */

/* Cyclic stick -> swash scale. The raw stick throw (~±330us) drove nearly the
 * full swash range and felt too sharp (pilot 2026-07-13). <1 softens the
 * immediate response; the rate-loop accumulation/hold (stabilize.c) is
 * unaffected, so cyclic stays firm to hold but gentle to command. */
#define CYCLIC_GAIN  0.45f

/* Collective SOURCE (the one open architecture decision — see memory
 * heli-project-status). Both paths are implemented; the choice waits on the
 * "does ch6 stream on fresh TX batteries" bench test:
 *   0 = synthesize collective from the fast throttle stick (data[2]). Live at
 *       ~147Hz, but the cut switch (which forces data[2] to idle) drags it —
 *       violates the pilot's "no switch touches pitch" rule. Current default:
 *       works today regardless of TX ch5-8 rate.
 *   1 = read collective from its own channel data[5] (stock architecture) —
 *       SWITCH-IMMUNE (cut only forces data[2]=throttle). Responsiveness is
 *       whatever rate the TX sends ch5-8; only viable if that's fast (healthy
 *       batteries). Flip to 1 once the bench test confirms ch6 streams. */
#define COLLECTIVE_FROM_CH5  1
#define COLL_RAW_LOW   1725  /* data[5] at stick bottom (reversed like throttle) */
#define COLL_RAW_HIGH  1328  /* data[5] at stick top */

/* Left-stick calibration (reversed on this TX). stick 0.0..1.0 = bottom..top:
 *   stick = (THR_RAW_IDLE - data[CH_THROTTLE]) / (THR_RAW_IDLE - THR_RAW_FULL)
 * Drives BOTH the throttle curve (motor) and the pitch curve (collective). */
#define THR_RAW_IDLE   1930  /* stick at bottom */
#define THR_RAW_FULL   1109  /* stick at top (bench-measured 2026-07-07) */
#define THR_IDLE_NORM  0.05f /* stick "at idle" band for arming/tail-off */

/* Throttle CURVE knee: the motor reaches full head speed by this fraction of
 * stick travel, then holds a flat top (above it collective provides the lift,
 * not more RPM — standard CP normal-mode curve). Linear-to-full-at-the-top was
 * far too soft (pilot 2026-07-13: nominal only reached by mid-stick; wanted it
 * by the first ~quarter/third). Lower = snappier spool, higher = softer. */
#define THR_CURVE_KNEE  0.28f

/* Main-motor SLEW limiter (2026-07-16). Fresh-pack symptom: the 0.28 knee
 * sweeps the whole throttle in a third of the stick, the main ESC is still in
 * its own soft-start while the tail FF already assumes full torque — the tail
 * spins up BEFORE the rotor and yaws the nose CW during spool. The slew makes
 * the commanded motor value rise at a bounded rate (full range in ~2s), fall
 * fast (chop must cut now), and — critically — the tail feedforward is fed
 * THIS slewed value, so anti-torque tracks the real spool, not the stick. */
#define MOTOR_SLEW_RISE_PER_S  0.5f   /* full range in 2s on the way up */
#define MOTOR_SLEW_FALL_PER_S  3.0f   /* near-instant on the way down */

/* NORMAL-mode collective curve (community standard: only a fraction of
 * negative pitch below mid-stick — enough to descend, not to tuck): bottom =
 * -COLL_NEG_FRAC of full travel, zero at mid, full positive at top. Applied
 * globally for now (per-mode curves need the mode detector — deferred);
 * symmetric 3D pitch returns when idle-up detection lands. */
#define COLL_NEG_FRAC  0.33f

/* No on-chip mode handling (2026-07-12): the TX applies idle-up/cut onto
 * data[CH_THROTTLE] itself (see CH_THROTTLE note) — the old MODE_IDLEUP_THRESH
 * check against data[CH_MODE] misread a static 1410 as "idle-up engaged" and
 * permanently vetoed the arming interlock. */

/* Collective PITCH CURVE — linear, symmetric about mid-stick, applied in BOTH
 * modes. coll = COLL_DIR*(stick-0.5)*2*COLL_TRAVEL, added to all three swash
 * servos (the CCPM collective). Deliberately modest for first bring-up; widen
 * once travel is bench-checked. COLL_DIR bench-verified 2026-07-07: stick-up
 * must raise the swash, so the direction is inverted (-1). */
#define COLL_TRAVEL  180   /* us, half-range of collective at the swash.
                            * Pilot note 2026-07-12: at collective extremes the
                            * FRONT servo runs out of physical throw slightly
                            * before the two rears — mechanical (linkage/horn
                            * geometry; the mix commands all three equally at
                            * pure collective). If it matters under power, add
                            * a per-servo scale/trim or trim this travel. */
#define COLL_DIR     (+1)  /* flip if the swash goes the wrong way with stick.
                            * +1 as of 2026-07-13: pilot saw collective range
                            * inverted with the -1 that an earlier (07-07) check
                            * had set — re-verify direction holds after this. */

/* Swash servo output clamp — deliberately tighter than 1000..2000 for first
 * bring-up so a wrong mix sign can't drive a linkage into a hard mechanical
 * stop. Widen once travel directions are verified on the bench. */
#define SWASH_MIN_US  1150
#define SWASH_MAX_US  1890

/* Per-servo mechanical direction: +1 normal, -1 if the servo horn/linkage runs
 * the opposite way. A reversed servo has its WHOLE output mirrored about center
 * so collective, pitch and roll all move the swash the right way. Bench-verified
 * 2026-07-05: rear-right servo reversed. (DIR_FRONT was briefly flipped to -1
 * on 2026-07-14 to "fix the front servo" — WRONG: this per-servo flag also
 * inverts the front's COLLECTIVE contribution, which desynced the swash on
 * collective. The real complaint was pitch direction, fixed by CYCLIC_PITCH_DIR
 * below. Reverted to +1.) */
#define DIR_FRONT   (+1)
#define DIR_REAR_L  (+1)
#define DIR_REAR_R  (-1)

/* Cyclic MIX direction (elevator/aileron), separate from the per-servo DIR
 * flags: reverses a whole cyclic axis (all servos + stab together) without
 * touching collective. Set during swash direction setup AFTER the per-servo
 * DIR flags are right on collective. Pitch reversed 2026-07-14. */
#define CYCLIC_PITCH_DIR  (+1)
#define CYCLIC_ROLL_DIR   (+1)

enum {
    OUT_FRONT = 0,   /* front swash servo */
    OUT_REAR_L,      /* rear-left swash servo */
    OUT_REAR_R,      /* rear-right swash servo */
    OUT_MAIN,        /* main motor ESC */
    OUT_TAIL,        /* tail motor ESC */
    OUT_COUNT,
};

/* Physical pin map, verified 2026-07-05 by the bench servo-ID sweep (drive one
 * TIM2 output, see which servo wags). The real wiring is NOT PA0..PA2=swash +
 * PA3=motor as the docs assumed — it is shifted by one: the three swash servos
 * are on PA1/PA2/PA3 and the main motor ESC is on PA0. That mismatch is why the
 * rear-left servo only ever responded to the throttle stick (the firmware was
 * driving TIM2_CCR4/PA3 as the motor). */
static volatile uint32_t *const pwm_out[OUT_COUNT] = {
    [OUT_FRONT]  = &TIM2_CCR2,   /* PA1 */
    [OUT_REAR_L] = &TIM2_CCR4,   /* PA3 */
    [OUT_REAR_R] = &TIM2_CCR3,   /* PA2 */
    [OUT_MAIN]   = &TIM2_CCR1,   /* PA0 */
    [OUT_TAIL]   = &TIM3_CCR3,   /* PB0 */
};
/* ======================================================================= */

/* PWM mode 1 (OCxM=110) + output-compare preload (OCxPE) for one channel,
 * shifted into its low or high half of a CCMRx register. */
#define OC_LOW   ((6u << 4) | (1u << 3))       /* CHn odd  → 0x0068 */
#define OC_HIGH  ((6u << 12) | (1u << 11))     /* CHn even → 0x6800 */

static void pwm_init(void) {
    /* TIM2: PA0=CH1, PA1=CH2, PA2=CH3, PA3=CH4 (swashplate + main throttle) */
    RCC_APB1ENR |= (1u << 0);  /* TIM2 */
    RCC_AHBENR |= (1u << 17);  /* GPIOA */

    /* PA0-PA3: alternate function TIM2 (AF2) */
    for (int i = 0; i < 4; i++) {
        GPIOA_MODER &= ~(3u << (i * 2));
        GPIOA_MODER |= (2u << (i * 2));  /* alternate function */
        GPIOA_AFRL &= ~(0xFu << (i * 4));
        GPIOA_AFRL |= (2u << (i * 4));  /* AF2 = TIM2 */
    }

    TIM2_PSC = PWM_PSC;
    TIM2_ARR = PWM_ARR;
    TIM2_CCMR1 = OC_LOW | OC_HIGH;  /* CH1 + CH2: PWM mode 1, preload */
    TIM2_CCMR2 = OC_LOW | OC_HIGH;  /* CH3 + CH4: PWM mode 1, preload */
    TIM2_CCER = 0x1111;  /* enable CH1..CH4 outputs */
    TIM2_EGR = 1;        /* UG: latch PSC/ARR into shadow registers */
    TIM2_CR1 = (1u << 7) | 1u;  /* ARPE + CEN */

    /* TIM3: PB0=CH3 (tail throttle ESC) */
    RCC_APB1ENR |= (1u << 1);  /* TIM3 */
    RCC_AHBENR |= (1u << 18);  /* GPIOB */

    /* PB0: alternate function TIM3 (AF1) */
    GPIOB_MODER &= ~(3u << 0);
    GPIOB_MODER |= (2u << 0);  /* alternate function */
    GPIOB_AFRL &= ~(0xFu << 0);
    GPIOB_AFRL |= (1u << 0);  /* AF1 = TIM3 */

    TIM3_PSC = PWM_PSC;
    /* Tail ESC gets a FASTER PWM frame than the servos (TIM3 drives ONLY the
     * tail): 2500us period = 400Hz vs the servos' 125Hz on TIM2. At 125Hz the
     * ESC sampled our command up to 8ms late — pure added dead time on a loop
     * already limited by the ESC's ~80ms lag (pilot symptom: overshoot when
     * stopping a pirouette). 400Hz cuts that to <=2.5ms. Pulse width stays
     * 1000-2000us (still fits in 2500us); most ESCs accept 400Hz standard PWM —
     * bench-verify the toy ESC arms and runs before flying. */
    TIM3_ARR = 2500;
    TIM3_CCMR2 = OC_LOW;   /* CH3: PWM mode 1, preload */
    TIM3_CCER = 0x0100;    /* enable CH3 output */
    TIM3_EGR = 1;          /* UG: latch PSC/ARR */
    TIM3_CR1 = (1u << 7) | 1u;  /* ARPE + CEN */
}

#if IMU_POLL_ENABLE
/* Latest stabilization correction, refreshed at IMU cadence in the poll block
 * and applied pre-mix below. Zero pitch/roll axes + tail at centre = "no
 * correction" (disengaged: motor idle, signal loss, or STAB_ENABLE off). */
static StabCorrection_t g_stab = { 0.0f, 0.0f, PWM_CENTER };
#endif

/* CCR is in microseconds directly (1us/tick). Clamp to the servo/ESC band. */
static void pwm_set(volatile uint32_t *ccr, uint16_t us) {
    if (us < PWM_MIN) us = PWM_MIN;
    if (us > PWM_MAX) us = PWM_MAX;
    *ccr = us;
}

/* Write a signed microsecond value clamped to an explicit [lo,hi] window —
 * used for the swash servos, where the CCPM mix can over/undershoot and a
 * tighter-than-full clamp protects the linkages during first bring-up. */
static void pwm_set_clamped(volatile uint32_t *ccr, int32_t us, int32_t lo, int32_t hi) {
    if (us < lo) us = lo;
    if (us > hi) us = hi;
    *ccr = (uint32_t)us;
}

/* FAILSAFE — the SINGLE definition of "no valid RC" output, per the pilot's
 * spec: swashplate dead-centre, both motors off. One function so the two call
 * sites (sustained signal loss, and boot before the first frame) can never
 * drift apart.
 *   - Swash servos -> PWM_CENTER (1520us) = level swashplate. NOT PWM_MIN:
 *     1000us is a servo end-stop, and driving three linked servos there can
 *     bind the swash against its own mechanical limits.
 *   - Main + tail ESC -> PWM_MIN = throttle off.
 * Note: this is triggered ONLY by sustained loss (SFHSS_HasSignal false). The
 * per-frame failsafe BIT (cmd&0x04) is deliberately NOT wired here — the TX
 * broadcasts a failsafe-flagged frame ~every 30s in normal flight (see sfhss.h),
 * so acting on it would kick the swash to centre periodically. */
static void apply_failsafe(void) {
    pwm_set(pwm_out[OUT_FRONT],  PWM_CENTER);
    pwm_set(pwm_out[OUT_REAR_L], PWM_CENTER);
    pwm_set(pwm_out[OUT_REAR_R], PWM_CENTER);
    pwm_set(pwm_out[OUT_MAIN],   PWM_MIN);
    pwm_set(pwm_out[OUT_TAIL],   PWM_MIN);
}

void Reset_Handler(void);
void Default_Handler(void) { while (1); }

#if BENCH_SERVO_ID
volatile uint32_t g_bench_sel;  /* 0..2 = swash output to jog; set over SWD */
#endif

#if TAIL_STEP_TEST
volatile uint8_t  g_step_go;        /* SWD: write 1 to start; firmware sets 0 when done */
volatile uint16_t g_step_n;         /* samples logged so far */
int16_t  g_step_log[256];           /* gyro-Z, dps x10, at the 100Hz IMU tick */
uint16_t g_step_tail[256];          /* commanded tail us at the same tick */
#endif

#if RATE_LOG
typedef struct { int16_t roll, pitch, yaw; } RateRec_t;   /* dps x10 */
/* Format sentinel, same tripwire idea as g_bb_fmt: rate_dump.sh refuses to
 * decode a board whose flashed build doesn't carry exactly this value. */
const uint32_t g_rl_fmt = 0xBB000104u;
RateRec_t g_rl_log[RATE_LOG_N];
volatile uint16_t g_rl_head;        /* total written; newest = (head-1)%N */
#endif

#if TAIL_RELAY_TEST
volatile uint8_t  g_relay_go;       /* mww 1 over SWD to start; 2 = running, 0 = done */
volatile uint16_t g_relay_n;        /* samples logged so far (excludes settle) */
int16_t g_relay_rate[RELAY_N];      /* gyro-Z, dps x10, at the 100Hz IMU tick */
uint8_t g_relay_hi[RELAY_N];        /* relay state: 1 = base+d, 0 = base-d */
float   g_relay_mean;               /* learned pirouette-drift rate, dps (SWD-readable) */
#endif

__attribute__((section(".isr_vector")))
void (*const vector_table[])() = {
    (void (*)())0x20001000, Reset_Handler,
    Default_Handler, Default_Handler, Default_Handler, Default_Handler,
    Default_Handler, Default_Handler, Default_Handler, Default_Handler,
    Default_Handler, Default_Handler, Default_Handler,
    Default_Handler, Default_Handler, Default_Handler,
};

/* SYSCLK 48MHz either way. SysTick runs on HCLK/8 = 6MHz; RVR is set so the
 * counter completes exactly 1,000,000 us per wrap (6,000,000 ticks), and
 * micros() divides the 6MHz tick count by 6 to return real microseconds in
 * [0, 1000000). So every existing us constant stays valid; only the wrap period
 * shrank from 16.7s to 1s (the sigLost failsafe latch already covers the wrap
 * alias). elapsed() below is modulo 1,000,000 to match.
 *
 * SOURCE (2026-07-16): prefer the X1 16MHz crystal on the back of the board
 * (PF0/PF1, pins 2-3 — the stock firmware clocked from it). HSE x3 = 48MHz with
 * crystal ppm accuracy; HSI is only ~1% and drifts with board temperature. That
 * matters here: the S-FHSS slot is 1681us, so 1% of clock error alone is ~17us
 * of hop-window walk, and the hop grid is re-anchored on measured timing that
 * assumes our microsecond is a real microsecond.
 *
 * The crystal is inferred from board photos, never yet proven to oscillate, so
 * HSE start is TIMED OUT and falls back to the old HSI/2 x12 path. A bare
 * "while (!HSERDY)" would brick the heli on a dead/absent crystal. Read
 * g_clk_hse over SWD to see which path booted (nm firmware.elf | grep g_clk). */
volatile uint8_t g_clk_hse;  /* 1 = running on the X1 crystal, 0 = HSI fallback */

static void clock_init(void) {
    FLASH_ACR = 0x11;                 /* prefetch on + 1 wait state (>24MHz) */

    /* HSEON, crystal mode (HSEBYP=0). ~60ms of budget at the 8MHz HSI we boot
     * on — a small SMD crystal settles in single-digit ms. */
    RCC_CR &= ~(1u << 18);            /* HSEBYP = 0: oscillator, not ext clock */
    RCC_CR |= (1u << 16);             /* HSEON */
    for (uint32_t t = 0; t < 100000u && !(RCC_CR & (1u << 17)); t++) { }

    if (RCC_CR & (1u << 17)) {        /* HSERDY */
        RCC_CFGR2 = (RCC_CFGR2 & ~0xFu);  /* PREDIV = /1 -> PLL input 16MHz */
        /* PLLMUL = x3 (bits[21:18] = mul-2 = 0001), PLLSRC = HSE/PREDIV (10b at
         * bits[16:15]). Both must be set while the PLL is still off. */
        RCC_CFGR = (RCC_CFGR & ~((0xFu << 18) | (0x3u << 15)))
                 | (1u << 18) | (0x2u << 15);
        g_clk_hse = 1;
    } else {
        RCC_CR &= ~(1u << 16);        /* HSEOFF — do not leave a dead osc driven */
        /* PLLMUL = x12 (bits[21:18]=1010), PLLSRC = HSI/2 (00b). */
        RCC_CFGR = (RCC_CFGR & ~((0xFu << 18) | (0x3u << 15))) | (10u << 18);
        g_clk_hse = 0;
    }

    RCC_CR |= (1u << 24);             /* PLLON */
    while (!(RCC_CR & (1u << 25)));   /* wait PLLRDY */
    RCC_CFGR = (RCC_CFGR & ~0x3u) | 0x2u;        /* SW = PLL */
    while (((RCC_CFGR >> 2) & 0x3u) != 0x2u);    /* wait SWS = PLL */
}

static void systick_init(void) {
    STK_RVR = 6000000u - 1u;   /* 6MHz / 6e6 = 1Hz wrap = 1,000,000 us */
    STK_CVR = 0;
    STK_CSR = 1; /* ENABLE, CLKSOURCE=HCLK/8, no interrupt */
}

static uint32_t micros(void) {
    return ((6000000u - 1u) - STK_CVR) / 6u;   /* real us, [0, 1000000) */
}

/* Radio reception is polled from the main loop (SFHSS_Poll below), not a timer
 * ISR. An earlier attempt moved it into a TIM14 ISR to decouple it from the IMU
 * read, but at 8MHz the bit-bang poll was too slow for a fixed-rate ISR (it
 * either starved the loop or hopped out of sync). Raising the clock to 48MHz
 * (clock_init) made the main loop iterate fast enough that plain polling keeps
 * the receiver in sync AND the (now ~6x shorter) IMU read no longer gaps it —
 * measured ~147 frames/s with the IMU active. See memory
 * heli-radio-interrupt-architecture. */

/* Normalized left stick 0..1 (bottom..top) from the throttle channel. */
static float throttle_stick_norm(void) {
    int32_t raw = (int32_t)g_sfhss.data[CH_THROTTLE];
    float s = (float)(THR_RAW_IDLE - raw) / (float)(THR_RAW_IDLE - THR_RAW_FULL);
    if (s < 0.0f) s = 0.0f;
    if (s > 1.0f) s = 1.0f;
    return s;
}

/* Throttle curve: rise to full by THR_CURVE_KNEE of stick travel, then flat.
 * Shared by the mix block (main motor) and the stabilization call (the tail's
 * anti-torque feedforward scales with THIS drive). */
static float throttle_curve(float stick) {
    float thr = stick / THR_CURVE_KNEE;
    if (thr > 1.0f) thr = 1.0f;
    return thr;
}

/* Collective after the 3-point normal curve, -1..+1 (see COLL_NEG_FRAC).
 * Single source for both the swash mix and the tail's blade-loading FF. */
static float collective_curve(void) {
#if COLLECTIVE_FROM_CH5
    float p = (float)(COLL_RAW_LOW - (int32_t)g_sfhss.data[CH_COLLECTIVE])
              / (float)(COLL_RAW_LOW - COLL_RAW_HIGH);
#else
    float p = throttle_stick_norm();
#endif
    if (p < 0.0f) p = 0.0f;
    if (p > 1.0f) p = 1.0f;
    if (p < 0.5f) return -(0.5f - p) * 2.0f * COLL_NEG_FRAC;
    return (p - 0.5f) * 2.0f;
}

/* Normalized collective 0..1 (0.5 = flat pitch) for the stabilizer's tail
 * feedforward — reflects the CURVED value so blade loading is truthful. */
static float collective_norm(void) {
    return 0.5f + 0.5f * collective_curve();
}

/* Main-motor slew state (see MOTOR_SLEW_*): the value actually driving the
 * ESC and the tail feedforward. Reset on failsafe/disarm so re-arm respools. */
static float g_motor_slew = 0.0f;

#if IMU_POLL_ENABLE
/* True when the throttle stick is at/below idle (motors treated as off). Used
 * by the IMU block to gate stabilization before the mix block has run this
 * iteration. */
static uint8_t throttle_is_idle(void) {
    return throttle_stick_norm() < THR_IDLE_NORM;
}
#endif


void Reset_Handler(void) {
    /* .data copy + .bss zero */
    uint32_t *src = &_sidata;
    for (uint32_t *dst = &_sdata; dst < &_edata;) *dst++ = *src++;
    for (uint32_t *dst = &_sbss; dst < &_ebss;) *dst++ = 0;

    clock_init(); /* 48MHz PLL — do this before anything timing-dependent */
    IWDG_KR = IWDG_REFRESH;

    systick_init();
    LED_Init();
    SPI_RC_Init();
    pwm_init();
    SFHSS_Init(); /* reset + config + 30-channel calibration, ~50ms */

#if BENCH_SERVO_ID
    /* pin->servo identification loop: motors OFF. The output selected by
     * g_bench_sel (0=PA0, 1=PA1, 2=PA2; set over SWD) OSCILLATES 1300<->1700
     * so exactly one servo visibly wags; the other two hold 1500us. */
    pwm_set(pwm_out[OUT_TAIL], PWM_MIN);  /* tail motor stays off */
    {
        uint16_t sweep = 1300;
        int16_t dir = 4;
        uint32_t last = micros();
        LED_SetRed(1);  /* power indicator on */
        while (1) {
            IWDG_KR = IWDG_REFRESH;
            uint32_t t = micros();
            LED_SetBlue((t >> 18) & 1);  /* ~1.5Hz heartbeat: board is alive */
            /* step the sweep every ~2ms → ~0.2s per end-to-end pass */
            if (elapsed_us(last, t) >= 2000u) {
                last = t;
                sweep += dir;
                if (sweep >= 1700) dir = -4;
                if (sweep <= 1300) dir = 4;
            }
            uint32_t sel = g_bench_sel;
            /* sel 0..3 sweep PA0..PA3; non-selected TIM2 outputs hold 1500us
             * (a steady mid pulse won't arm an ESC, so this is safe even if
             * one of these pins turns out to drive the main motor). */
            pwm_set(pwm_out[OUT_FRONT],  sel == 0 ? sweep : 1500);
            pwm_set(pwm_out[OUT_REAR_L], sel == 1 ? sweep : 1500);
            pwm_set(pwm_out[OUT_REAR_R], sel == 2 ? sweep : 1500);
            pwm_set(pwm_out[OUT_MAIN],   sel == 3 ? sweep : 1500);
        }
    }
#endif

#if IMU_POLL_ENABLE
    MPU6500_Init();
    if (g_mpu_ok) MPU6500_CalibrateGyro(200); /* ~200ms, airframe must be still */
    Orientation_Init();
    Stabilize_Init();
#endif

#if IMU_POLL_ENABLE
    uint32_t last_imu_us = micros();
    uint32_t imu_tick = 0;
    /* IMU PIPELINE (2026-07-12, see memory heli-stab-radio-budget): the read,
     * the orientation update and the stabilization step used to run as ONE
     * atomic block inside a single loop pass — a multi-ms stall during which
     * SFHSS_Poll never ran. With STAB_ENABLE that stall crossed the hop-timing
     * tolerance and slot loss went 3.1% -> 57% (measured). Now each piece runs
     * in its OWN loop pass, so the radio is polled between every pair of
     * atoms. Costs 2 extra passes (~0.5ms) of pipeline latency per 10ms tick —
     * nothing for a hover stabilizer. maxPollGap (SWD) is the tripwire that
     * must be re-checked whenever this hot path changes. */
    uint8_t imu_stage = 0;      /* 0=await tick, 1=orient pending, 2=stab pending */
    uint8_t imu_use_accel = 0;
    float imu_dt_s = 0.0f;
    MPU6500_Sample_t imu_sample;
    float stab_gain = 0.0f;     /* engage ramp 0..1, see STAB_RAMP_S */
#endif

    /* Arming state. With ARMING_INTERLOCK=1 the ESCs stay OFF after boot (and
     * after signal loss) until the throttle channel is seen at idle once with
     * live signal; with 0 (current, stock semantics) any live signal arms.
     * Signal loss disarms either way — with the interlock off that only
     * matters as "failsafe wins until frames flow again". */
    uint8_t motor_armed = 0;

#if TAIL_STEP_TEST
    uint8_t  step_run = 0;
    uint16_t step_cmd = 0;
#endif

#if TAIL_RELAY_TEST
    uint8_t  relay_run = 0;    /* 0=idle, 1=settle (learn mean), 2=relay */
    uint8_t  relay_hi = 0;
    uint16_t relay_settle = 0; /* settle ticks elapsed */
#endif

    /* Start the 2kHz radio-poll ISR only now — AFTER the IMU probe + gyro
     * calibration. Those are blocking bit-bang I2C bursts on the same GPIO port
     * as the CC2500; letting the radio ISR preempt them during init disrupted
     * the I2C and left g_mpu_ok = 0. The radio is only in FINDING at boot, so
     * the ~200ms it waits for the IMU to settle costs nothing. */
    while (1) {
        IWDG_KR = IWDG_REFRESH;

        uint32_t now = micros();
        SFHSS_Poll(now);

#if IMU_POLL_ENABLE
        /* One pipeline stage per pass — never more (see declaration above). */
        if (g_mpu_ok) {
            if (imu_stage == 0) {
                uint32_t dt_us = elapsed_us(last_imu_us, now);
                /* Start the I2C read only inside the slot's dead zone (just
                 * hopped, next packet >=1.6ms away) so the ~0.5-1.3ms burst
                 * can't collide with packet arrival or the hop deadline.
                 * Escape hatch: if the quiet window somehow never comes, read
                 * anyway after 3 periods so the gyro never starves. */
                if (dt_us >= IMU_PERIOD_US &&
                    (SFHSS_QuietForImu(now) || dt_us >= 3u * IMU_PERIOD_US)) {
                    last_imu_us = now;
                    imu_use_accel = ((imu_tick++ % IMU_ACCEL_EVERY) == 0u);
                    if (imu_use_accel) MPU6500_GetSample(&imu_sample);      /* accel+gyro, 14B */
                    else               MPU6500_GetGyroSample(&imu_sample);  /* gyro only, 6B */
                    imu_dt_s = (float)dt_us / 1000000.0f;
                    imu_stage = 1;
                }
            } else if (imu_stage == 1) {
                /* on_ground for the yaw auto-trim: throttle sitting at idle
                 * with live signal. Signal required — with the TX off the
                 * machine might be carried around, and idle can't be told
                 * from failsafe. */
                Orientation_Update(&imu_sample, imu_dt_s, imu_use_accel,
                                   SFHSS_HasSignal(now) && throttle_is_idle());
#if RATE_LOG
                {
                    static uint8_t rl_decim;
                    /* Same engage condition as the stab loop: records while
                     * flying, freezes at throttle idle so landing preserves
                     * the last ~7.5s aloft. */
                    if (SFHSS_HasSignal(now) && !throttle_is_idle()) {
                        if (++rl_decim >= RATE_LOG_DECIM) {
                            rl_decim = 0;
                            RateRec_t *r = &g_rl_log[g_rl_head % RATE_LOG_N];
                            r->roll  = (int16_t)(g_orientation.roll_rate_dps  * 10.0f);
                            r->pitch = (int16_t)(g_orientation.pitch_rate_dps * 10.0f);
                            r->yaw   = (int16_t)(g_orientation.yaw_rate_dps   * 10.0f);
                            g_rl_head++;
                        }
                    }
                }
#endif
#if TAIL_STEP_TEST
                if (step_run && g_step_n < 256) {
                    g_step_log[g_step_n]  = (int16_t)(g_orientation.yaw_rate_dps * 10.0f);
                    g_step_tail[g_step_n] = step_cmd;
                    g_step_n++;
                }
#endif
#if TAIL_RELAY_TEST
                /* Relay decision runs on the fresh-gyro tick (100Hz), the
                 * actual pwm write is in the actuation block below. */
                if (relay_run) {
                    float rate = g_orientation.yaw_rate_dps;
                    if (relay_run == 1) {
                        /* Settle: constant base thrust, learn the pirouette
                         * drift the relay will oscillate around. tau ~0.5s. */
                        g_relay_mean += 0.02f * (rate - g_relay_mean);
                        if (++relay_settle >= 150) { relay_run = 2; relay_hi = 0; }
                    } else {
                        /* Negative feedback: too fast nose-right -> less
                         * thrust. Hysteresis holds the last state in between. */
                        if      (rate > g_relay_mean + RELAY_HYST_DPS) relay_hi = 0;
                        else if (rate < g_relay_mean - RELAY_HYST_DPS) relay_hi = 1;
                        if (g_relay_n < RELAY_N) {
                            g_relay_rate[g_relay_n] = (int16_t)(rate * 10.0f);
                            g_relay_hi[g_relay_n]   = relay_hi;
                            g_relay_n++;
                        } else {
                            relay_run = 0; g_relay_go = 0;   /* done */
                        }
                    }
                }
#endif
                imu_stage = 2;
            } else {
                /* Step the stabilization loops at the IMU cadence and stash
                 * the correction for the pre-mix injection below. Engage only
                 * while flying with signal and above idle; otherwise reset the
                 * PIDs (clean re-engage, no windup) and apply nothing. */
                if (STAB_ENABLE && SFHSS_HasSignal(now) &&
                    (STAB_BENCH_TILT || !throttle_is_idle())) {
                    StabCorrection_t s =
                        Stabilize_Compute(g_sfhss.data[CH_ROLL], g_sfhss.data[CH_PITCH],
                                          g_sfhss.data[CH_RUDDER],
                                          g_motor_slew, /* real spool, not the stick */
                                          collective_norm(), imu_dt_s);
                    /* Ease the correction in (STAB_RAMP_S) so engaging stab —
                     * e.g. by flipping idle-up, which forces throttle full —
                     * doesn't snap the swash to the tilt correction. */
                    if (stab_gain < 1.0f) {
                        stab_gain += imu_dt_s / STAB_RAMP_S;
                        if (stab_gain > 1.0f) stab_gain = 1.0f;
                    }
                    g_stab.pitch_axis = s.pitch_axis * stab_gain;
                    g_stab.roll_axis  = s.roll_axis  * stab_gain;
                    /* Tail spools up from OFF toward base+correction over the
                     * ramp — an overpowered tail motor jumping straight to its
                     * anti-torque base is a bench hazard. */
                    g_stab.tail_us = (uint16_t)(PWM_MIN +
                        (int32_t)(((int32_t)s.tail_us - PWM_MIN) * stab_gain));
                } else {
                    Stabilize_Reset();
                    stab_gain = 0.0f;
                    g_stab.pitch_axis = 0.0f;
                    g_stab.roll_axis = 0.0f;
                    g_stab.tail_us = PWM_CENTER;
                }
                imu_stage = 0;
            }
        }
#endif

        /* Map RC channels to PWM outputs. Gate on HasSignal (recent frame),
         * not the instantaneous CONNECTED phase — the phase flickers every hop
         * and would chatter the outputs against the failsafe floor. */
        if (SFHSS_HasSignal(now)) {
            /* --- normalize raw TX axes (this TX sends raw sticks) --- */
            int32_t roll  = (int32_t)(((float)g_sfhss.data[CH_ROLL]  - CYCLIC_CENTER) * CYCLIC_GAIN);
            int32_t pitch = (int32_t)(((float)g_sfhss.data[CH_PITCH] - CYCLIC_CENTER) * CYCLIC_GAIN);

            /* Left stick 0..1 (bottom..top) from the fast throttle channel —
             * this one stick drives both collective and throttle. */
            float stick = (float)(THR_RAW_IDLE - (int32_t)g_sfhss.data[CH_THROTTLE])
                          / (float)(THR_RAW_IDLE - THR_RAW_FULL);
            if (stick < 0.0f) stick = 0.0f;
            if (stick > 1.0f) stick = 1.0f;

            /* Collective — linear pitch curve, -TRAVEL at bottom .. +TRAVEL at
             * top, added to all three swash servos. Source per COLLECTIVE_FROM_CH5. */
#if COLLECTIVE_FROM_CH5
            /* Stock architecture: from data[5], its own (switch-immune) channel,
             * reversed like throttle. Holds its last value between ch5-8 frames. */
            float coll_pos = (float)(COLL_RAW_LOW - (int32_t)g_sfhss.data[CH_COLLECTIVE])
                             / (float)(COLL_RAW_LOW - COLL_RAW_HIGH);
#else
            /* Synthesized from the fast throttle stick (data[2]). */
            float coll_pos = stick;
#endif
            (void)coll_pos;
            /* 3-point NORMAL pitch curve via collective_curve() (single source
             * shared with the tail feedforward, see COLL_NEG_FRAC). */
            int32_t coll = (int32_t)(COLL_DIR * collective_curve() * (float)COLL_TRAVEL);

            /* Stick/channel at idle — for arming and tail-off. Note this reads
             * the THROTTLE CHANNEL, which already carries the TX's mode
             * switches: idle-up forces it full, cut forces it here (idle). */
            uint8_t stick_idle = (stick < THR_IDLE_NORM);

            /* Arming. With the interlock: throttle channel must be seen at
             * idle once with live signal. Without (stock semantics, pilot's
             * 2026-07-12 call): signal alone arms. */
#if ARMING_INTERLOCK
            if (stick_idle) motor_armed = 1;
#else
            motor_armed = 1;
#endif

            /* Motor actually producing thrust — drives tail-active and
             * swash-recentre. */
            uint8_t motor_live = motor_armed && !stick_idle;

            /* --- 120° CCPM swash mix (H-3: 1 front servo + 2 rear) ---
             * Pins are corrected above (front=PA1, rear-left=PA3, rear-right=
             * PA2). Standard H-3 geometry: collective moves all three together;
             * pitch drives the front servo full and both rears half the other
             * way; roll splits the two rears while the front (fore-aft axis)
             * stays put. Mix directions + per-servo DIR flags bench-verified
             * 2026-07-05 (servo-ID sweep across collective/pitch/roll). */
            /* Pre-mix stabilization injection: add the self-level correction
             * to the raw pitch/roll axes so the CCPM mix + DIR flags route and
             * mirror it exactly like a stick nudge. Zero when disengaged. */
            int32_t pitch_s = pitch;
            int32_t roll_s  = roll;
#if IMU_POLL_ENABLE
            pitch_s += (int32_t)g_stab.pitch_axis;
            roll_s  += (int32_t)g_stab.roll_axis;
#endif
            /* Elevator/aileron MIX direction (applied after stab so stick and
             * stabilization flip together — keeps their relationship). Reversed
             * pitch 2026-07-14: after DIR_FRONT was corrected on collective, the
             * fore-aft tilt came out backwards; this is the standard "reverse
             * elevator" step and does NOT touch collective. */
            pitch_s *= CYCLIC_PITCH_DIR;
            roll_s  *= CYCLIC_ROLL_DIR;
            int32_t front  = CYCLIC_CENTER + DIR_FRONT  * (coll + pitch_s);
            int32_t rear_l = CYCLIC_CENTER + DIR_REAR_L * (coll - pitch_s / 2 + roll_s);
            int32_t rear_r = CYCLIC_CENTER + DIR_REAR_R * (coll - pitch_s / 2 - roll_s);

#if SWASH_RECENTER_ON_IDLE
            if (!motor_live) {
                /* Motor off / parked: recentre the swash so a parked heli
                 * doesn't sit with collective/cyclic cocked. */
                front = rear_l = rear_r = PWM_CENTER;
            }
#endif
            pwm_set_clamped(pwm_out[OUT_FRONT],  front,  SWASH_MIN_US, SWASH_MAX_US);
            pwm_set_clamped(pwm_out[OUT_REAR_L], rear_l, SWASH_MIN_US, SWASH_MAX_US);
            pwm_set_clamped(pwm_out[OUT_REAR_R], rear_r, SWASH_MIN_US, SWASH_MAX_US);

            /* Main motor ESC → standard 1000..2000us, straight from the
             * throttle channel. All mode behavior (idle-up's constant head
             * speed, cut) is already baked into that channel by the TX. */
            /* Slew the motor toward its curve target (see MOTOR_SLEW_*). */
            {
                static uint32_t slew_prev = 0;
                static uint8_t  slew_seen = 0;
                float target = motor_armed ? throttle_curve(stick) : 0.0f;
                float dt = slew_seen ? (float)elapsed_us(slew_prev, now) / 1e6f : 0.0f;
                slew_prev = now; slew_seen = 1;
                float step_up = MOTOR_SLEW_RISE_PER_S * dt;
                float step_dn = MOTOR_SLEW_FALL_PER_S * dt;
                if (target > g_motor_slew + step_up)      g_motor_slew += step_up;
                else if (target < g_motor_slew - step_dn) g_motor_slew -= step_dn;
                else                                      g_motor_slew = target;
            }
            pwm_set(pwm_out[OUT_MAIN],
                    (uint16_t)(PWM_MIN + g_motor_slew * (PWM_MAX - PWM_MIN)));

            /* Tail motor ESC.
             *  - Idle: off (PWM_MIN) so at power-up it sees a steady zero and
             *    arms (feeding centred rudder ~1520 was why it wouldn't arm).
             *  - Running + heading-hold gyro: gyro-corrected tail value,
             *    centred on the hover anti-torque base. Tail-rotor direction
             *    confirmed 2026-07-05, so running this closed loop is safe.
             *  - Running without the gyro (STAB_ENABLE=0 / IMU off): provisional
             *    add-only rudder->thrust map (a single fixed-pitch tail motor
             *    can only add thrust, so half of rudder travel stays at idle). */
            if (!motor_live) {
                pwm_set(pwm_out[OUT_TAIL], PWM_MIN);
            }
#if IMU_POLL_ENABLE
            else if (STAB_ENABLE && g_mpu_ok) {
                pwm_set(pwm_out[OUT_TAIL], g_stab.tail_us);
            }
#endif
            else {
                int32_t yaw = (int32_t)g_sfhss.data[CH_RUDDER] - CYCLIC_CENTER;
                if (yaw < 0) yaw = 0;
                pwm_set(pwm_out[OUT_TAIL], (uint16_t)(PWM_MIN + yaw * 2));
            }
        } else {
            /* Total signal loss failsafe, continuously updated (never just
             * once): swashplate centred, both motors off. See apply_failsafe(). */
            apply_failsafe();
            /* Do NOT zero the slew here: the rotor keeps spinning through a
             * brief RF outage (inertia), and zeroing meant power crawled back
             * over ~2s after recovery while the heli fell. Instead the slew
             * COASTS DOWN like the freewheeling rotor: a short blip recovers
             * near-instantly, a long outage (rotor truly stopped) ends near
             * zero and respools honestly. */
            {
                static uint32_t fs_prev = 0;
                static uint8_t  fs_seen = 0;
                float dt = fs_seen ? (float)elapsed_us(fs_prev, now) / 1e6f : 0.0f;
                fs_prev = now; fs_seen = 1;
                g_motor_slew -= 0.25f * dt;   /* rotor freewheel decay */
                if (g_motor_slew < 0.0f) g_motor_slew = 0.0f;
            }
            /* Disarm on signal loss: re-arming requires the throttle be seen at
             * idle again once the link is back (see interlock note above). */
            motor_armed = 0;
        }

#if TAIL_STEP_TEST
        /* Runs AFTER the mix/failsafe writes so its outputs win the pass.
         * Main motor forced off for the whole sequence; tail follows the
         * step schedule regardless of RC. */
        if (g_step_go == 1 && !step_run) {
            step_run = 1; g_step_go = 2;
            g_step_n = 0;
        }
        if (step_run) {
            /* Schedule keyed to the SAMPLE COUNT (10ms IMU ticks), not
             * elapsed_us — micros() wraps every 1s, shorter than the test. */
            uint16_t n = g_step_n;
            if (n < 100)       step_cmd = 1150;  /* settle at base, 1s */
            else if (n < 200)  step_cmd = 1400;  /* UP step, 1s */
            else if (n < 256)  step_cmd = 1150;  /* DOWN step, 0.56s */
            else { step_run = 0; g_step_go = 0; step_cmd = 0; }
            pwm_set(pwm_out[OUT_MAIN], PWM_MIN);
            pwm_set(pwm_out[OUT_TAIL], step_run ? step_cmd : PWM_MIN);
        }
#endif

#if TAIL_RELAY_TEST
        /* Runs AFTER the mix/failsafe writes so its outputs win the pass —
         * same convention as TAIL_STEP_TEST. Main motor forced off. */
        if (g_relay_go == 1 && !relay_run) {
            relay_run = 1; g_relay_go = 2;
            relay_settle = 0; g_relay_n = 0;
            g_relay_mean = g_orientation.yaw_rate_dps;  /* seed, no cold jump */
        }
        if (relay_run) {
            uint16_t cmd = (relay_run == 1) ? RELAY_BASE_US
                         : (relay_hi ? RELAY_BASE_US + RELAY_D_US
                                     : RELAY_BASE_US - RELAY_D_US);
            pwm_set(pwm_out[OUT_MAIN], PWM_MIN);
            pwm_set(pwm_out[OUT_TAIL], cmd);
        }
#endif

        switch (g_sfhss.phase) {
        case SFHSS_PH_CONNECTED:
            LED_SetBlue(1);
            /* Red while connected: solid = disarmed (interlock builds only);
             * SLOW BLINK = IMU DEAD (g_mpu_ok=0) — stabilization silently off,
             * DO NOT FLY. Added 2026-07-16 after a brownout reboot left the
             * IMU down with no outward sign. Dark = armed and healthy. */
            if (!motor_armed)       LED_SetRed(1);
            else if (!g_mpu_ok)     LED_SetRed((now >> 17) & 1);
            else                    LED_SetRed(0);
            break;
        case SFHSS_PH_FINDING:
            /* slow blue blink while searching for a transmitter */
            LED_SetBlue((now >> 17) & 1);
            LED_SetRed(1);
            break;
        default:
            /* reconnecting: fast blue blink */
            LED_SetBlue((now >> 15) & 1);
            LED_SetRed(1);
            break;
        }
    }
}
