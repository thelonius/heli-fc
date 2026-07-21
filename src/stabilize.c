#include "stabilize.h"

/* Tail ESC neutral / hover anti-torque base, and the rudder spring-centre the
 * yaw-rate setpoint is measured from. Matches main.c's CYCLIC_CENTER/PWM band
 * (not shared via header to keep this module off the PWM driver). */
#define SRV_MIN     1000
#define SRV_CENTER  1520
#define SRV_MAX     2000

static PID_t s_roll_pid, s_pitch_pid, s_yaw_pid;

/* Output LPF state for the swash correction (see STAB_OUTPUT_LPF_HZ). */
static float s_pitch_axis_lpf, s_roll_axis_lpf;

/* Input LPF state for the cyclic rate feedback (see STAB_CYCLIC_IN_LPF_HZ). */
static float s_roll_rate_lpf, s_pitch_rate_lpf;

/* Tail working-point probe (see the block that fills these, at the end of the
 * tail section). Read over SWD after landing WITHOUT cycling power. */
volatile float g_tail_ema_us, g_tail_max_us;
volatile uint32_t g_tail_sat_ms;

/* Filtered D state per cyclic axis (see STAB_CYCLIC_D_* in the header):
 * [0] previous measurement, [1..2] the two cascaded LPF poles.
 * The step function itself lives below lpf_step, which it uses. */
static float s_roll_d[3], s_pitch_d[3];

#if STAB_CYCLIC_NOTCH_ENABLE
/* 7.5Hz notch state, direct form 1: [x1 x2 y1 y2] per cyclic axis
 * (see STAB_NOTCH_* in the header for the war story and coefficients). */
static float s_roll_notch[4], s_pitch_notch[4];

static float notch_step(float st[4], float x) {
    float y = STAB_NOTCH_B0 * x + STAB_NOTCH_B1 * st[0] + STAB_NOTCH_B2 * st[1]
            - STAB_NOTCH_A1 * st[2] - STAB_NOTCH_A2 * st[3];
    st[1] = st[0]; st[0] = x;
    st[3] = st[2]; st[2] = y;
    return y;
}
#endif

/* Setpoint LPF state + previous value: LPF feeds both the iterm-relax HPF
 * (set - LPF) and the B-term filtered derivative (d(LPF)/dt). */
static float s_roll_set_lpf, s_pitch_set_lpf;
static float s_roll_set_prev, s_pitch_set_prev;

/* Yaw setpoint LPF for the yaw iterm relax (see STAB_YAW_RELAX_DPS). */
static float s_yaw_set_lpf;

#if STAB_CYCLIC_RATE_MODE == 2
/* MODE 2: the attitude the stick has steered us to, degrees. This is the whole
 * point of the mode — the angle the machine REMEMBERS between stick inputs.
 * Zeroed by Stabilize_Reset(), which main.c calls whenever the throttle sits at
 * idle: that is the stock's Math_QuaternionIdentity() snap-to-horizon. */
static float s_roll_target_deg, s_pitch_target_deg;
/* Was the stick steering this axis last tick? On the deflected->released edge
 * the target CAPTURES the actual angle — see the note in Stabilize_Compute. */
static uint8_t s_roll_steering, s_pitch_steering;
#endif

/* Spool-term state: previous motor_frac for the d(thr)/dt feedforward. Seeded
 * on the first compute after a reset so engaging doesn't fake a spool spike. */
static float s_prev_motor_frac;
static uint8_t s_prev_motor_valid;

#if STAB_BLACKBOX
/* Yaw blackbox (see stabilize.h). The log itself survives Stabilize_Reset —
 * main.c resets on every throttle-idle pass, and wiping there would destroy
 * the flight record right after landing. Only the window accumulators reset. */
const uint32_t g_bb_fmt = STAB_BB_FMT;   /* dumper checks this before trusting records */
StabBBRec_t g_bb_log[STAB_BB_N];
volatile uint16_t g_bb_head;
static float s_bb_dt_acc, s_bb_err_acc, s_bb_tail_acc;
static float s_bb_roll_acc, s_bb_pitch_acc, s_bb_rollc_acc, s_bb_pitchc_acc;
#endif

#define TWO_PI  6.2831853f

/* One-pole low-pass step: y += alpha*(x - y), with alpha derived from the real
 * dt so the cutoff stays fixed across the IMU-read jitter. tau = 1/(2*pi*fc);
 * alpha = dt/(dt + tau), clamped to 1 (a long-gap tick just passes through). */
static float lpf_step(float y, float x, float dt_s, float fc_hz) {
    float tau = 1.0f / (TWO_PI * fc_hz);
    float alpha = dt_s / (dt_s + tau);
    if (alpha > 1.0f) alpha = 1.0f;
    return y + alpha * (x - y);
}

/* D on the MEASUREMENT derivative, two-pole low-passed (see STAB_CYCLIC_D_*).
 * Returns the term ready to be SUBTRACTED from the correction: with
 * error = setpoint - meas, d(error)/dt keeps -d(meas)/dt, and dropping the
 * setpoint half is what removes the derivative kick on stick moves. */
static float dterm_step(float st[3], float meas, float dt_s) {
    float d = (dt_s > 0.0f) ? (meas - st[0]) / dt_s : 0.0f;
    st[0] = meas;
    st[1] = lpf_step(st[1], d,     dt_s, STAB_CYCLIC_D_LPF_HZ);
    st[2] = lpf_step(st[2], st[1], dt_s, STAB_CYCLIC_D_LPF_HZ);
    return st[2];
}

/* Deadband that SUBTRACTS the edge instead of stepping over it: 0 inside the
 * band, and just outside it the result starts from 0 rather than jumping to
 * db. Keeps the stick response continuous through the band. */
static float deadband_f(float x, float db) {
    if (x >  db) return x - db;
    if (x < -db) return x + db;
    return 0.0f;
}

/* Rotorflight's transition(): linear blend a->b as x crosses [lo, hi]. */
static float transition_f(float x, float lo, float hi, float a, float b) {
    if (x <= lo) return a;
    if (x >= hi) return b;
    return a + (b - a) * (x - lo) / (hi - lo);
}

void PID_Reset(PID_t *pid) {
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

float PID_Update(PID_t *pid, float setpoint, float measurement, float dt_s) {
    return PID_UpdateRelax(pid, setpoint, measurement, dt_s, 1.0f);
}

/* iterm_relax scales only the INTEGRAL ACCUMULATION this tick (RF itermRelax):
 * 1.0 = normal, 0 = integral frozen while the setpoint is moving fast. */
float PID_UpdateRelax(PID_t *pid, float setpoint, float measurement, float dt_s,
                      float iterm_relax) {
    float error = setpoint - measurement;

    pid->integral += error * dt_s * iterm_relax;
    if (pid->integral > pid->integral_limit) pid->integral = pid->integral_limit;
    if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;

    float derivative = (dt_s > 0.0f) ? (error - pid->prev_error) / dt_s : 0.0f;
    pid->prev_error = error;

    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    if (output > pid->output_limit) output = pid->output_limit;
    if (output < -pid->output_limit) output = -pid->output_limit;
    return output;
}

/* Field-by-field so the freestanding, -nodefaultlibs build never emits a
 * memset/memcpy for the compound-literal zero-fill (there is no libc here). */
static void pid_init(PID_t *p, float kp, float ki, float kd,
                     float integral_limit, float output_limit) {
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->integral = 0.0f;
    p->integral_limit = integral_limit;
    p->output_limit = output_limit;
    p->prev_error = 0.0f;
}

void Stabilize_Init(void) {
#if STAB_CYCLIC_RATE_MODE
    /* Modes 1 and 2 share the SAME rate PID: in mode 2 it is the inner loop of
     * the cascade, fed by the angle error instead of the stick. */
    pid_init(&s_roll_pid, STAB_CYCLIC_RATE_KP, STAB_CYCLIC_RATE_KI,
             STAB_CYCLIC_RATE_KD, STAB_CYCLIC_RATE_I_LIMIT, STAB_CYCLIC_RATE_OUT_LIMIT);
    pid_init(&s_pitch_pid, STAB_CYCLIC_RATE_KP, STAB_CYCLIC_RATE_KI,
             STAB_CYCLIC_RATE_KD, STAB_CYCLIC_RATE_I_LIMIT, STAB_CYCLIC_RATE_OUT_LIMIT);
#else
    pid_init(&s_roll_pid, STAB_ROLL_KP, STAB_ROLL_KI, STAB_ROLL_KD,
             STAB_SWASH_INTEGRAL_LIMIT, STAB_SWASH_OUTPUT_LIMIT);
    pid_init(&s_pitch_pid, STAB_PITCH_KP, STAB_PITCH_KI, STAB_PITCH_KD,
             STAB_SWASH_INTEGRAL_LIMIT, STAB_SWASH_OUTPUT_LIMIT);
#endif
    pid_init(&s_yaw_pid, STAB_YAW_KP, STAB_YAW_KI, STAB_YAW_KD,
             STAB_YAW_INTEGRAL_LIMIT, STAB_YAW_OUTPUT_LIMIT);
}

void Stabilize_Reset(void) {
    PID_Reset(&s_roll_pid);
    PID_Reset(&s_pitch_pid);
    PID_Reset(&s_yaw_pid);
    /* Clear the LPFs so re-engage doesn't start from stale values. */
    s_pitch_axis_lpf = 0.0f;
    s_roll_axis_lpf = 0.0f;
    s_roll_rate_lpf = 0.0f;
    s_pitch_rate_lpf = 0.0f;
    for (int i = 0; i < 3; i++) { s_roll_d[i] = 0.0f; s_pitch_d[i] = 0.0f; }
#if STAB_CYCLIC_NOTCH_ENABLE
    for (int i = 0; i < 4; i++) { s_roll_notch[i] = 0.0f; s_pitch_notch[i] = 0.0f; }
#endif
    s_roll_set_lpf = 0.0f;
    s_pitch_set_lpf = 0.0f;
    s_roll_set_prev = 0.0f;
    s_pitch_set_prev = 0.0f;
    s_yaw_set_lpf = 0.0f;
    s_prev_motor_valid = 0;  /* re-seed the spool term on next compute */
#if STAB_BLACKBOX
    /* Window accumulators only — the LOG must survive this call: main.c
     * resets on every throttle-idle pass, i.e. right after landing, and the
     * whole point of the blackbox is reading the flight afterwards. */
    s_bb_dt_acc = s_bb_err_acc = s_bb_tail_acc = 0.0f;
    s_bb_roll_acc = s_bb_pitch_acc = s_bb_rollc_acc = s_bb_pitchc_acc = 0.0f;
#endif
#if STAB_CYCLIC_RATE_MODE == 2
    /* Stock's Math_QuaternionIdentity(): the steered target snaps back to
     * horizon. main.c calls this at throttle idle, so lowering the throttle is
     * the pilot's "forget the bank, level off" gesture — as on the original. */
    s_roll_target_deg = 0.0f;
    s_pitch_target_deg = 0.0f;
    s_roll_steering = 0;
    s_pitch_steering = 0;
#endif
}

static uint16_t clamp_tail(float us) {
    if (us < SRV_MIN) return SRV_MIN;
    if (us > SRV_MAX) return SRV_MAX;
    return (uint16_t)us;
}

StabCorrection_t Stabilize_Compute(uint16_t roll_us, uint16_t pitch_us,
                                   uint16_t rudder_us, float motor_frac,
                                   float coll_frac, float dt_s) {
    StabCorrection_t out;
    out.pitch_axis = 0.0f;
    out.roll_axis = 0.0f;
    out.tail_us = SRV_CENTER;

    if (!STAB_ENABLE) return out; /* pass-through */

#if STAB_CYCLIC_RATE_MODE == 2
    /* STOCK ATTITUDE mode: the stick steers a TARGET angle, the loop holds the
     * machine on it, and the target survives stick release — the bank stays put
     * until the pilot moves it or drops the throttle. See the long note at
     * STAB_CYCLIC_RATE_MODE in the header for the decompiled original.
     *
     * Signs are inherited, not re-derived: positive stick = right roll = +roll
     * rate = +roll_deg (orientation.c, hand-tilt verified 2026-07-04), and the
     * same STAB_*_CORR_SIGN maps the output onto the pre-mix axes as in the
     * other two modes. */
    float roll_stick  = (float)roll_us  - SRV_CENTER;
    float pitch_stick = (float)pitch_us - SRV_CENTER;

    /* Deadband FIRST — see STAB_ATT_STICK_DEADBAND_US. Without it a resting
     * stick offset integrates the target into the rails on its own.
     *
     * LEASH, one-sided (rewritten 2026-07-16 after the first flights): the
     * STICK may not push the target further than LEASH_DEG beyond the actual
     * angle — but the actual angle NEVER moves the target. The first version
     * clamped target to [actual-LEASH, actual+LEASH] unconditionally, which
     * DRAGGED the target along whenever the airframe overshot it (full-stick
     * FF overshoots by design): overshoot to 60 deg rewrote a 35 deg target to
     * 45, releasing the stick then HELD that bank, and on the way back the
     * clamp kept dragging the target 25 deg ahead so the loop fought the
     * return the whole way down. Every maneuver/gust beyond LEASH rewrote the
     * target permanently — the pilot's "pulls somewhere it shouldn't". A
     * disturbance now leaves the target alone and the loop brings the machine
     * BACK, which is the whole point of attitude hold. */
    float roll_slew_dps  = deadband_f(roll_stick,  STAB_ATT_STICK_DEADBAND_US)
                           * STAB_ATT_STICK_TO_DPS;
    float pitch_slew_dps = deadband_f(pitch_stick, STAB_ATT_STICK_DEADBAND_US)
                           * STAB_ATT_STICK_TO_DPS;
    {
        float r = g_orientation.roll_deg, p = g_orientation.pitch_deg;
        float lim, step, prev;

        /* CAPTURE-ON-RELEASE (2026-07-17, first cascade flight): while the
         * stick is deflected, main.c's raw-stick feedforward rotates the
         * airframe FASTER than the target slews, so the actual angle runs
         * ahead of the target. Holding the OLD target on release made the
         * loop visibly spring the machine BACK to it — the pilot's "angle is
         * not held after release". Stock muscle memory says release = the
         * bank stays WHERE THE MACHINE IS. So on the deflected->released
         * edge each axis captures the ACTUAL angle as its target; while
         * released the captured value holds (that is the gust-rejection
         * memory), while deflected the target integrates as before, bounded
         * by the one-sided leash. */
        if (roll_slew_dps != 0.0f) {
            s_roll_steering = 1;
            step = roll_slew_dps * dt_s;
            prev = s_roll_target_deg;
            s_roll_target_deg += step;
            lim = r + STAB_ATT_LEASH_DEG;
            if (step > 0.0f && s_roll_target_deg > lim)
                s_roll_target_deg = (prev > lim) ? prev : lim;
            lim = r - STAB_ATT_LEASH_DEG;
            if (step < 0.0f && s_roll_target_deg < lim)
                s_roll_target_deg = (prev < lim) ? prev : lim;
        } else if (s_roll_steering) {
            s_roll_steering = 0;
            s_roll_target_deg = r;   /* the bank stays where the pilot left it */
        }

        if (pitch_slew_dps != 0.0f) {
            s_pitch_steering = 1;
            step = pitch_slew_dps * dt_s;
            prev = s_pitch_target_deg;
            s_pitch_target_deg += step;
            lim = p + STAB_ATT_LEASH_DEG;
            if (step > 0.0f && s_pitch_target_deg > lim)
                s_pitch_target_deg = (prev > lim) ? prev : lim;
            lim = p - STAB_ATT_LEASH_DEG;
            if (step < 0.0f && s_pitch_target_deg < lim)
                s_pitch_target_deg = (prev < lim) ? prev : lim;
        } else if (s_pitch_steering) {
            s_pitch_steering = 0;
            s_pitch_target_deg = p;
        }

        /* Absolute clamp last — the hard ceiling regardless of anything. */
        if (s_roll_target_deg >  STAB_ATT_MAX_DEG) s_roll_target_deg =  STAB_ATT_MAX_DEG;
        if (s_roll_target_deg < -STAB_ATT_MAX_DEG) s_roll_target_deg = -STAB_ATT_MAX_DEG;
        if (s_pitch_target_deg >  STAB_ATT_MAX_DEG) s_pitch_target_deg =  STAB_ATT_MAX_DEG;
        if (s_pitch_target_deg < -STAB_ATT_MAX_DEG) s_pitch_target_deg = -STAB_ATT_MAX_DEG;
    }

    /* OUTER loop of the cascade (see STAB_ATT_ANGLE_P): the angle error
     * commands a body RATE; the inner loop — mode 1's flight-proven rate PID
     * below — is the only thing that talks to the swash. The stick's commanded
     * target slew is fed forward into the rate setpoint (ArduPilot feeds its
     * shaped target rate the same way), so during deflection the inner loop is
     * asked FOR the rotation the stick commands instead of being asked to
     * fight the feedforward that produces it. */
    /* ATTITUDE TRIM added to the target (see STAB_ATT_*_TRIM_DEG): the hover
     * attitude that makes the machine hold POSITION is not level — the tail
     * pushes sideways and the disk must lean to answer it. In attitude mode
     * this belongs on the TARGET, not on the swash: a constant cyclic bias
     * (which is all a TX trim can produce) is just a disturbance that this
     * loop cancels by holding the angle. */
    float roll_set  = STAB_ATT_ANGLE_P * ((s_roll_target_deg + STAB_ATT_ROLL_TRIM_DEG)
                                          - g_orientation.roll_deg)
                      + roll_slew_dps;
    float pitch_set = STAB_ATT_ANGLE_P * ((s_pitch_target_deg + STAB_ATT_PITCH_TRIM_DEG)
                                          - g_orientation.pitch_deg)
                      + pitch_slew_dps;
    if (roll_set  >  STAB_ATT_MAX_RATE_DPS) roll_set  =  STAB_ATT_MAX_RATE_DPS;
    if (roll_set  < -STAB_ATT_MAX_RATE_DPS) roll_set  = -STAB_ATT_MAX_RATE_DPS;
    if (pitch_set >  STAB_ATT_MAX_RATE_DPS) pitch_set =  STAB_ATT_MAX_RATE_DPS;
    if (pitch_set < -STAB_ATT_MAX_RATE_DPS) pitch_set = -STAB_ATT_MAX_RATE_DPS;

    /* Airborne gate on the INNER integral — see STAB_ATT_AIRBORNE_LO. On the
     * ground the cascade still commands a rate the airframe cannot perform
     * (plant gain is zero there), so without this the RATE integral winds up
     * instead of the old angle one — same disease, one loop lower. Collective,
     * not spool: this heli's head speed is the same on the ground as in a
     * hover, only the blade pitch knows the difference. */
    float ground_gate = transition_f(coll_frac, STAB_ATT_AIRBORNE_LO,
                                     STAB_ATT_AIRBORNE_HI, 0.0f, 1.0f);
#elif STAB_CYCLIC_RATE_MODE == 1
    /* RATE mode (flybarless "normal"): the stick commands a body RATE, the gyro
     * rate is the feedback. main.c still adds the raw stick as feedforward, so
     * this loop mostly damps rotation the stick didn't ask for — release the
     * stick and it drives the residual rate to zero, holding the bank (no
     * self-level). The PID output is a microsecond swash delta on the same
     * pre-mix axes; *_CORR_SIGN is the bench-flippable sign. */
    float roll_stick  = (float)roll_us  - SRV_CENTER;
    float pitch_stick = (float)pitch_us - SRV_CENTER;
    float roll_set  = roll_stick  * STAB_CYCLIC_STICK_TO_DPS;
    float pitch_set = pitch_stick * STAB_CYCLIC_STICK_TO_DPS;
    float ground_gate = 1.0f;  /* zero stick = zero setpoint on the ground:
                                * rate mode never had the windup, keep as-was */
#endif

#if STAB_CYCLIC_RATE_MODE
    /* ---- INNER rate loop, shared by modes 1 (stick-fed) and 2 (angle-fed) */
    /* Feedback through the input LPF (see STAB_CYCLIC_IN_LPF_HZ). */
    s_roll_rate_lpf  = lpf_step(s_roll_rate_lpf, g_orientation.roll_rate_dps,
                                dt_s, STAB_CYCLIC_IN_LPF_HZ);
    s_pitch_rate_lpf = lpf_step(s_pitch_rate_lpf, g_orientation.pitch_rate_dps,
                                dt_s, STAB_CYCLIC_IN_LPF_HZ);
#if STAB_CYCLIC_NOTCH_ENABLE
    /* ...then through the 7.5Hz notch (see STAB_NOTCH_* / flights 6-7):
     * the airframe mode's band is removed from what the PID may react to. */
    float roll_rate_fb  = notch_step(s_roll_notch, s_roll_rate_lpf);
    float pitch_rate_fb = notch_step(s_pitch_notch, s_pitch_rate_lpf);
#else
    float roll_rate_fb  = s_roll_rate_lpf;
    float pitch_rate_fb = s_pitch_rate_lpf;
#endif

    /* Setpoint LPF -> (a) iterm relax factor from the HPF residue (RF:
     * relax = 1-|setpoint_hpf|/level, freezes I accumulation during fast stick
     * moves so a STRONG KI doesn't collect garbage mid-maneuver), and (b) the
     * B-term = Kb x filtered d(setpoint)/dt (crisper maneuver entry). */
    s_roll_set_lpf  = lpf_step(s_roll_set_lpf, roll_set, dt_s, STAB_CYCLIC_SET_LPF_HZ);
    s_pitch_set_lpf = lpf_step(s_pitch_set_lpf, pitch_set, dt_s, STAB_CYCLIC_SET_LPF_HZ);
    float roll_relax = 1.0f, pitch_relax = 1.0f;
    {
        float h = roll_set - s_roll_set_lpf;
        if (h < 0.0f) h = -h;
        roll_relax = 1.0f - h / STAB_CYCLIC_RELAX_DPS;
        if (roll_relax < 0.0f) roll_relax = 0.0f;
        h = pitch_set - s_pitch_set_lpf;
        if (h < 0.0f) h = -h;
        pitch_relax = 1.0f - h / STAB_CYCLIC_RELAX_DPS;
        if (pitch_relax < 0.0f) pitch_relax = 0.0f;
    }
    float roll_b = 0.0f, pitch_b = 0.0f;
    if (dt_s > 0.0f) {
        roll_b  = STAB_CYCLIC_B_GAIN * (s_roll_set_lpf - s_roll_set_prev) / dt_s;
        pitch_b = STAB_CYCLIC_B_GAIN * (s_pitch_set_lpf - s_pitch_set_prev) / dt_s;
    }
    s_roll_set_prev = s_roll_set_lpf;
    s_pitch_set_prev = s_pitch_set_lpf;

#if STAB_CYCLIC_RATE_MODE == 1
    /* RF error decay (see STAB_CYCLIC_DECAY_TAU_S): the stored integral bleeds
     * toward zero so it carries live correction, not history. MODE 1 ONLY
     * (2026-07-17): in the cascade a held bank has zero angle error, zero rate
     * setpoint, zero P — the standing cyclic that fights the pendulum lives
     * ENTIRELY in this integral. Decaying it produced a sag-and-catch limit
     * cycle at the tau timescale: bank droops, angle error grows, the outer
     * loop pushes it back, repeat — the pilot's "delayed compensation /
     * inertia" report from the first cascade flight. In rate mode the pilot IS
     * the outer loop, so the decay there stays. */
    if (dt_s > 0.0f) {
        float k = dt_s / STAB_CYCLIC_DECAY_TAU_S;
        if (k > 1.0f) k = 1.0f;
        s_roll_pid.integral  -= s_roll_pid.integral * k;
        s_pitch_pid.integral -= s_pitch_pid.integral * k;
    }
#endif

    float pitch_corr = PID_UpdateRelax(&s_pitch_pid, pitch_set, pitch_rate_fb,
                                       dt_s, pitch_relax * ground_gate) + pitch_b
                     - STAB_CYCLIC_D_GAIN * dterm_step(s_pitch_d, pitch_rate_fb, dt_s);
    float roll_corr  = PID_UpdateRelax(&s_roll_pid, roll_set, roll_rate_fb,
                                       dt_s, roll_relax * ground_gate) + roll_b
                     - STAB_CYCLIC_D_GAIN * dterm_step(s_roll_d, roll_rate_fb, dt_s);
#if STAB_CYCLIC_RATE_MODE == 1
    /* Stock's adaptive softening: full loop authority at stick centre, fading
     * as deflection grows (see STAB_CYCLIC_SOFTEN). Mode 1 only — in the
     * cascade the stick steers the target, not this loop, so softening here
     * would just weaken the return to a target the pilot deliberately moved. */
    {
        float m = roll_stick < 0.0f ? -roll_stick : roll_stick;
        float mp = pitch_stick < 0.0f ? -pitch_stick : pitch_stick;
        if (mp > m) m = mp;
        float atten = 1.0f / (1.0f + STAB_CYCLIC_SOFTEN * (m / 330.0f));
        pitch_corr *= atten;
        roll_corr  *= atten;
    }
#endif
#else
    (void)roll_us; (void)pitch_us;
    /* Self-level assist: setpoint 0 deg = wings/nose level. The PID output is
     * a microsecond-space delta on the (pre-mix) pitch/roll stick axes; the
     * *_CORR_SIGN macro is the bench-flippable sign (see stabilize.h). */
    float pitch_corr = PID_Update(&s_pitch_pid, 0.0f, g_orientation.pitch_deg, dt_s);
    float roll_corr  = PID_Update(&s_roll_pid, 0.0f, g_orientation.roll_deg, dt_s);
#endif
    /* Low-pass the finished swash correction to strip the rotor-vibration band
     * that the self-level D-term otherwise pumps into the servos (see
     * STAB_OUTPUT_LPF_HZ). Applied after the sign so the filtered value is what
     * main.c injects pre-mix. */
    s_pitch_axis_lpf = lpf_step(s_pitch_axis_lpf,
                                STAB_PITCH_CORR_SIGN * pitch_corr,
                                dt_s, STAB_OUTPUT_LPF_HZ);
    s_roll_axis_lpf  = lpf_step(s_roll_axis_lpf,
                                STAB_ROLL_CORR_SIGN * roll_corr,
                                dt_s, STAB_OUTPUT_LPF_HZ);
    out.pitch_axis = s_pitch_axis_lpf;
    out.roll_axis  = s_roll_axis_lpf;

    /* Heading-hold tail gyro with a physics-split anti-torque feedforward
     * (see STAB_TAIL_FF_* in the header): blade profile drag + collective
     * loading x head speed + rotor spool-up. The PID corrects around that, and
     * thrust-REDUCING corrections keep the stock x4.2 boost (one-direction
     * tail sheds thrust poorly). Gyro sign verified under power 2026-07-12. */
    float desired_yaw_rate = ((float)rudder_us - SRV_CENTER) * STAB_YAW_STICK_TO_DPS;
    float yaw_err = desired_yaw_rate - g_orientation.yaw_rate_dps;
    /* Yaw iterm relax (see STAB_YAW_RELAX_DPS): freeze I while the setpoint
     * moves fast so stick work / release transients can't load it. */
    s_yaw_set_lpf = lpf_step(s_yaw_set_lpf, desired_yaw_rate, dt_s,
                             STAB_YAW_SET_LPF_HZ);
    float yaw_relax;
    {
        float h = desired_yaw_rate - s_yaw_set_lpf;
        if (h < 0.0f) h = -h;
        yaw_relax = 1.0f - h / STAB_YAW_RELAX_DPS;
        if (yaw_relax < 0.0f) yaw_relax = 0.0f;
    }
    float yaw_corr = PID_UpdateRelax(&s_yaw_pid, desired_yaw_rate,
                                     g_orientation.yaw_rate_dps, dt_s, yaw_relax);

    float coll_load = coll_frac - 0.5f;              /* |blade loading|, 0..1 */
    if (coll_load < 0.0f) coll_load = -coll_load;
    coll_load *= 2.0f;
    if (coll_load > 1.0f) coll_load = 1.0f;

    /* Cyclic-to-yaw precomp (see STAB_TAIL_FF_CYC_US): |cyclic| deflection
     * 0..1 from whichever axis is larger, like the RF mainDeflection sum. */
    float cyc_load;
    {
        float r = (float)roll_us - SRV_CENTER, p = (float)pitch_us - SRV_CENTER;
        if (r < 0.0f) r = -r;
        if (p < 0.0f) p = -p;
        cyc_load = (r > p ? r : p) / 330.0f;
        if (cyc_load > 1.0f) cyc_load = 1.0f;
    }

    if (!s_prev_motor_valid) {                        /* no fake spool on engage */
        s_prev_motor_frac = motor_frac;
        s_prev_motor_valid = 1;
    }
    float spool = 0.0f;
    if (dt_s > 0.0f) {
        float d = (motor_frac - s_prev_motor_frac) / dt_s;   /* frac per second */
        if (d > 0.0f) spool = d * STAB_TAIL_FF_SPOOL_US;     /* spin-up only */
        if (spool > STAB_TAIL_FF_SPOOL_MAX) spool = STAB_TAIL_FF_SPOOL_MAX;
    }
    s_prev_motor_frac = motor_frac;

    /* RUDDER FEEDFORWARD (see STAB_TAIL_FF_RUD_*): the pedal drives tail
     * thrust DIRECTLY, outside the PID and its output_limit, so pilot yaw
     * authority no longer has to squeeze through the correction clamp.
     *
     * SIGN IS DERIVED, NOT ASSUMED. The 2026-07-05 bench note ("+tail_us =
     * nose right") and STAB_YAW_CORR_SIGN = -1 contradict each other, and the
     * flying loop is the one that must be right. So instead of picking a
     * convention, mirror the loop's own initial answer to a pedal command:
     * at turn entry the machine is still (actual ~ 0), so the loop pushes
     * corr ~ CORR_SIGN * KP * desired_yaw_rate. Using CORR_SIGN * desired
     * here makes the feedforward push the SAME way the P term would, under
     * either convention. Get this wrong and the FF fights the pilot, so it is
     * deliberately tied to the sign that is already flight-proven. */
    float rud_ff = STAB_TAIL_FF_RUD_PER_DPS * desired_yaw_rate;
    if (rud_ff >  STAB_TAIL_FF_RUD_MAX_US) rud_ff =  STAB_TAIL_FF_RUD_MAX_US;
    if (rud_ff < -STAB_TAIL_FF_RUD_MAX_US) rud_ff = -STAB_TAIL_FF_RUD_MAX_US;
    rud_ff *= (float)STAB_YAW_CORR_SIGN;

    float tail_ff = (float)SRV_MIN
                  + STAB_TAIL_FF_BASE_US
                  + STAB_TAIL_FF_COLL_US * coll_load * motor_frac
                  + STAB_TAIL_FF_CYC_US * cyc_load * motor_frac
                  + spool
                  + rud_ff;

    float corr = STAB_YAW_CORR_SIGN * yaw_corr;
    /* RF-style directional stop gain: add (stopGain-1) x the P contribution,
     * blended by which way the error pushes TAIL THRUST (e_thrust > 0 = needs
     * more). Only P is boosted — the integral stays symmetric. */
    {
        float e_thrust = STAB_YAW_CORR_SIGN * yaw_err;
        float stop_gain = transition_f(e_thrust,
                                       -STAB_YAW_STOP_TRANS_DPS,
                                       STAB_YAW_STOP_TRANS_DPS,
                                       STAB_YAW_STOP_GAIN_DOWN,
                                       STAB_YAW_STOP_GAIN_UP);
        corr += STAB_YAW_CORR_SIGN * STAB_YAW_KP * yaw_err * (stop_gain - 1.0f);
    }
    if (corr < 0.0f) corr *= STAB_YAW_NEG_BOOST;   /* =1.0, retired (see .h) */
    float tail_raw = tail_ff + corr;               /* pre-clamp, for the probe */
    out.tail_us = clamp_tail(tail_raw);

    /* TAIL WORKING-POINT PROBE (2026-07-19, pilot: "осцилляции хвоста
     * постепенно появляются когда аккумулятор разряжен, на заряженном не
     * видно"). Observation only — nothing here feeds control. As the pack
     * sags the same thrust costs more command, so the tail's operating point
     * climbs and the motor has less room left to ACCELERATE; the dead time
     * that already sets our gain ceiling (~80-100ms) effectively grows and the
     * 4Hz ring stops being damped. These three separate the two candidate
     * mechanisms, which need different fixes:
     *   g_tail_ema_us  - slow mean of the commanded tail = the sag proxy.
     *                    Rises through a flight if the pack is drooping.
     *   g_tail_max_us  - highest command seen.
     *   g_tail_sat_ms  - milliseconds spent PINNED at the clamp (counts the
     *                    pre-clamp value exceeding the rail, so it also catches
     *                    "wanted more than 2000").
     * high EMA + sat time  -> COMMAND ceiling: the loop is asking past the rail
     *                         (gain scheduling on the working point is the fix)
     * high EMA + no sat    -> THRUST limited: full command, motor just cannot
     *                         deliver at that voltage (only lower gain helps)
     * NOT cleared by Stabilize_Reset — they must survive the landing to be
     * read over SWD, so they cover the whole power-up, not one flight. */
    {
        float a = dt_s / (dt_s + 2.0f);            /* ~2s time constant */
        g_tail_ema_us += a * (tail_raw - g_tail_ema_us);
        if (tail_raw > g_tail_max_us) g_tail_max_us = tail_raw;
        if (tail_raw >= (float)SRV_MAX) g_tail_sat_ms += (uint32_t)(dt_s * 1000.0f);
    }

#if STAB_BLACKBOX
    /* Time-weighted window accumulation (dt jitters with the IMU quiet-zone
     * waits, so weight by dt rather than counting samples). Noisy signals are
     * meaned; slow states (targets, integral) snapshot at window end. The
     * swash corrections logged are the FILTERED values main.c actually injects
     * (out.roll_axis/pitch_axis), before the stab_gain engage ramp. */
    s_bb_err_acc   += yaw_err * dt_s;
    s_bb_tail_acc  += (float)out.tail_us * dt_s;
    s_bb_roll_acc  += g_orientation.roll_deg  * dt_s;
    s_bb_pitch_acc += g_orientation.pitch_deg * dt_s;
    s_bb_rollc_acc += out.roll_axis  * dt_s;
    s_bb_pitchc_acc+= out.pitch_axis * dt_s;
    s_bb_dt_acc    += dt_s;
    if (s_bb_dt_acc >= STAB_BB_PERIOD_S) {
        StabBBRec_t *r = &g_bb_log[g_bb_head % STAB_BB_N];
        float inv = 1.0f / s_bb_dt_acc;
        float cf  = coll_frac;
        if (cf < 0.0f) cf = 0.0f;
        if (cf > 1.0f) cf = 1.0f;
        r->err_dps10    = (int16_t)(s_bb_err_acc  * inv * 10.0f);
        r->yaw_i_x100   = (int16_t)(s_yaw_pid.integral * 100.0f);
        r->tail_us      = (uint16_t)(s_bb_tail_acc * inv);
        r->roll_deg100  = (int16_t)(s_bb_roll_acc  * inv * 100.0f);
        r->pitch_deg100 = (int16_t)(s_bb_pitch_acc * inv * 100.0f);
        r->roll_corr10  = (int16_t)(s_bb_rollc_acc * inv * 10.0f);
        r->pitch_corr10 = (int16_t)(s_bb_pitchc_acc* inv * 10.0f);
        r->coll200      = (uint8_t)(cf * 200.0f);
#if STAB_CYCLIC_RATE_MODE == 2
        r->roll_tgt100  = (int16_t)(s_roll_target_deg  * 100.0f);
        r->pitch_tgt100 = (int16_t)(s_pitch_target_deg * 100.0f);
        r->steer        = (uint8_t)((s_roll_steering ? 1 : 0) |
                                    (s_pitch_steering ? 2 : 0));
#else
        r->roll_tgt100 = r->pitch_tgt100 = 0;
        r->steer = 0;
#endif
        g_bb_head++;
        s_bb_dt_acc = s_bb_err_acc = s_bb_tail_acc = 0.0f;
        s_bb_roll_acc = s_bb_pitch_acc = s_bb_rollc_acc = s_bb_pitchc_acc = 0.0f;
    }
#endif

    return out;
}
