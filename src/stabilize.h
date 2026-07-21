#ifndef STABILIZE_H
#define STABILIZE_H

#include <stdint.h>
#include "orientation.h"

/* Phase 4 stabilization: gyro/accel correction added on top of the pilot's
 * RC command. No ARM/DISARM or tilt lockout here — deliberately deferred per
 * instruction; this module only ever nudges what main.c is about to mix.
 *
 * PRE-MIX injection (changed 2026-07-05): the CCPM swash mix + per-servo
 * direction flags (DIR_REAR_R = -1) live in main.c and run on RAW TX stick
 * axes. So corrections are returned as deltas ON THOSE SAME virtual axes
 * (pitch/roll, in the microsecond units of `data[]-CENTER`), and main.c adds
 * them to the stick axes *before* the mix. The mix then routes and mirrors a
 * correction exactly like a stick nudge — no matrix inverse, and a reversed
 * servo mirrors the correction too (the old post-mix Stabilize_Apply added
 * straight to servo microseconds and got the reversed rear-right servo wrong,
 * which is why it was disconnected during the CCPM refactor).
 *
 * Yaw (tail): the tail output drives an ESC into a fixed-pitch tail rotor,
 * not a servo — classic RC-heli "heading-hold" gyro. Rudder stick sets a
 * desired yaw RATE, gyro Z is the feedback, the corrected tail ESC value is
 * returned directly (centred on SRV_CENTER, bidirectional about a hover
 * anti-torque base). Mirrors the original firmware's YAW_DRIFT_CORRECTION_K.
 *
 * Axis-sign status:
 *  - Sensor signs CONFIRMED 2026-07-04 on hardware: nose-up = +pitch_deg,
 *    right-side-down = +roll_deg, nose-right = +yaw_rate_dps.
 *  - Tail-rotor direction CONFIRMED 2026-07-05 by the user (tail motor
 *    spinning, props off): raising tail_us yaws the nose right (+yaw_rate),
 *    so the heading-hold loop below is negative feedback (was the one
 *    positive-feedback risk; now cleared).
 *  - Swash CORRECTION direction (does the pitch/roll nudge counteract or
 *    amplify a tilt) is NOT yet bench-checked — it depends on the mechanical
 *    stick-axis-to-heli-response mapping, which no hand-tilt test settled.
 *    Verify on the bench (rotor OFF, STAB_ENABLE=1, SWASH_RECENTER_ON_IDLE=0,
 *    motor NOT idle): tilt the airframe and confirm the swash moves the way
 *    that would push it back level. If an axis amplifies instead, flip that
 *    axis's STAB_*_CORR_SIGN below — a single-macro fix, no other reasoning. */
#define STAB_ENABLE   1   /* Swash-correction sign test PASSED 2026-07-12 (bench
                           * tilt, rotor off, STAB_BENCH_TILT build): both axes
                           * push the swash back toward level, signs verified
                           * +1/+1. Radio starvation fixed the same day (IMU
                           * pipeline + slot quiet-zone, memory
                           * heli-stab-radio-budget) — 0.3% slot loss with the
                           * full stab math running. */

/* Sign of each swash correction relative to its stick axis. +1 = correction
 * adds to the axis the same way a positive stick would; flip to -1 if the
 * bench tilt test shows that axis amplifying tilt instead of counteracting.
 * BENCH-VERIFIED 2026-07-12 (tilt test, rotor off): both axes counteract
 * with +1 — swash holds level against frame tilt. */
#define STAB_PITCH_CORR_SIGN  (+1)
#define STAB_ROLL_CORR_SIGN   (+1)

/* Output low-pass on the swash correction (Hz). The self-level D-term is
 * essentially -Kd*gyro_rate (error = 0 - angle, angle ~= integral of gyro), so
 * it feeds the rotor-vibration band (~33-48Hz, only ~-7dB at the 20Hz gyro
 * DLPF) straight into the swash and dithers the servos (pilot 2026-07-13:
 * dither survived the DLPF tightening). A single-pole LPF on the finished
 * correction attenuates that band for EVERY term at once (D dominates, but P
 * ripple and dt-jitter artifacts ride along too) while a hover heli's <5Hz
 * attitude dynamics pass untouched. 10Hz adds ~-11dB at 40Hz on top of the
 * DLPF (~-18dB total). Raise toward 15-20Hz if the swash feels laggy, lower
 * toward 6-8Hz if dither persists. alpha is recomputed from the real dt each
 * tick, so the cutoff holds despite the 10-30ms IMU-read jitter. */
#define STAB_OUTPUT_LPF_HZ   10.0f

/* CYCLIC CONTROL MODE. All three keep main.c's raw-stick feedforward in the
 * mix; only the gyro/accel loop below differs:
 *   0 = SELF-LEVEL   PID(setpoint = 0 deg, measurement = ANGLE) — stick release
 *                    pulls back to horizon.
 *   1 = RATE         PID(setpoint = stick RATE, measurement = gyro RATE) — no
 *                    attitude memory at all; the airframe's pendulum is just a
 *                    disturbance the integral has to out-muscle.
 *   2 = STOCK ATTITUDE (2026-07-16, default) — see below.
 *
 * MODE 2 reproduces what the ORIGINAL firmware actually did, as decompiled:
 * it ran attitude-hold on a MOVING TARGET. A quaternion integrated the gyro
 * (Math_QuaternionMultiply), the stick STEERED the target attitude
 * (Control_AxisCorrection + Math_QuaternionRotateVector), the loop held the
 * machine on that target, and Math_QuaternionIdentity() snapped the target back
 * to horizon at throttle minimum. Read as behavior that is exactly the pilot's
 * muscle memory: bank accumulates while the stick is held, STAYS where you left
 * it for as long as you like, and drops back to level when you lower the
 * throttle. Modes 0 and 1 cannot produce that — 0 always pulls to horizon, and
 * 1 has no angle in the loop to remember.
 *
 * We do it on Euler angles from orientation.c rather than quaternions: near
 * hover the small-angle approximation already backing that filter is enough,
 * and this MCU has no FPU. The target reset falls out for free — main.c calls
 * Stabilize_Reset() exactly when throttle_is_idle(), which zeroes the target
 * below. That IS Math_QuaternionIdentity() at throttle minimum.
 *
 * Modes 0/1 stay for the planned idle-up split (Rotorflight-style rate flying
 * with integral decay on the aerobatic switch, stock attitude in normal). */
#define STAB_CYCLIC_RATE_MODE  2

/* ---- MODE 2 (stock attitude) ---------------------------------------------
 * Stick -> TARGET SLEW RATE, deg/s per us of stick deflection.
 * LOWERED 1.5 -> 0.3 (2026-07-16, first mode-2 flights). The 1.5 came from a
 * units error: "match the RATE mode's STICK_TO_DPS" compared a BODY-RATE
 * command against a TARGET-ANGLE slew — incommensurable things. At 1.5 a full
 * stick slewed the target at ~495 deg/s, pinning it on the +-35 clamp in 70ms:
 * the stick was a 3-position switch, not a proportional control, and every
 * input instantly demanded the full P authority. 0.3 = ~93 deg/s effective
 * (deadband leaves 310us of throw), reaching the clamp in ~0.4s of held
 * full stick — bank accumulates visibly, like the stock behavior the pilot
 * remembers. Raise toward 0.5 if steering feels too slow, drop toward 0.2 if
 * still too eager. */
/* 0.3 -> 0.4 (2026-07-19, pilot "немного увеличим чувствительность стиков").
 * Follows this block's own tuning note ("raise toward 0.5 if steering feels
 * too slow") and the earlier "чуть туповато" report. Chosen over raising
 * main.c's CYCLIC_GAIN (0.45) deliberately: that one is the RAW-stick swash
 * feedforward, and pushing it alone would bank the machine faster than the
 * target slews, leaving the loop to pull back against the pilot's own stick.
 * This knob is pure COMMAND PATH — no loop gain changes — so it cannot alter
 * stability, which is what keeps it separable from the KD change below when
 * reading the next flight. ~124 deg/s of target slew at full stick. */
#define STAB_ATT_STICK_TO_DPS  0.4f

/* ATTITUDE TRIM, degrees added to the target (2026-07-19, pilot zeroed all TX
 * trims and asked for the cyclic to be trimmed instead).
 * WHY IT LIVES HERE AND NOT ON THE TX: in attitude mode the stick's raw
 * feedforward reaches the swash, but this loop holds the ANGLE — so a constant
 * swash bias, which is all a TX trim can make, is seen as a disturbance and
 * cancelled by the loop. A TX cyclic trim below the 20us target deadband does
 * nothing lasting; above it, it walks the target integrator to its rails (the
 * bug the deadband was added to stop on 2026-07-16). Neither is a trim. The
 * quantity that actually needs shifting is the ATTITUDE the machine holds:
 * a hovering heli must lean slightly to answer the tail rotor's side thrust,
 * so "hold position" is a small bank, not level.
 * +roll = right side down, +pitch = nose up.
 * STILL ZERO — deliberately NOT guessed. The one flight that showed a hover
 * bank (flight 4, hold windows ~+9deg) came from the stuck-estimate era and
 * cannot be trusted. MEASURE IT: fly with STAB_BLACKBOX on, hold the machine
 * stationary over a spot, then read the hold-window (steer=0) roll_tgt/
 * pitch_tgt — the pilot steers the target to whatever stops the drift, so that
 * value IS the trim. Bake it in here and the machine holds it by itself. */
#define STAB_ATT_ROLL_TRIM_DEG   0.0f
#define STAB_ATT_PITCH_TRIM_DEG  0.0f

/* Stick DEADBAND around centre, us, applied to the TARGET INTEGRATOR only.
 * MANDATORY, not a comfort feature: an integrator has infinite DC gain, so any
 * resting stick offset — trim, pot slop, a stick that just doesn't sit at
 * 1520 — is integrated forever and walks the target to its rails with the
 * pilot's hands off the sticks. Measured on this airframe 2026-07-16: roll rest
 * = 1528 (+8us), pitch rest = 1508 (-12us), which drove the target at 12 and 18
 * deg/s until pitch pinned at the -35 clamp and roll at the leash. That was the
 * "swash drifts off on its own once stab engages" report.
 *
 * 20us ~= 6% of the 330us throw. Sized over the measured offsets with margin,
 * and the offset is SUBTRACTED past the edge so response stays continuous
 * (no jump from nothing to full slew). RATE mode never needed this: an offset
 * there is just a small constant rate command, and nothing accumulates.
 *
 * The deadband is a backstop, not the cure — trim the TX so the sticks actually
 * read 1520 (watch g_sfhss.data[0]/[1] over SWD). Anything left inside the band
 * simply never moves the target. */
#define STAB_ATT_STICK_DEADBAND_US  20.0f

/* Target clamp, degrees. Without it, holding the stick winds the target past
 * vertical and the loop dutifully tries to fly there. Also the reason the
 * small-angle orientation filter stays inside its valid band (it degrades
 * above ~30 deg). */
#define STAB_ATT_MAX_DEG       35.0f

/* Target LEASH, degrees — ONE-SIDED (see the block comment in stabilize.c):
 * the stick cannot wind the target further than this beyond the actual angle
 * (machine can't follow -> stop demanding more), but the actual angle never
 * moves the target. The first, unconditional version dragged the target along
 * on every overshoot/gust past the leash and made the loop HOLD attitudes the
 * pilot never commanded. */
#define STAB_ATT_LEASH_DEG     25.0f

/* Mode-2 CASCADE (rewritten 2026-07-17; the direct version lived one day,
 * 2026-07-16 — it ran an angle PID straight into the swash and paid for it twice: a 6-8Hz
 * limit cycle (its D term contributed KD*A*w ~= 35A at 7Hz vs P's 6A — it drove
 * the cycle, and with KD=0 the loop had no damping at all), and an antiphase
 * "swash one way, machine pulled the other" once loop lag passed 90 deg. This
 * is exactly why neither ArduPilot Stabilize nor Rotorflight Angle mode drives
 * servos from an angle PID. Both run a CASCADE: angle error -> commanded body
 * rate -> the rate loop talks to the actuators. Ours now does the same, and
 * the inner loop is mode 1's rate PID VERBATIM — the one loop on this airframe
 * with a flight-proven sign, damping, relax and decay.
 *
 * ANGLE_P: commanded dps per degree of angle error (units 1/s). 4.5 is
 * ArduPilot's ANG_RLL_P/ANG_PIT_P default and a sane starting point: a 10 deg
 * error asks for 45 dps. Raise for a snappier return to target, lower if the
 * return overshoots. The rate clamp caps the ask on big errors — 180 dps is
 * well inside what full stick commanded in mode 1 (495), so the inner loop is
 * never asked for more than it has flown. */
#define STAB_ATT_ANGLE_P        4.5f
#define STAB_ATT_MAX_RATE_DPS   180.0f

/* AIRBORNE GATE on the attitude integral, in coll_frac units (collective_norm:
 * 0..1, 0.5 = flat blade pitch). Below LO the integral does not accumulate at
 * all; it fades in to full accumulation by HI. P and D are untouched.
 *
 * WHY THIS EXISTS (measured 2026-07-16, the "the whole cyclic has shifted and
 * pulling back sends it right" report). Stab engages at 5% THROTTLE, long
 * before the machine leaves the ground. Sitting there the loop sees a non-zero
 * angle (this airframe rests at ~+7.8 deg pitch), commands cyclic to fix it,
 * and nothing happens — on the ground the plant gain is zero. An integrator
 * against a zero-gain plant always winds to its rail, and both did: roll
 * integral pinned at +80.00, pitch at -80.00, i.e. KI(0.5) x 80 = 40us of
 * standing swash on EACH axis, diagonally. That is the "shift"; adding pitch
 * stick to a standing 40us of roll moves the disk on the diagonal.
 *
 * WHY THE GATE IS COLLECTIVE AND NOT SPOOL. The obvious "freeze until the rotor
 * is up" is WRONG for this aircraft: throttle_curve() reaches full by
 * THR_CURVE_KNEE (0.28) of stick and is FLAT after, and collective is its own
 * channel (COLLECTIVE_FROM_CH5). Head speed is therefore identical sitting on
 * the ground and hovering — motor_frac cannot tell them apart. Collective can:
 * measured on the ground with the rotor spinning, coll_frac = 0.362 (blades at
 * negative pitch). Nothing lifts this machine until the pitch does.
 *
 * Rate mode never needed this — on the ground the rates are zero, so there is
 * no error to accumulate. The integral of an ANGLE is what introduced it. */
#define STAB_ATT_AIRBORNE_LO  0.52f  /* just above flat pitch */
#define STAB_ATT_AIRBORNE_HI  0.62f

/* RATE-mode cyclic: stick deflection (us from centre) -> commanded body rate
 * (deg/s). This is the STICK AUTHORITY / how fast bank accumulates while you
 * hold the stick. LOWERED 2.5->1.0 (2026-07-14): first hover flew too sharp;
 * 2.5 = 825dps at full stick is a 3D-aerobatic rate, hover wants ~150-330dps.
 * 1.0 = ~330dps full stick. Raise if it feels sluggish once acclimatised. */
#define STAB_CYCLIC_STICK_TO_DPS  1.5f

/* RATE-mode cyclic PID (deg/s error -> us swash correction). The raw-stick
 * feedforward in main.c gives the primary disk tilt; this loop mainly DAMPS
 * residual rotation (stops the roll when the stick is released) and rejects
 * gusts. KP is the DAMPING firmness — the "servos fidget too eagerly" knob
 * (pilot 2026-07-13); lower it for calmer servos, raise until the head locks
 * without wobble. KD amplifies raw-gyro noise (rate feedback is unfiltered
 * angular rate) so it's OFF by default here. NOTE: rate feedback is inherently
 * more servo-active than attitude mode (raw rate vs smoothed angle); the 10Hz
 * output LPF and low KP carry the calm. STAB_*_CORR_SIGN re-used from the
 * ATTITUDE verify; behavior confirms it damps (not amplifies). Final gains are
 * a FLIGHT/hover task — a stationary bench can't close a rate loop (no real
 * rotation for the gyro to track), so the integral just winds up there. */
/* INTEGRAL-DOMINANT tune (2026-07-13): stock's bench feel — gentle KP (calm
 * servos), strong KI (visible roll accumulation that holds). A big KP would
 * mask the accumulation on a stationary bench (it answers the whole rate error
 * instantly, since the frame never actually rotates to null it). KI is the
 * "compensation authority + accumulation" knob the pilot asked to raise. */
/* KI DOUBLED 0.9 -> 1.8 (2026-07-16, pilot: "крен должен удерживаться — это
 * rate/acro"): in true rate mode the airframe's pendulum restoring moment is a
 * DISTURBANCE the loop must reject — the integral has to accumulate enough
 * cyclic to freeze the commanded bank. At 0.9 the pendulum out-ran the
 * integral and the heli self-levelled. Paired with ITERM RELAX below (a strong
 * integral without relax collects garbage during fast maneuvers -> bounce). */
/* KP 0.35 -> 0.25 (2026-07-19, flight 6 = first RATE_LOG spectrum). The
 * "small fast oscillation" the pilot felt as ~4Hz measured as a steady
 * 7.4Hz limit cycle on ALL axes, ROLL loudest (25-33dps envelope, pitch ~18,
 * yaw ~15; ~0.6deg of angle). One frequency on three axes = one physical
 * mode, and it is THE known 6-8Hz band (tail pulse at stop gain 1.8; the
 * first mode-2 cyclic wobble) — per the standing note, the cure is to not
 * feed gain in that band. At 7.4Hz the outer ANGLE_P contributes ~nothing
 * (4.5 x 0.6deg = 2.7dps vs 30dps swings), KD is already 0, KI's gain at
 * 7.4Hz is 1.2/46 = negligible — the only cyclic-loop gain left in-band is
 * this KP. DISCRIMINATING experiment, one variable: amplitude drops with the
 * gain => the loop was feeding the mode (keep trimming / notch later);
 * unchanged => the loop is a spectator, go look at mechanics (rotor balance,
 * head damping) and the tail stop gain. Compare via the same rate-log
 * spectrum, rate_fft.py, roll line @ 7.4Hz: was 16.6 amplitude.
 * OUTCOME (flight 7): 7.4Hz line GONE — the loop WAS the feeder. But 0.25
 * can't damp the pendulum (broad 1.5-3Hz wallow, integral-dominant misery,
 * see the notch note above STAB_CYCLIC_NOTCH_*). KP restored to 0.35 WITH
 * the notch taking the 7.5Hz band out of the loop. */
#define STAB_CYCLIC_RATE_KP  0.35f
#define STAB_CYCLIC_RATE_KI  1.2f  /* step 2 (2026-07-16): moderate raise, paired
                                    * with relax + decay below. The bare 1.8 of the
                                    * failed experiment held stale garbage. */
/* KD 0.0 -> 0.02 (2026-07-19, pilot "немного задемпфируем осцилляции по
 * ролу"). D is the damping term proper — it fights oscillation without
 * softening command authority, which is why it beats lowering ANGLE_P here
 * (that would undo the sensitivity raise above).
 *
 * WHY IT IS SAFE NOW, having been zeroed before: the note above says "KD
 * amplifies raw-gyro noise (rate feedback is unfiltered angular rate)" — that
 * premise is GONE. The feedback now passes STAB_CYCLIC_IN_LPF_HZ (6Hz) and
 * then the 7.5Hz notch before the PID, so the exact mode D used to pump is
 * removed before D can see it, and above ~15Hz the LPF has already cut what D
 * would multiply by omega. (The infamous KD 0.8 kill was the OLD direct
 * attitude-PID, a different architecture and 40x this gain.)
 * Derivative kick is bounded too: PID_UpdateRelax differentiates the ERROR,
 * but in the cascade the setpoint comes from a rate-limited target integrator,
 * so roll_set RAMPS (worst case ANGLE_P x target slew ~ 560 dps/s -> ~11us)
 * instead of stepping.
 * Sizing: at a 2Hz wobble of +-30dps, D gives ~7.5us against P's ~10.5us —
 * real damping, not dominant.
 * HONEST CAVEAT: the roll wobble frequency has NOT been measured yet (the
 * flight-8 log was analysed for yaw only and the reflash erased it). This is
 * a reasoned first step, not a measured fix. NEXT FLIGHT: run the FULL
 * rate_fft (all axes, not just yaw) on a calm hover to finally pin the roll
 * line. Roll back to 0 if servos dither or a new high-frequency buzz appears;
 * raise toward 0.04 if the wobble merely shrinks. Applies to pitch too (both
 * cyclic PIDs share these gains) — harmless, likely helpful. */
#define STAB_CYCLIC_RATE_KD  0.0f   /* PID-INTERNAL D stays OFF — see the
   * filtered external D below. The internal one differentiates the ERROR and
   * is unfiltered, which is exactly what rang at 16.75Hz on flight 10. */

/* FILTERED D (2026-07-19, flight 10 measured the whole story).
 * Flight 10 ran the plain PID D at 0.02 and the rate log settled it:
 *   roll @2.5Hz (the wobble we targeted): 6.4 -> 2.5 dps — D WORKED,
 *   roll @16.75Hz: 0.6 -> 37.6 dps — a brand-new D-driven limit cycle.
 * Compared across every stored log, so it is not the rotor aliasing into the
 * 50Hz sample rate (33.25Hz would fold to 16.75Hz, but then flights 6-8 would
 * show it too; they read 0.6-1.2). The arithmetic agrees: a raw D's gain
 * rises with frequency, so at 16.75Hz D/P = KD*omega/KP = 0.02*105/0.35 = 6.
 * D was six times P up there.
 * So keep D where it helps and take its top off — the standard D-term
 * low-pass every modern FC ships. TWO poles, because a single pole rolls off
 * at exactly the rate D rises (-20dB/dec vs +20dB/dec) and would leave D FLAT
 * above the cutoff instead of falling. At 6Hz, two poles give:
 *   2.5Hz  -> 0.85 of D kept  => D ~ 0.76 x P, real damping survives
 *   16.75Hz-> 0.11 of D kept  => D ~ 0.68 x P, no longer dominant, no ring
 * Also moved onto the MEASUREMENT derivative rather than the error: a wobble
 * looks the same either way (setpoint ~ constant), but stick/setpoint moves no
 * longer kick D at all — free, and it pairs well with the raised stick rate.
 * TUNING: wobble returns -> raise gain toward 0.03; any new HF buzz or servo
 * dither -> lower the LPF toward 4Hz FIRST (that is the frequency-selective
 * knob), gain second. Verify on the rate log: @2.5Hz should stay ~2-3, and
 * @16.75Hz must go back under ~2. */
#define STAB_CYCLIC_D_GAIN    0.02f  /* us per (dps/s), on measurement */
/* 6.0 -> 4.0 (2026-07-19, flight 11). The two-pole filter at 6Hz did kill the
 * 16.75Hz line (37.6 -> 1.0 dps) and kept the damping (roll @2.5Hz stayed
 * 2.7), but the ring MOVED rather than died: 13.88Hz went 2.0 -> 22.5 dps.
 * Invisible to the pilot (22 dps at 13.9Hz is only ~0.25 deg of travel) but it
 * is servo dither, and it is my regression to clean up.
 * The design error is exact and worth remembering: for a low-passed D term,
 * D_eff/P peaks EXACTLY AT THE CUTOFF (d/df of f/(1+(f/fc)^2) is zero at
 * f=fc). At fc=6 that peak is 1.08 — still above 1, so D could dominate
 * somewhere, and the loop simply picked whichever frequency closed the phase.
 * Choosing fc is therefore choosing the peak, not just the rolloff:
 *   fc=6 -> peak 1.08   fc=4 -> peak 0.72   fc=3 -> peak 0.54
 * 4Hz puts the peak safely under 1 everywhere while 2.5Hz damping only drops
 * from 0.76 to 0.65 of P. Follows this block's own rule: LPF first, gain
 * second. If a ring still appears, go to 3Hz before touching the gain. */
#define STAB_CYCLIC_D_LPF_HZ  4.0f   /* two cascaded poles at this cutoff */

/* Rotorflight B-term (boost): Kb x d(setpoint)/dt, filtered — sharpens the
 * ENTRY into a maneuver without raising steady-state sensitivity (the
 * "чуть-чуть острее" knob orthogonal to rates). us per dps/s; a full-stick
 * step (~400dps over ~50ms = 8000dps/s) gives ~40us kick at 0.005. */
#define STAB_CYCLIC_B_GAIN     0.0f   /* OFF until step 3 */

/* Rotorflight iterm relax: attenuate I ACCUMULATION while the setpoint moves
 * fast — relax = max(0, 1 - |HPF(setpoint)|/LEVEL). The HPF is (setpoint -
 * LPF(setpoint)) with the LPF below (also feeds the B-term derivative). */
#define STAB_CYCLIC_RELAX_DPS  60.0f  /* active (step 2) */
#define STAB_CYCLIC_SET_LPF_HZ 12.0f

/* RF error decay (step 2): the stored cyclic integral bleeds toward zero with
 * this time constant, ONLY while stab is engaged (airborne-ish). It carries
 * live correction, not history (trim offsets, gyro bias, maneuver leftovers).
 * Deliberate trade: a held bank drains over ~tau seconds — raise tau for
 * longer hold, but RF ships decay ON for a reason. */
#define STAB_CYCLIC_DECAY_TAU_S  2.0f

/* INPUT low-pass on the roll/pitch gyro rates feeding the rate PID (stock did
 * the same: Math_LowPassFilter shift-2 ~= 5Hz at its tick). Strips rotor
 * vibration and landing-gear chatter from the FEEDBACK before the PID reacts,
 * on top of the 10Hz output LPF. Yaw stays raw — stock fed its tail raw gyro
 * too. */
#define STAB_CYCLIC_IN_LPF_HZ  6.0f

/* NOTCH on the cyclic rate feedback (2026-07-19, flights 6-7). Flight 6
 * measured a steady 7.4-7.6Hz limit cycle on all axes, roll loudest — the
 * loop was feeding an airframe/head mode (KP 0.35 -> 0.25 KILLED it, flight
 * 7 spectrum clean in-band: that was the discriminating experiment). But
 * 0.25 lost the pendulum: flight 7 shows a broad 1.5-3Hz wallow, roll swings
 * to 140dps, pilot "compensation accumulates, out of stick" — the loop went
 * integral-dominant (KP 0.25 vs KI 1.2) and I collects garbage the relax
 * then freezes. So: gain comes BACK (KP 0.35) and the 7.5Hz band is removed
 * SURGICALLY here instead of by blanket gain cut — the standing 6-8Hz note
 * said exactly this ("не давать усиления в полосе").
 *
 * RBJ biquad notch, fs=100Hz (IMU tick), f0=7.5Hz, Q=3 (BW ~2.5Hz: covers
 * the measured 7.38-7.62 lines and tolerates the tick jitter that detunes a
 * fixed-fs filter). No libm on this build — coefficients precomputed by
 * hand, normalized by a0; DC gain checked = 1.0 exactly:
 *   w0=2*pi*7.5/100=0.4712389, cos=0.8910065, sin=0.4539905, a=sin/6
 * Applied AFTER the 6Hz input LPF, cyclic axes only (yaw untouched — its
 * band involvement was passenger-level). State reset with the PIDs. */
#define STAB_CYCLIC_NOTCH_ENABLE 1
#define STAB_NOTCH_B0   0.9296571f
#define STAB_NOTCH_B1  (-1.6566633f)
#define STAB_NOTCH_B2   0.9296571f
#define STAB_NOTCH_A1  (-1.6566633f)
#define STAB_NOTCH_A2   0.8593142f

/* Stock's adaptive gain: the correction divisor grew with stick deflection
 * (|stick|*6+900), i.e. full stabilization authority at centre, ~1.67x softer
 * at full stick — the pilot's command outranks the loop. corr *= 1/(1+K*frac),
 * frac = max(|roll|,|pitch|)/full-throw. */
#define STAB_CYCLIC_SOFTEN     0.67f
/* RATE mode gets its OWN integral + output limits (the shared SWASH ones below
 * are sized for attitude mode). The integral is the "rate hold" that makes the
 * swash ACCUMULATE and then HOLD while the stick is deflected (pilot wants this
 * visible, like stock). A high integral limit lets it ramp the swash over ~0.5s
 * instead of clamping at 80 in ~0.1s. The output limit is the max cyclic
 * COMPENSATION authority — raised so stronger correction isn't clipped. */
/* Integral limit sized to the OUTPUT clamp: KI * I_LIMIT ~= OUT_LIMIT. The
 * earlier 600 let the integral wind ~2x past what the output could express —
 * pure windup whose unwind time added phase lag and pumped a slow swash
 * wobble on the bench (2026-07-13, blades at speed). */
#define STAB_CYCLIC_RATE_I_LIMIT   200.0f  /* dps·s; x KI(1.2) ~= 250us clamp
                                            * (resize WITH KI — learned twice) */
#define STAB_CYCLIC_RATE_OUT_LIMIT 250.0f  /* us */

/* Roll/pitch attitude-hold (self-level assist), degrees -> microseconds.
 * Used only when STAB_CYCLIC_RATE_MODE = 0. */
#define STAB_ROLL_KP   6.0f
#define STAB_ROLL_KI   0.5f
#define STAB_ROLL_KD   0.8f
#define STAB_PITCH_KP  6.0f
#define STAB_PITCH_KI  0.5f
#define STAB_PITCH_KD  0.8f
#define STAB_SWASH_INTEGRAL_LIMIT   80.0f  /* us, anti-windup clamp */
#define STAB_SWASH_OUTPUT_LIMIT     150.0f /* us, max correction authority */

/* Tail ESC hover anti-torque base, us: the heading-hold PID corrects AROUND
 * this. Was SRV_CENTER (1520 = ~52% power) — far too hot for this tail motor
 * ("очень большой запас по мощности", pilot 2026-07-12: races at no load).
 * Lowered to a gentle starting point; tune on the bench until the tail just
 * balances main-rotor torque in hover, the PID handles the rest. */
/* TAIL FEEDFORWARD — physics-split (2026-07-13, refined past the stock law).
 * Stock held the table rock-solid with tail = THROTTLE*0.5 + PID, negative
 * corrections x4.2, no D (decoded from heli_decompiled.c). But (pilot):
 *   - stock's tail motor was the weak brushed one; ours is far stronger, so a
 *     50% throttle fraction is wildly oversized;
 *   - main-rotor torque comes mostly from COLLECTIVE loading the spun-up disk
 *     (stock's throttle term only tracked that implicitly, throttle and pitch
 *     sharing one stick in normal mode), plus a burst during spool-up.
 * So the feedforward is split into its real physical parts:
 *   ff = BASE                              blade profile drag, keeps ESC alive
 *      + COLL_GAIN * |coll-centre| * thr   blade-loading torque (the main term)
 *      + SPOOL_GAIN * d(thr)/dt (+only)    rotor spin-up reaction
 * PID corrects on top; thrust-REDUCING corrections keep the stock x4.2 boost
 * (a one-direction tail sheds thrust poorly — that physics didn't change). */
#define STAB_TAIL_FF_BASE_US  105.0f  /* us above floor at any live throttle.
   * 60 -> 105 (2026-07-17, MEASURED by the yaw blackbox, first instrumented
   * flight): the yaw integral held a steady +46us of tail thrust on top of
   * the FF through the whole flight (corr_i mean -46, range -19..-64), and
   * the deficit did NOT shrink when collective dropped 0.9 -> 0.6 on landing
   * (-46 -> -55us) — so the miss is in the BASE term, not FF_COLL. Cost of
   * the old value: every throttle-idle pass zeroes the integral, so each
   * takeoff flew ~2-4s with the tail 45us weak while I re-wound — the likely
   * "tail kicks right after liftoff" report. Expected on the next blackbox
   * log: corr_i centred near 0; if it goes persistently POSITIVE, this
   * overshot (battery sag made the flight-long deficit drift -37 -> -51, so
   * the true value is a mid-pack compromise). */
#define STAB_TAIL_FF_COLL_US  200.0f  /* us at full collective x full throttle */
#define STAB_TAIL_FF_SPOOL_US 120.0f  /* us per unit throttle-frac/s of spool-up */
#define STAB_TAIL_FF_SPOOL_MAX 150.0f /* us cap on the spool term */
#define STAB_YAW_NEG_BOOST    1.0f    /* RETIRED to 1.0 (2026-07-16): multiplied
                                       * the WHOLE correction (integral included)
                                       * on the thrust-reducing side — crude.
                                       * Superseded by the Rotorflight-style
                                       * directional STOP GAINS below, which
                                       * scale only the P term. Keep the macro
                                       * for a quick revert. */

/* Rotorflight-style directional stop gains (ported 2026-07-16 from RF pid.c:
 * stopGain = transition(errorRate, -10, +10, CCW, CW), scales ONLY the P term).
 * Boosts the loop's reaction while KILLING rotation — the "crisp stop" knob.
 * Named by TAIL THRUST demand instead of CW/CCW so the CORR_SIGN flip can't
 * mislabel them: _UP = error demands more thrust (strong, fast side of our
 * one-way tail), _DOWN = error demands less (weak side: ESC dead time ~60ms,
 * prop coasts down) — gets the bigger boost. Blend over ±STOP_TRANS dps of
 * error so the gain switch is smooth, not a relay. Too high => fast yaw
 * oscillation (RF docs). */
/* YAW iterm relax (step 1.5, 2026-07-16): freeze yaw-I accumulation while the
 * rudder setpoint moves fast. Cures the RUBBER-BAND tail: on stick release the
 * setpoint steps to 0 while the nose (ESC lag) still rotates — that transient
 * loaded the integral with "rotate back" error and the tail returned to the
 * OLD heading instead of holding where released. With relax, stopping is P/
 * stop-gain work; I only holds an already-standing heading. */
#define STAB_YAW_RELAX_DPS    30.0f
#define STAB_YAW_SET_LPF_HZ   12.0f

#define STAB_YAW_STOP_GAIN_UP    1.0f
#define STAB_YAW_STOP_GAIN_DOWN  1.0f  /* SYMMETRIC (step 1, 2026-07-16): the 1.4/2.2 asymmetry rectified hover gyro noise into a steady CW yaw drift (asymmetric P answers symmetric noise with a DC push). Symmetric 1.8 == plain P boost; re-split gently later if stops need direction-specific help.
   * 1.4 -> 1.0 (2026-07-19, flight 8 rate log): full-rudder release rang at
   * ~4Hz, decay ~0.05 (-181 -> +136 -> -116 -> +72 dps, ~yaw spectrum top
   * 3.5-4.5Hz) — the tail's OLD marginal frequency, still underdamped at
   * eff. P 2.8 (KP 2.0 x 1.4; symmetric stop = plain boost). Same direction
   * that already worked once (3.6 -> 2.8 killed open pulsing). Expected:
   * ring decays faster, stops slightly softer; heading stiffness lives on
   * KI 2.0 and stays. If ringing persists -> filtered yaw KD next (phase
   * lead vs the ~90ms ESC dead time); in-flight relay autotune in reserve.
   * Verify with the same metric: rate-log yaw extremes after a rudder
   * release. NB the pilot's original "~4Hz" feel was THIS ring — it
   * coexisted with the 7.4Hz cyclic buzz all along. */
#define STAB_YAW_STOP_TRANS_DPS  10.0f

/* Cyclic-to-yaw precomp (RF: mainDeflection = |coll|*collFF + |cyclic|*cycFF):
 * cyclic input loads the disk and adds torque too — feed the tail BEFORE the
 * gyro sees the kick. us at full cyclic deflection x full throttle. */
#define STAB_TAIL_FF_CYC_US   0.0f   /* OFF (step 1): reacted to standing cyclic trim, adding constant tail thrust; re-enable later with a trim deadband */

/* RUDDER FEEDFORWARD (2026-07-19, flight 9 "уперся в стену на развороте").
 * Until now the pedal fed ONLY desired_yaw_rate, so every degree of commanded
 * turn had to be manufactured by the PID and squeezed through its
 * output_limit — the tail topped out around FF+250 = ~1500us while the motor
 * reaches 2000. Worse, the yaw iterm relax FREEZES the integral exactly during
 * a fast pedal move, so a decisive turn-in ran on P alone. This term gives the
 * pedal its own path to thrust, the way stock and Rotorflight do it; the PID
 * is left to trim around it instead of generating the whole maneuver.
 *
 * GAIN IS MEASURED, not guessed: flight 8's rate log shows full pedal with the
 * correction clamped at 250us producing a ~272 dps peak, i.e. roughly 1 dps of
 * yaw per 1 us of tail command. 0.9 us/dps is deliberately just UNDER that —
 * the feedforward should slightly under-deliver and let the PID top the rest
 * up, since an over-strong FF makes the loop fight the pilot's own command.
 * Full pedal (~330 dps) -> ~297us, under the 320 clamp.
 * TUNING: turn still walls -> raise toward 1.1; yaw feels twitchy or overshoots
 * the commanded rate -> lower toward 0.7. Sign is derived in stabilize.c from
 * STAB_YAW_CORR_SIGN (read the comment there before touching it). */
#define STAB_TAIL_FF_RUD_PER_DPS  0.9f    /* us of tail per dps commanded */
#define STAB_TAIL_FF_RUD_MAX_US 320.0f    /* clamp on the pedal FF term */

/* Sign of the yaw correction on the tail ESC. The 2026-07-05 note claimed
 * +tail_us = nose-right (making +1 negative feedback), but with the motors
 * actually RUNNING the pilot observed the response inverted (2026-07-12), so
 * -1. One of the two observations mislabeled a direction; the macro makes it
 * a bench flip either way. Re-verify: with the tail spinning, a quick
 * nose-RIGHT twist must SLOW the tail, nose-LEFT must speed it up. NOTE: this
 * also flips the rudder-stick feedforward; if the stick then works backwards,
 * negate STAB_YAW_STICK_TO_DPS instead of touching this. */
#define STAB_YAW_CORR_SIGN  (-1)

/* Yaw heading-hold (rate loop), deg/s -> microseconds on the tail ESC.
 * Stock-derived (2026-07-13): P ~6us/dps (stock rate_err/2..3 in 16.4LSB/dps
 * units), I roughly an order hotter than our old 1.5, D absent in stock.
 * History: our earlier low-gain fixed-base attempts limit-cycled at ~4Hz on
 * the bench — that was the STRUCTURE (fixed base + symmetric gain vs one-sided
 * tail authority + table stick-slip), not the gains; see STAB_TAIL_FF_FRAC. */
/* Tail gains, THIRD cut 2026-07-13. The oscillation frequency stayed ~3-4Hz
 * through every gain change — that frequency is set by the ESC+prop response
 * lag (~80ms), gain only decides whether it decays. LOOP gain = these us-gains
 * x the motor's thrust-per-us, and our overpowered tail has several times the
 * stock brushed motor's thrust-per-us — so stock-derived numbers (KP~6) ran
 * the loop several times hotter than stock actually was. Cut to ~1/3; raise
 * back toward the wobble threshold in small steps once stable. */
#define STAB_YAW_KP    2.0f
#define STAB_YAW_KI    2.0f   /* heading STIFFNESS ("подтягивает к позиции").
                               * 1.0 held limply — partly because the integral
                               * limit below was still sized for the old KI=6
                               * and capped hold authority at 40us. Keep
                               * KI x I_LIMIT ~= OUT_LIMIT when retuning. */
#define STAB_YAW_KD    0.0f
#define STAB_YAW_INTEGRAL_LIMIT   125.0f  /* dps*s; x KI(2) = 250us. UNCHANGED:
   * this bounds the heading-hold windup and must stay tight; the extra turn
   * authority below comes from P, not from letting I wind further. */
#define STAB_YAW_OUTPUT_LIMIT     250.0f  /* us, max CORRECTION authority.
   * Briefly raised to 600 (2026-07-19) to break the "уперся в стену на
   * развороте" wall by brute force, then put BACK to 250 the same session:
   * that build never flew, and the real cure — STAB_TAIL_FF_RUD_* above —
   * gives the pedal its own path to thrust, so the PID no longer needs a wide
   * clamp to produce a turn. Keeping 250 also keeps the 4Hz post-release ring
   * partly clamped (it swings yaw_err to ~180 dps = 360us of P, which 600
   * would have passed in full). So vs the last FLOWN config the only change
   * is the feedforward — one variable. Raise this only if the FF proves
   * insufficient AND the ring stays clean. */

/* Rudder stick deflection (us from center) -> desired yaw rate (deg/s).
 * NEGATIVE (2026-07-13): with the gyro sign correct (STAB_YAW_CORR_SIGN=-1,
 * verified under power), the rudder STICK came out reversed — the two are
 * independent, and per that macro's note the stick is the one to flip here,
 * NOT the gyro sign. Magnitude raised 0.6->1.0 (2026-07-13: stick felt weak).
 * LOWERED 1.0 -> 0.5 (2026-07-19, pilot "очень чувствителен стик yaw, надо
 * занизить до практичных уровней"). The 2026-07-13 raise was compensating for
 * a problem we have since FIXED: back then the pedal fed only a rate demand
 * that the PID had to build up through its output_limit, so the tail answered
 * LATE and the stick felt weak — the pilot asked for more rate to get response
 * sooner. STAB_TAIL_FF_RUD_* now delivers the thrust immediately, so the
 * compensation became an over-correction and full pedal was asking for 330
 * dps. 0.5 = ~165 dps at full stick (360 deg in ~2.2s), a practical circuit
 * rate; it will still feel CRISPER than 0.6 ever did pre-feedforward, because
 * what was missing then was timing, not magnitude. The feedforward scales
 * itself (it is expressed per dps of demand), so nothing else needs retuning.
 * Raise toward 0.7 if pirouettes feel lazy. */
#define STAB_YAW_STICK_TO_DPS  (-0.5f)

typedef struct {
    float kp, ki, kd;
    float integral;
    float integral_limit;
    float output_limit;
    float prev_error;
} PID_t;

float PID_Update(PID_t *pid, float setpoint, float measurement, float dt_s);
float PID_UpdateRelax(PID_t *pid, float setpoint, float measurement, float dt_s,
                      float iterm_relax);
void PID_Reset(PID_t *pid);

/* Correction to apply pre-mix. pitch_axis/roll_axis are deltas added to the
 * raw stick pitch/roll axes (same microsecond units) before main.c's CCPM
 * mix. tail_us is the finished heading-hold tail ESC value, clamped to the
 * 1000-2000us band. */
typedef struct {
    float pitch_axis;
    float roll_axis;
    uint16_t tail_us;
} StabCorrection_t;

void Stabilize_Init(void);

/* Step the stabilization loops once (call at a fixed cadence with a fresh
 * g_orientation and the seconds elapsed since the previous call). roll_us/
 * pitch_us/rudder_us are the raw spring-centred (~1520) stick channels: in
 * RATE mode roll_us/pitch_us set the commanded cyclic RATE, in ATTITUDE mode
 * they are unused (self-level holds 0). rudder_us always sets the desired yaw
 * rate. motor_frac is the MAIN motor drive fraction 0..1 (post throttle
 * curve); coll_frac is the collective position 0..1 (0.5 = flat pitch). Both
 * feed the tail's physics-split anti-torque feedforward (see STAB_TAIL_FF_*).
 * Returns zero corrections / tail=SRV_CENTER when STAB_ENABLE is 0. */
StabCorrection_t Stabilize_Compute(uint16_t roll_us, uint16_t pitch_us,
                                   uint16_t rudder_us, float motor_frac,
                                   float coll_frac, float dt_s);

/* Zero the PID integrators/history — call whenever stabilization is
 * disengaged (motor idle, signal loss) so it re-engages without windup. */
void Stabilize_Reset(void);

/* ---- STAB BLACKBOX (2026-07-17, cyclic fields added 2026-07-19) ----------
 * SWD dies from RF pickup at flight RPM, so the only way to see what the loops
 * did in flight is to record it on board and read RAM after landing WITHOUT
 * cycling power (same methodology as the statRcv/statLost counters). Ring
 * buffer of one record per STAB_BB_PERIOD_S of ENGAGED time (samples
 * accumulate only while Stabilize_Compute runs, so the buffer freezes at
 * throttle idle and the landing doesn't overwrite the flight). Noisy signals
 * (angles, corrections, tail err/cmd) are time-weighted means over the window
 * (decimating raw 100Hz samples to 4Hz would alias gyro noise); slow states
 * (targets, tail integral) are snapshots at window end. Read with
 * tools/bb_dump.sh.
 *
 * CYCLIC DRIFT DIAGNOSIS (why this grew): pilot reported a steady hover trim
 * (rolls left, pitches forward, held back+right the whole hover). In mode 2
 * that is one of two things, and these fields separate them:
 *   - ESTIMATE bias (accel/IMU-mount thinks a real tilt is "level"): *_deg
 *     reads ~0 and *_corr ~0 while the machine physically leans — the loop is
 *     content at a tilt.
 *   - MECHANICAL/CG trim the loop can't fully null: *_deg reads a real steady
 *     angle and *_corr sits at a steady nonzero push trying to hold it.
 *   steer bits flag windows where the STICK was moving the target, so a hold
 *   (both bits 0) is told apart from active steering.
 *
 * RAM: 20 bytes x 112 = 2240B of the 4K (leaves ~1.2K stack). N dropped from
 * 192 so the wider record still fits. Do NOT co-compile with the bench log
 * arrays (guarded by #error in main.c). */
#define STAB_BLACKBOX     1
#define STAB_BB_N         112     /* records; x0.25s = last 28s of flight */
#define STAB_BB_PERIOD_S  0.25f

typedef struct {
    int16_t  err_dps10;    /* tail: mean yaw-rate error (set - gyro), dps x10 */
    int16_t  yaw_i_x100;   /* tail: s_yaw_pid.integral at window end, dps*s x100;
                            * x KI(2) = standing tail us the loop holds */
    uint16_t tail_us;      /* tail: mean commanded tail ESC, us */
    int16_t  roll_deg100;  /* mean measured roll, deg x100 (+ = right down) */
    int16_t  pitch_deg100; /* mean measured pitch, deg x100 (+ = nose up) */
    int16_t  roll_tgt100;  /* roll TARGET at window end, deg x100 (mode 2) */
    int16_t  pitch_tgt100; /* pitch TARGET at window end, deg x100 (mode 2) */
    int16_t  roll_corr10;  /* mean roll swash correction injected, us x10 */
    int16_t  pitch_corr10; /* mean pitch swash correction injected, us x10 */
    uint8_t  coll200;      /* coll_frac x200 (100 = flat pitch) */
    uint8_t  steer;        /* bit0 = roll target steering, bit1 = pitch */
} StabBBRec_t;

/* Format sentinel, checked by tools/bb_dump.sh BEFORE it trusts any record.
 * Lives in flash (const) so the dumper reads the FLASHED image's identity, and
 * bump the low byte whenever the record layout changes. This exists because
 * 2026-07-19 the reader silently printed a full CSV of garbage off the WRONG
 * firmware (old 8-byte records read as 20-byte ones) — nothing flagged the
 * mismatch. Now a wrong/old image fails the magic check instead of lying.
 * High 3 bytes = 'BB\0' tag, low byte = layout version. */
#define STAB_BB_FMT   0xBB000003u

#if STAB_BLACKBOX
extern const uint32_t g_bb_fmt;       /* == STAB_BB_FMT; dumper's tripwire */
extern StabBBRec_t g_bb_log[STAB_BB_N];
extern volatile uint16_t g_bb_head;   /* total records ever written; newest =
                                       * (head-1) % STAB_BB_N, wrapped iff
                                       * head >= STAB_BB_N */
#endif

#endif /* STABILIZE_H */
