#!/usr/bin/env python3
"""Åström–Hägglund fit for the tail relay experiment.

    tools/relay_dump.sh --run | tools/relay_fit.py
    tools/relay_fit.py relay.csv          # or from a saved dump

Input: the relay_dump CSV (t_s,rate_dps,hi). The relay toggled the tail ESC
by +-d us around the base; the yaw rate settles into a limit cycle whose
period Tu and amplitude a give the ultimate gain Ku = 4*d/(pi*a) in the same
us-per-dps units our PID uses (output us = KP * err_dps).

Suggestions are STARTING POINTS for the bench/flight, not gospel: the
experiment runs with the main rotor off, so main-disk damping and the real
anti-torque load are absent. The gain CEILING here is still the stock ESC's
~80-100ms lag — expect Tu a few hundred ms and modest Ku until BLHeli_S.
"""
import csv
import math
import sys

D_US = 60.0  # relay half-amplitude, must match RELAY_D_US in main.c


def main():
    src = open(sys.argv[1]) if len(sys.argv) > 1 else sys.stdin
    t, rate, hi = [], [], []
    for row in csv.reader(src):
        try:
            t.append(float(row[0])), rate.append(float(row[1]))
            hi.append(int(row[2]))
        except (ValueError, IndexError):
            continue  # headers, openocd chatter
    if len(t) < 50:
        sys.exit(f"only {len(t)} samples — not a relay log")

    # Full periods = time between consecutive RISING edges of the relay state.
    rising = [i for i in range(1, len(hi)) if hi[i] and not hi[i - 1]]
    if len(rising) < 3:
        sys.exit(f"only {len(rising)} relay cycles — no limit cycle "
                 "(hysteresis too wide for the response, or d too small?)")
    periods = [t[b] - t[a] for a, b in zip(rising, rising[1:])]
    tu = sum(periods) / len(periods)
    cv = (max(periods) - min(periods)) / tu

    # Per-cycle amplitude from the rate swing; robust to the pirouette drift
    # because max-min is taken within one cycle.
    amps = []
    for a, b in zip(rising, rising[1:]):
        seg = rate[a:b]
        amps.append((max(seg) - min(seg)) / 2.0)
    amp = sum(amps) / len(amps)
    ku = 4.0 * D_US / (math.pi * amp)

    print(f"cycles={len(periods)}  Tu={tu:.3f}s (spread {cv * 100:.0f}%)"
          f"  a={amp:.1f}dps  Ku={ku:.2f} us/dps")
    if cv > 0.3:
        print("WARNING: period spread >30% — limit cycle not stationary, "
              "don't trust the numbers (rig swinging? wind? cord torsion?)")

    # Two PI rules; PID fields per stabilize.h (KI in us per dps*s).
    zn_kp = 0.45 * ku
    zn_ki = zn_kp / (tu / 1.2)
    tl_kp = ku / 3.2
    tl_ki = tl_kp / (2.2 * tu)
    print(f"Ziegler-Nichols PI : KP={zn_kp:.2f}  KI={zn_ki:.2f}")
    print(f"Tyreus-Luyben PI   : KP={tl_kp:.2f}  KI={tl_ki:.2f}  (gentler)")
    print("current (2026-07-17): KP=2.00  KI=2.00  "
          "(x stop gain 1.4 near stops)")


if __name__ == "__main__":
    main()
