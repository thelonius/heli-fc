#!/usr/bin/env python3
"""Spectrum of the high-rate gyro log — which axis carries the oscillation.

    tools/rate_dump.sh | tools/rate_fft.py
    tools/rate_fft.py rates.csv

Plain DFT (no numpy): 373 samples @50Hz -> 0.13Hz resolution, 0.5..20Hz shown.
Prints the per-axis spectrum peaks and a verdict line: the axis with the
dominant narrow peak is the oscillating one (yaw = tail loop/ESC; roll/pitch =
cyclic/airframe). Broad flat energy = vibration/turbulence, not a loop mode.
"""
import csv
import math
import sys

FS = 50.0


def dft_mag(x, f):
    n = len(x)
    re = sum(v * math.cos(2 * math.pi * f * i / FS) for i, v in enumerate(x))
    im = sum(v * math.sin(2 * math.pi * f * i / FS) for i, v in enumerate(x))
    return 2.0 * math.hypot(re, im) / n


def main():
    src = open(sys.argv[1]) if len(sys.argv) > 1 else sys.stdin
    cols = {'roll': [], 'pitch': [], 'yaw': []}
    for row in csv.reader(src):
        try:
            cols['roll'].append(float(row[1]))
            cols['pitch'].append(float(row[2]))
            cols['yaw'].append(float(row[3]))
        except (ValueError, IndexError):
            continue
    n = len(cols['roll'])
    if n < 100:
        sys.exit(f"only {n} samples — not a rate log")
    print(f"n={n} ({n / FS:.1f}s @ {FS:.0f}Hz)")

    freqs = [f / 8.0 for f in range(4, 161)]  # 0.5..20Hz, 0.125Hz step
    peaks = {}
    for ax, x in cols.items():
        m = sum(x) / len(x)
        x = [v - m for v in x]  # remove DC (mean rate / slow drift)
        spec = [(f, dft_mag(x, f)) for f in freqs]
        spec.sort(key=lambda p: -p[1])
        peaks[ax] = spec[:5]
        rms = (sum(v * v for v in x) / len(x)) ** 0.5
        top = ", ".join(f"{f:.2f}Hz:{a:.1f}" for f, a in spec[:3])
        print(f"{ax:6s} rms={rms:5.1f}dps  peaks: {top}")

    best_ax, (bf, ba) = max(
        ((ax, p[0]) for ax, p in peaks.items()), key=lambda t: t[1][1])
    print(f"\nverdict: strongest line = {best_ax} @ {bf:.2f}Hz, "
          f"amplitude ~{ba:.1f}dps")
    print("yaw -> tail loop/ESC territory; roll/pitch -> cyclic/airframe.")


if __name__ == "__main__":
    main()
