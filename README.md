# heli-fc — replacement flight-controller firmware for a V977R-class 3D helicopter

Bare-metal firmware for the flight controller of an XK/WLtoys **V977R-V6** micro
3D helicopter (STM32F031K6). It replaces the stock firmware entirely: its own
S-FHSS receiver driver, IMU driver, orientation filter, stabilization loops, and
a brushless-ESC output stage in place of the original brushed motor drive.

Written from scratch. The stock firmware was studied to learn the radio protocol
and the board's wiring; none of it is copied here, and none of it is
redistributed (see [Stock firmware](#stock-firmware)).

---

## ⚠️ Safety

**This flies a real aircraft with carbon blades at several thousand RPM. Treat
every build as unverified until you have proven it on your own bench.**

- Nothing here has been validated for anyone else's airframe, ESC, or transmitter.
- There is deliberately **no arming gate**: the motors run whenever throttle-cut
  is off and the receiver has signal. Bench with the blades off.
- Change **one variable per flight**. This project has been burned repeatedly by
  two-at-a-time changes.
- After any change to signs, gains, or mixing, re-check swash direction on the
  bench with the rotor stopped before flying.

No warranty. If you fly it, you own the outcome.

---

## Hardware

| Part | Detail |
|---|---|
| MCU | STM32F031K6 (Cortex-M0, 32 KB flash / 4 KB RAM), 48 MHz |
| Clock | 16 MHz `X1` crystal (HSE) → PLL ×3, with a timed-out fallback to HSI |
| Radio | TI CC2500, bit-banged SPI — Futaba **S-FHSS** receiver |
| IMU | InvenSense **ICM-20689** (marked MPU-6050A), bit-banged I2C |
| Outputs | 3× CCPM swash servos + main ESC on TIM2, tail ESC on TIM3, 125 Hz |
| Watchdog | IWDG active, ~60 ms — refreshed every main-loop pass |

Pin map, the full LQFP32 ledger, and the board survey are in
[`docs/heli.md`](docs/heli.md).

## Status

Flying. Cyclic runs a stock-style attitude cascade (outer angle → rate, inner
rate PID) with a hand-computed 7.5 Hz notch on the rate feedback; the tail is a
heading-hold loop with a physics-split feedforward. Transmitter bind persistence
is designed but **not implemented** — the receiver currently adopts the first
S-FHSS transmitter it hears on every power-up.

## Build and flash

Requires `arm-none-eabi-gcc` and `openocd` with an ST-Link.

```sh
cd src && make
```

A real build is around 17 KB, comfortably inside the 32 KB part. **If you get
exactly 32768 bytes you have picked up a stale artifact, not this build** —
check the path before you flash it. Never chain a flash behind an unchecked
build: this project once flashed a stale image onto a live aircraft because
`make` failed and the next command in the chain ran anyway.

```sh
openocd -f interface/stlink.cfg -f target/stm32f0x.cfg \
  -c "init" -c "reset halt" \
  -c "flash write_image erase src/firmware.bin 0x08000000" \
  -c "verify_image src/firmware.elf" \
  -c "reset run" -c "shutdown"
```

> **Never run `verify_image` (or any flash operation) against a *running*
> board.** OpenOCD downloads its CRC stub into the work area, which
> `stm32f0x.cfg` places at `0x20000000` with backup disabled — directly over the
> receiver's state struct — and never restores it. The receiver goes silently
> deaf until reset. The `error executing cortex_m crc algorithm` message is the
> tell, not benign noise. Plain memory reads (`mdw`, `dump_image`) go over the
> DAP without the work area and are safe on a live board.

Symbol addresses move on every rebuild — re-derive them rather than reusing a
number from an earlier session:

```sh
arm-none-eabi-nm src/firmware.elf | grep g_
```

## Layout

```
src/      firmware: radio (sfhss), IMU (mpu6500), orientation, stabilize, main
src/tests bring-up programs (blink, PWM, SPI) used to bisect the hardware
tools/    SWD readout + analysis: live telemetry, blackbox dump, rate FFT
docs/     board survey, protocol notes, tuning framework, board photos
```

`tools/` talks to a running board over OpenOCD: `read_sfhss.sh` for receiver
state, `mon_rate.tcl` for live link rate, `bb_dump.sh` + `rate_fft.py` for the
on-board blackbox and its spectrum. SWD dies from RF pickup at flight RPM, so
in-flight data is recorded to a RAM ring buffer and read after landing with the
power still on.

## Stock firmware

The original image is **not distributed here**, and neither is its
decompilation. What is published is the analysis: register tables, protocol
structure, timings, and addresses, in [`docs/heli.md`](docs/heli.md).

If you want to verify those findings against the same binary, dump your own off
your own board over SWD and check that you have the identical image:

```sh
openocd -f interface/stlink.cfg -f target/stm32f0x.cfg \
  -c "init" -c "halt" -c "dump_image stock.bin 0x08000000 0x8000" \
  -c "shutdown"
shasum -a 256 stock.bin
# e0a985746fc2f07508f689a9bce057c762f454622b821b2f1c531dfc541e4fa5
```

## Write-up

A long-form illustrated account of the reverse engineering — the S-FHSS
teardown, the control law, the IMU, and the firmware nuances — is published as a
case study at
<https://thelonius.github.io/projects/heli-anthology>.

## License

MIT — see [LICENSE](LICENSE).
