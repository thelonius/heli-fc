# Flashing and first setup

Everything needed to get this firmware onto a board, verify it came up, and
configure a transmitter for it.

Read [Safety](#safety-before-the-first-spin-up) before the first spin-up. This
firmware has no arming gate.

---

## 1. What it can be flashed to

**Target board: XK / WLtoys `V977R-V6`** — the flight controller from a V977
Power Star / V977R-class micro 3D helicopter. Silkscreen on the component side
reads `V977R-V6`; the MCU is an `STM32F031K6` in LQFP32.

A caveat worth understanding before you start. The back of the board is
silkscreened `Futaba v2.0` and carries a model checklist —
`K100R / K123R / K124R / A600R / A430R` — because the factory uses **one PCB
across a whole family of S-FHSS models** and picks the firmware per model. So a
board out of a different airframe may well be electrically identical. That does
**not** make this firmware a drop-in for those models: pin roles were confirmed
by continuity-testing *this* board, and every gain, servo direction, mixing
constant, and stick calibration in the source is specific to this airframe.
Treat any other model as untested — start from the bench checks in section 7 and
expect to re-derive directions and calibration.

### Required hardware conversion

This firmware **does not drive the stock brushed motors.** It emits standard
1000–2000 µs servo/ESC pulses at 125 Hz on the main and tail motor pins, which
means:

- The stock brushed motor drive stage (the `CMS4576` complementary FET pair) is
  expected to be **removed**. On the original board it is fed from the same pins
  this firmware now uses as ESC signal outputs.
- You need **two brushless ESCs** (main and tail) and matching motors.
- `PB1` must be left as a **high-impedance input**. It sits next to the tail
  output and driving it causes glitches on `PB0`.

If your board still has the brushed drive populated, this firmware is not
directly usable — you would be feeding an ESC signal into a FET gate.

### Pin map

| MCU pin | Timer | Role |
|---|---|---|
| `PA0` | TIM2_CH1 | Main motor ESC (also the PPM CH1 connector) |
| `PA1` | TIM2_CH2 | Front swash servo |
| `PA2` | TIM2_CH3 | Rear-right swash servo |
| `PA3` | TIM2_CH4 | Rear-left swash servo |
| `PB0` | TIM3_CH3 | Tail motor ESC |
| `PB1` | — | **must stay input / Hi-Z** |
| `PA6` / `PA7` | — | Status LEDs (blue / red) |
| `PB3` `PB4` `PB5` `PA8` | — | CC2500 radio: SO, SCLK, SI, CSn |
| `PB6` / `PB7` | — | IMU I2C: SCL / SDA |
| `PA13` / `PA14` | — | SWDIO / SWCLK — **keep free for the debugger** |

All outputs run at 125 Hz with the timer ticking at 1 MHz, so the compare
registers are written directly in microseconds (centre 1520, range 1000–2000).

---

## 2. What you need

### Hardware

- **ST-Link v2** (or v3) SWD debug probe. Clones work.
- Four wires to the SWD header: the through-holes above the MCU are silkscreened
  `GND CLK DIO 3.3V`.
- A **Futaba S-FHSS transmitter**. Any S-FHSS-capable Futaba radio should bind;
  the firmware implements the receiver side of the protocol.
- A bench power source for the board, and the aircraft with **blades removed**
  for everything in sections 6–7.

### Software

- `arm-none-eabi-gcc` — the ARM bare-metal toolchain.
- `openocd` — flashing and all SWD readout.
- Optional: `python3` with `numpy` for `tools/rate_fft.py` (flight-log spectra).

Install:

```sh
# macOS
brew install --cask gcc-arm-embedded
brew install openocd

# Debian / Ubuntu
sudo apt install gcc-arm-none-eabi openocd
```

Verify both are on the path:

```sh
arm-none-eabi-gcc --version
openocd --version
```

---

## 3. Wiring the debugger

The header is the row of through-holes **above the MCU**, silkscreened
`GND CLK DIO 3.3V` on the component side. Photographs of both sides, the
observed pigtail colour map, and the board layout are in the
[README's board section](../README.md#where-to-solder-swd).

| ST-Link | Board header |
|---|---|
| SWDIO | `DIO` (→ `PA13`) |
| SWCLK | `CLK` (→ `PA14`) |
| GND | `GND` |
| 3.3 V | `3.3V` |

> **Do not use the gold through-holes near the crystal on the back side** —
> those are the power switch pins. The 3.3 V lead is also easier to take from
> the rail beside the tantalum than from the header hole itself.

`BOOT0` must be **LOW** for the MCU to boot from flash — leave the strap alone.
If you ever manage to lock SWD by misconfiguring `PA13`/`PA14`, pulling `BOOT0`
HIGH boots the system bootloader instead and gets the debugger back in.

Check the probe sees the target before going further:

```sh
openocd -f interface/stlink.cfg -f target/stm32f0x.cfg -c "init" -c "halt" -c "shutdown"
```

---

## 4. Build

```sh
cd src
make
```

A real build is **around 17 KB**, comfortably inside the 32 KB part.

> **Sanity-check the size before every flash.** If `firmware.bin` is exactly
> `32768` bytes you are looking at a stale full-flash artifact, not this build.
> Flashing one of those onto a live aircraft has happened on this project: a
> `make` failed, the next command in the same shell chain ran anyway, and picked
> up an old image. Use absolute paths and never chain a flash behind an
> unchecked build.

```sh
ls -l firmware.bin   # expect ~17000, never 32768
```

---

## 5. Flash

```sh
openocd -f interface/stlink.cfg -f target/stm32f0x.cfg \
  -c "init" -c "reset halt" \
  -c "flash write_image erase firmware.bin 0x08000000" \
  -c "verify_image firmware.elf" \
  -c "reset run" -c "shutdown"
```

It must print `verified NNNNN bytes`. Note that the verify happens **while the
target is halted**, inside the same session, and `reset run` follows it — that
ordering matters:

> ### Never run `verify_image` against a *running* board
>
> OpenOCD downloads a CRC helper stub into the target's RAM to verify. The
> `stm32f0x.cfg` work area is at `0x20000000` with backup disabled — which is
> exactly where the receiver's state struct lives — and it is never restored.
> Verify a running board and the radio goes silently deaf until the next reset,
> with symptoms that look precisely like broken RF hardware. The message
> `error executing cortex_m crc algorithm` is the tell, not harmless noise.
>
> Plain memory reads (`mdw`, `mdh`, `dump_image`) go over the debug port without
> a work area and are safe on a live board. Only the CRC path bites.

`tools/flash.sh` wraps the same sequence.

### If `reset halt` times out

Seen on this board even with known-good firmware, usually after replugging the
probe. `halt` on its own connects fine — drop the `reset` and flash with
`halt` + `flash write_image erase` + `verify_image` + `reset run`. If it
persists, reseat and power-cycle before suspecting the firmware.

---

## 6. Verify it came up

Symbol addresses **move on every rebuild**. Always re-derive them; never reuse a
number from an earlier session:

```sh
arm-none-eabi-nm src/firmware.elf | grep g_
```

Then read the globals that matter over SWD (`mdb` for bytes, address from `nm`):

| Symbol | Expect | Meaning |
|---|---|---|
| `g_clk_hse` | `1` | Running on the 16 MHz crystal. `0` means the crystal failed to start and it fell back to the internal RC — the link will be less reliable. |
| `g_mpu_ok` | `1` | IMU answered a valid `WHO_AM_I`. **`0` means the IMU is dead and the machine must not fly.** |
| `g_mpu_addr` | `0x68` or `0x69` | Which I2C address answered. |
| `g_mpu_whoami` | `0x98` | `ICM-20689`. `0x70` would be a genuine MPU-6500; both are accepted. |

Other quick checks:

- **Heartbeat** — read `GPIOA_ODR` (`0x48000014`) repeatedly; the LED bits
  should toggle. If the program counter is parked in a fault handler instead,
  nothing else below will work.
- **Receiver** — `tools/read_sfhss.sh` dumps the S-FHSS state: with the
  transmitter on you should see the phase reach connected, the received-frame
  counter climbing, and `data[0..7]` tracking the sticks.
- **Live link rate** — `tools/mon_rate.tcl` shows frames per second. Only
  *deltas* are meaningful; the cumulative loss counters tick forever while the
  transmitter is off, by design.

> The ST-Link stops working reliably once the rotor is at flight RPM — RF pickup
> kills it. SWD measurements are only possible at low revs, so they do not
> represent flight conditions. That is why in-flight data is recorded to an
> on-board ring buffer and read after landing with the power still on
> (`tools/bb_dump.sh`, `tools/rate_dump.sh`).

---

## 7. Transmitter setup

### Binding

There is **no stored bind** yet. On every power-up the receiver adopts the
**first S-FHSS transmitter it hears**. On a bench with one radio this just works.
At a field with several Futaba radios powered on, it can latch onto someone
else's — power up with only your transmitter live. Persisting the bound address
to flash is designed but not implemented.

### Channels the firmware expects

| S-FHSS channel | `data[]` | Function | Notes |
|---|---|---|---|
| 1 | `[0]` | Aileron / roll | spring-centred, ~1520 µs |
| 2 | `[1]` | Elevator / pitch | spring-centred, ~1520 µs |
| 3 | `[2]` | Throttle (left stick) | **reversed** on the reference TX |
| 4 | `[3]` | Rudder / yaw | spring-centred, ~1520 µs |
| 5 | `[4]` | *(unused)* | static on the reference TX |
| 6 | `[5]` | Collective pitch | own channel, **reversed** |

**The firmware performs the 120° CCPM swash mix itself** (H-3: one front servo,
two rear), operating on raw stick axes. The transmitter must therefore **not**
apply swash mixing of its own — set it to the non-mixing / single-servo swash
type so it transmits raw aileron, elevator, and collective. If both ends mix,
the swash will move on the wrong axes. Verify this on the bench before trusting
it: tilt one stick at a time and confirm only the expected servos respond.

### Stick calibration — you will need to redo this

The throttle and collective endpoint constants in `src/main.c` were measured on
one specific transmitter and **are not universal**:

```c
#define THR_RAW_IDLE   1930   /* throttle stick at bottom */
#define THR_RAW_FULL   1109   /* throttle stick at top   */
#define COLL_RAW_LOW   1725   /* collective at bottom    */
#define COLL_RAW_HIGH  1328   /* collective at top       */
```

Both channels read *reversed* on that radio. To adapt to yours: power the board,
turn your transmitter on, and read `g_sfhss.data[]` over SWD
(`tools/read_sfhss.sh`) while moving each stick to both extremes. Put the values
you observe into those four constants and rebuild. Getting this wrong means the
throttle curve and the collective range are simply mapped to the wrong band.

---

## Safety before the first spin-up

- **Blades off** for every bench check. All of them.
- There is deliberately **no arming gate**: the motors run whenever throttle-cut
  is off and the receiver has signal.
- Confirm the **swash correction direction** with the rotor stopped and the
  motor above idle: tilt the airframe and check that the swash moves to *oppose*
  the tilt. If an axis amplifies instead, flip that axis's `STAB_*_CORR_SIGN` in
  `src/stabilize.h`.
- Confirm the **tail direction** with the tail motor spinning and props off
  before trusting the heading-hold loop; a positive-feedback yaw loop is
  dangerous.
- **Change one variable per flight.** This project has been burned repeatedly by
  changing two things at once and being unable to attribute the result.
- Sustained signal loss drives the swash to centre and both motors to off. That
  is the only failsafe; the per-frame failsafe *bit* is deliberately ignored,
  because the transmitter broadcasts a failsafe-flagged frame roughly every 30 s
  during normal flight.

Nothing here has been validated on anyone else's airframe. If you fly it, you
own the outcome.
