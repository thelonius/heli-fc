# Helicopter FC Firmware Modernization - Master Manifest

## 🎯 Project Goal
Replace obsolete brushed motor control with a modern Brushless ESC system (125Hz PWM) and implement a stable flight control loop using a TI2500 RF transceiver and MPU6500 IMU.

## 🛠 Hardware Specification
- **MCU**: STM32F031K6T6 (48MHz HSI48 / 8MHz HSI)

### 📌 Pinout Map
| Pin | Component | Role | Note |
| :--- | :--- | :--- | :--- |
| **PA0** | Main Motor ESC | PWM (TIM2_CH1) | Throttle (corrected 2026-07-05: wiring is shifted by one) |
| **PA1** | Front swash servo | PWM (TIM2_CH2) | CCPM |
| **PA2** | Rear-right swash servo | PWM (TIM2_CH3) | CCPM (mechanically reversed: DIR_REAR_R=-1) |
| **PA3** | Rear-left swash servo | PWM (TIM2_CH4) | CCPM |
| **PA6** | LED Blue | GPIO | Status / Heartbeat |
| **PA7** | LED Red | GPIO | Status / Heartbeat |
| **PB3** | Radio (CC2500) | SO (MISO) | Data Output (verified via SWD probe 2026-07-03) |
| **PB4** | Radio (CC2500) | SCLK (SCK) | SPI Clock |
| **PA15** | — (suspect) | do not use | Was guessed as CC2500 GDO from the stock EXTI15 handler, but reads stuck high with TX live (floating). Routed somewhere on the board — destination unknown, needs a continuity check. Poll RXBYTES over SPI instead. |
| **PB5** | Radio (CC2500) | SI (MOSI) | Data Input |
| **PA8** | Radio (CC2500) | CSn | Chip Select, active low |
| **PB0** | Tail Motor ESC | PWM (TIM3_CH3) | Yaw Control |
| **PB1** | - | **INPUT (Hi-Z)** | **MUST be input to avoid glitches on PB0** |
| **PB6/PB7** | IMU (ICM-20689) | I2C (SCL=PB6, SDA=PB7) | addr 0x68; MPU6500 reg/pin-compatible |

- **Radio**: CC2500 (SPI: PB3=SO, PB5=SI, PB4=SCLK, PA8=CSn; no usable GDO line — poll RXBYTES).
  PARTNUM=0x80, VERSION=0x03. Original firmware is an S-FHSS receiver: sync 0xD391,
  13-byte packets (pkt[0]=0x81), 30 channels 2404.0–2447.5 MHz step 1.5 MHz
  (hops by rewriting FREQ2/1/0 from a table at flash 0x3020/0x2C60, CHANNR always 0).
- **Outputs**:
    - PA0-PA3 $\rightarrow$ TIM2 (125Hz PWM) $\rightarrow$ Swashplate & Main ESC
    - PB0 $\rightarrow$ TIM3 (125Hz PWM) $\rightarrow$ Tail ESC
- **Sensors**: ICM-20689 IMU (I2C on PB6/PB7; MPU6500-register-compatible, WHO_AM_I=0x98)
- **Watchdog**: IWDG active (Timeout ~60ms). Must refresh with `0xAAAA` in main loop.

### 🔩 Board V977R-V6 — physical survey (photos 2026-07-15, backlit + front-lit)

Board silkscreen: **V977R-V6**. Photographed after removing the stock motor-drive
components in preparation for the brushless/ESC conversion.
Photos archived in `docs/board_photos_2026-07-15/` (backlit shots 12-53-26/32 are the
ones to use for track tracing; 12-54-08 is the **back side** of the same board,
photographed with the SWD pigtail and battery leads still attached).

**ICs on board (only 3 + power stage):**
- **STM32F031K6** — LQFP32, marking `F031K6 780FB DC9U / PHL 9 47 A`, ST logo.
  Confirms the K6 part (32K flash / 4K RAM). The 26.000 MHz crystal next to the radio
  belongs to the CC2500. The MCU's own crystal (`X1`, 16.000 MHz) lives on the **back
  side**, directly under the chip — see the back-side survey below. Our firmware
  currently ignores it and runs HSI+PLL.
- **CC2500** — TI radio, QFN, next to the 26 MHz crystal.
- **IMU** — package marked `InvenSense MPU-6050A 1839`, but WHO_AM_I reads 0x98
  (ICM-20689 die). Marking and die disagree; trust the WHO_AM_I.
- **CMS4576** (Cmos brand, SOP-8, complementary MOSFET pair) + 100 µH inductor
  (`101`) + 47 µF tantalums near the B+/B− pads — power/motor-drive stage.
- SWD header on silkscreen: `GND CLK DIO 3.3V`, through-holes above the MCU.

**MCU orientation (pin 1 key):** the pin-1 dot sits at the corner next to the ST
logo. With the board held B+ top-right / 26 MHz crystal top-left (as in the backlit
photos), pin 1 is the **bottom-left** corner: pins 1–8 run along the bottom edge
left→right, 9–16 up the right edge, 17–24 along the top edge right→left, 25–32 down
the left edge. Cross-checked against known nets: PB3/PB4/PB5 (left edge, upper half)
head toward the CC2500, PB6/PB7 (left edge, lower half) toward the IMU, PA13/PA14
(top edge, left end) go to the CLK/DIO through-holes, PA0–PA3 (bottom edge) fan out
toward the servo/ESC connectors. All consistent — orientation is certain.

**Full pin ledger (LQFP32):**

| # | Pin | Status |
|---|-----|--------|
| 1 | VDD | power |
| 2 | PF0 | **NOT free** — OSC_IN, 16 MHz crystal `X1` on the back side under the MCU |
| 3 | PF1 | **NOT free** — OSC_OUT, same crystal |
| 4 | NRST | reset |
| 5 | VDDA | power |
| 6 | PA0 | main ESC PWM (TIM2_CH1) — PPM CH1 connector; stock emitted the same 1000-2000µs ESC signal here (see Main motor signal path note) |
| 7 | PA1 | front swash servo (TIM2_CH2) |
| 8 | PA2 | rear-right swash servo (TIM2_CH3) |
| 9 | PA3 | rear-left swash servo (TIM2_CH4) |
| 10 | PA4 | **candidate free** — right edge glared out in photos, beep-test. ADC_IN4 |
| 11 | PA5 | **candidate free** — same, ADC_IN5 |
| 12 | PA6 | blue LED |
| 13 | PA7 | red LED |
| 14 | PB0 | tail PWM (TIM3_CH3) |
| 15 | PB1 | held Hi-Z; "joined to PB0" verdict is under review — beep PB0↔PB1 now that the motor keys are desoldered; if separate, PB1 = free TIM3_CH4 (tail-reverse candidate). See Main motor signal path note |
| 16 | VSS | power |
| 17 | VDD | power |
| 18 | PA8 | CC2500 CSn |
| 19 | PA9 | **candidate free** — USART1_TX if confirmed |
| 20 | PA10 | **candidate free** — USART1_RX; backlit photo shows two adjacent bare pads mid-top-edge (PA10–PA12 zone) with no track at all |
| 21 | PA11 | **candidate free** — same zone |
| 22 | PA12 | **candidate free** — same zone |
| 23 | PA13 | SWDIO → `DIO` hole (keep!) |
| 24 | PA14 | SWCLK → `CLK` hole (keep!) |
| 25 | PA15 | routed somewhere but reads stuck high (stock FW had EXTI15 on it); destination unknown — avoid until traced |
| 26 | PB3 | CC2500 SO |
| 27 | PB4 | CC2500 SCLK |
| 28 | PB5 | CC2500 SI |
| 29 | PB6 | I2C SCL (IMU) |
| 30 | PB7 | I2C SDA (IMU) |
| 31 | BOOT0 | strap — leave alone |
| 32 | VSS | power |

**Photo verdict on free pins:** at least two of PA10/PA11/PA12 are visibly unrouted
(bare pads on the backlit shot). The right edge (PA4/PA5) is blown out by the
backlight hotspot in every frame, so PA4/PA5/PA9 need a multimeter continuity
check against neighboring pads before soldering to them.

**Back side survey (photo 12-54-08).** Back silkscreen: `Futaba v2.0` + a model
checklist `K100R / K110R / K123R / K124R / A600R / A430R` — the factory uses one
PCB across the whole XK/WLtoys S-FHSS family and picks the firmware per model
(ours is silkscreened V977R-V6 on the component side). Orientation lock verified
by three anchors (unplated mounting hole, glue-filled via, B+/B− pads): the back
photo maps to the front via a top-over-bottom flip. Findings:
- **`X1` = 16.000 MHz SMD crystal directly under the MCU**, pads landing in the
  PF0/PF1 corner (pins 2–3). This is the MCU's HSE crystal; stock firmware very
  likely clocked from it.
- **☑ Clock upgrade DONE 2026-07-16 — we now run on X1.** HSE 16 MHz, PREDIV /1,
  PLL ×3 = the same 48 MHz, but with crystal (ppm) accuracy instead of HSI's
  ~1% and its drift with board temperature. Directly relevant to S-FHSS
  hop-window timing (1681 µs slot ⇒ HSI drift alone can contribute ~±17 µs).
  Confirmed on the bench: `RCC_CR` = 0x03036483 (HSEON+HSERDY up, HSEBYP=0 so
  it really is an oscillator), `RCC_CFGR` = 0x0005000a (SWS=PLL, PLLSRC=10b
  = HSE/PREDIV, PLLMUL=×3), `RCC_CFGR2` = 0 (PREDIV /1), and the link then
  acquired and hopped normally — the 1681 µs slot would not forgive a wrong
  clock. So the X1 guess off the photo was right, and stock did clock from it.
  **HSE start is timed out with a fallback to the old HSI/2 ×12 path** — a bare
  `while (!HSERDY)` would brick the board if the crystal ever fails to start.
  `g_clk_hse` (1 = crystal, 0 = fell back) reports which path booted; re-derive
  its address after any rebuild, symbols drift.
- Second 47 µF tantalum on the back, on the 3.3 V rail.
- Two power diodes near the battery inlet (back side).
- SWD pigtail is tacked onto the back of the header holes; probable color map
  (from photo alignment, beep before trusting): white=GND, green=CLK, orange=DIO,
  red=3.3V picked up from the rail next to the tantalum, not from the hole.
- The gold through-holes left of the crystal are the power switch pins, not SWD.

**☑ TODO — multimeter continuity session (before reassembly, board is open NOW):**
- [ ] **PB0 ↔ PB1** (pins 14↔15): if they do NOT ring, PB1 is a free hardware
  PWM pin (TIM3_CH4) — unlocks the tail-reverse bridge. If they ring, the
  "joined" verdict stands and PB1 stays Hi-Z forever.
- [ ] **PA4, PA5** (pins 10, 11): beep against neighbors (PA3/PA6) and against
  ground/3.3V planes; photos are glared out here. Free ⇒ ADC inputs for
  battery voltage / current sensing.
- [ ] **PA9-PA12** (pins 19-22): confirm the two visibly-bare pads and check
  the other two; PA9/PA10 free ⇒ USART1 telemetry.
- [ ] **PA15** (pin 25): find where its trace actually goes (reads stuck high;
  stock had EXTI15 on it). Until traced — do not use.

**Expansion shortlist once beeped out:**
- **PA9/PA10 = USART1** (AF1) — telemetry/debug UART, the highest-value add.
- **PA4/PA5 = ADC** — battery voltage (and current shunt) sensing for the ESC era.
- **PA9/PA10 also carry TIM1_CH2/CH3** — spare hardware PWM if the tail-reverse
  bridge sketch ever needs a second leg. (TIM14_CH1 on PA4 is useless for output —
  TIM14 is our radio-poll timebase.)

### 📡 CC2500 Transceiver Interface

**Physical Layer:**
- **SPI Type**: Hardware SPI1 (implemented via bit-bang)
- **Pins**: PB3=SO (MISO), PB5=SI (MOSI), PB4=SCLK, PA8=CSn (SWD-verified 2026-07-03)
- **Max Clock**: 10 MHz
- **Mode**: SPI Mode 0 (CPOL=0, CPHA=0)

**SPI Protocol (CC2500-specific):**
Every SPI transaction starts with a **header byte**:
```
Bit 7:   R/W (0=write, 1=read)
Bit 6:   Burst (0=single, 1=burst)
Bits 5-0: Register address
```
After CS goes low, **wait for SO (MISO) to go low** before sending header byte. This indicates crystal is running.

**Key Header Bytes:**
| Byte | Operation |
|------|-----------|
| `addr \| 0x80` | Single register read |
| `addr` | Single register write |
| `0xFF` | Burst read RX FIFO |
| `0x7F` | Burst write TX FIFO |
| `0x34` | SRX command strobe (enable receive) |
| `0x36` | SIDLE command strobe (exit RX/TX) |

**Status Register Read:**
Status registers (0x30-0x3D) are read-only. Read with burst bit set:
- `0x70` = PARTNUM (should return 0x00 for CC2500)
- `0x71` = VERSION (should return 0x03 for CC2500)
- `0xF5` = MARCSTATE (current state machine)
- `0xFB` = RXBYTES (bytes in RX FIFO)

**CC2500 State Machine (MARCSTATE):**
| Value | State |
|-------|-------|
| 0x00 | IDLE |
| 0x01 | XOFF |
| 0x02 | FSTXON |
| 0x03 | Initialize |
| 0x04 | RXM (RSSI measurement) |
| 0x05 | RX (receive mode) |
| 0x06 | RX_END |
| 0x07 | TX |
| 0x08 | TX_END |
| 0x09 | RX FIFO OVERFLOW |

**Initialization Sequence:**
1. Pull CS low, wait for SO low
2. Write 40 configuration registers (single-byte writes)
3. Send SRX command strobe (0x34) to enable receive
4. Pull CS high

**Register Configuration (from original firmware):**
| Addr | Name | Value | Purpose |
|------|------|-------|---------|
| 0x00 | IOCFG2 | 0x07 | GDO2 output: sync word detected |
| 0x01 | IOCFG1 | 0x2E | GDO1: 3-state (default) |
| 0x02 | IOCFG0 | 0x07 | GDO0 output: sync word detected |
| 0x03 | FIFOTHR | 0x0F | FIFO threshold: TX=33, RX=32 bytes |
| 0x04 | SYNC1 | 0x1C | Sync word high byte (VERIFIED) |
| 0x05 | SYNC0 | 0x43 | Sync word low byte (VERIFIED) |
| 0x06 | PKTCTRL1 | 0x0D | PQT threshold=0, append status bytes |
| 0x07 | PKTCTRL0 | 0x04 | Fixed packet length, no whitening |
| 0x08 | ADDR | 0x60 | Device address (VERIFIED) |
| 0x09 | CHANNR | 0x2C | Channel number (VERIFIED) |
| 0x0A | FSCTRL1 | 0x00 | IF frequency control |
| 0x0B | FSCTRL0 | 0x06 | Frequency offset |
| 0x0C | FREQ2 | 0x00 | Frequency word [23:16] |
| 0x10 | FREQ1 | 0x2C | Frequency word [15:8] |
| 0x11 | FREQ0 | 0x43 | Frequency word [7:0] |
| 0x12 | MCSM2 | 0x03 | RX timeout disabled |
| 0x13 | MCSM1 | 0x23 | CCA=IDLE, RX→IDLE, TX→IDLE |
| 0x14 | MCSM0 | 0x7A | Auto-cal: IDLE→RX/TX, PO_TIMEOUT=64 |
| 0x15 | FOCCFG | 0x44 | Freq offset compensation |
| 0x16 | BSCFG | 0x07 | Bit sync config |
| 0x17 | AGCCTRL2 | 0x0C | AGC target=33dB, max LNA gain |
| 0x18 | AGCCTRL1 | 0x08 | AGC carrier sense threshold |
| 0x19 | AGCCTRL0 | 0x1D | AGC filter length=32 |
| 0x1A | FREND1 | 0x1C | LNA current, mixer current |
| 0x1B | FREND0 | 0x43 | PA power=0, LNA power=3 |
| 0x1C | FSCAL3 | 0x40 | PLL calibration control |
| 0x1D | FSCAL2 | 0x91 | VCO current calibration |
| 0x1E | FSCAL1 | 0x57 | VCO capacitor calibration |
| 0x1F | FSCAL0 | 0x6B | VCO band calibration |
| 0x20 | FSTEST | 0xF8 | Frequency synthesizer test |
| 0x21 | PTEST | 0xB6 | Power-down test |
| 0x22 | AGCTEST | 0x10 | AGC test |
| 0x23 | TEST2 | 0xEA | Various test settings |
| 0x24 | TEST1 | 0x0A | Various test settings |
| 0x25 | TEST0 | 0x11 | Various test settings |
| 0x2C | TEST2 | 0x88 | Improved sensitivity at ≤100 kBaud |
| 0x2D | TEST1 | 0x31 | Various test settings |
| 0x2E | TEST0 | 0x0B | Various test settings |



**Reference Implementation: FHSS Example (opiopan/rcstick-f)**

The repository https://github.com/opiopan/rcstick-f provides a reference implementation of the FHSS (Frequency Hopping Spread Spectrum) protocol used by Futaba S-FHSS transmitters. It demonstrates how to configure the CC2500 for frequency hopping, handle packet formatting, and interface with USB HID. This can serve as a useful reference when adapting the CC2500 driver for this helicopter flight collector.

## 🚀 Execution Roadmap

### Phase 2: Input/Output Shell (CURRENT)
*Goal: Establish a "Stick $\rightarrow$ Servo" path.*
- [x] PWM 125Hz verified on all channels.
- [x] ESC arming sequence (1ms pulse) validated.
- [x] Failsafe implemented (1500 center / 1000 throttle).
- [x] **Spi-RC Validation**: DONE 2026-07-03. S-FHSS receiver (src/sfhss.c) locks a TX,
  reaches phase=CONNECTED, statRcv climbs continuously, all 8 channels decode into
  g_sfhss.data[] and track the sticks. **Real frequency hopping confirmed**: `ch` sweeps
  0-29 following the hopcode from decoded frames (not a proximity/static-channel hack).
  Read state via tools/read_sfhss.sh (g_sfhss pinned @0x20000000; re-derive offsets from
  the ELF's DWARF info if the struct changes — see comment in that script).
- [x] **Channel Mapping**: implemented 2026-07-04, needs on-hardware verification.
  Futaba heli order (swash mix on TX, H-3): CH1=ail servo→PA1, CH2=ele servo→PA0,
  CH3=throttle→PA3 (anchor: verified empirically — throttle stick moves data[2]),
  CH4=rudder→PB0 (tail ESC), CH5=throttle cut (flag: <1200us → both throttles to
  1000us; polarity unverified), CH6=pitch servo→PA2. data[n] = CH(n+1).
  **S-FHSS pair gotcha**: each hop channel carries ch1-4 frame then ch5-8 frame
  ~6.8ms later; TX hops only after the second. Hop-on-every-frame looks connected
  but never decodes ch5-8 (data[4..7] frozen). Chip auto-IDLEs after each RX
  (MCSM1), so waiting for the second frame requires an explicit SRX strobe.
  **PWM clock gotcha**: system runs on 8MHz HSI, so PSC=7 (→1MHz timer, CCR in us
  directly), NOT PSC=47 from the old 48MHz-based test. Failsafe (SFHSS_HasSignal
  false = no valid frame for 0.5s) drives all 5 channels to 1000us continuously;
  gate outputs on HasSignal, not the per-hop CONNECTED phase.
- [ ] **On-hardware check** (ST-Link was unplugged): flash latest build; confirm
  data[4..7] update; scope each output per stick; verify throttle-cut polarity.
  Routing lives in one table — `ch_map[]` in src/main.c (CHANNEL MAP block), one
  line per output; cut polarity is the THR_CUT_ENGAGED macro next to it.
- **EXIT CRITERIA**: Linear response of all actuators to RC stick movements.

### Phase 3: Sensor Integration
*Goal: Obtain calibrated orientation data.*
- [x] **I2C Driver**: rewritten 2026-07-04 (src/mpu6500.c). Originally built as
  SPI bit-bang (same shape as spi_rc.c), but continuity-testing the board
  found chip pin 22 (nCS) tied directly to VDD. Per the MPU-6500 datasheet
  (section 4.9 / Figure 6), that's exactly how I2C mode is selected — SPI
  needs the host to actively drive nCS. So this chip only speaks I2C: pin 23
  (SCL) and pin 24 (SDA, bidirectional open-drain — NOT a push-pull MOSI as
  first assumed) are the bus, and pin 9 (labeled AD0/SDO in the SPI diagram)
  is the address-select strap, not a data line. Confirmed by continuity test:
  **SCL = PB6** (chip pin 23), **SDA = PB7** (chip pin 24), both open-drain
  with internal pull-ups enabled in firmware. The I2C address (0x68 if AD0
  strapped low, 0x69 if high) isn't separately checked — `MPU6500_Init()`
  probes both via WHO_AM_I and uses whichever answers a known value, into
  `g_mpu_addr`.
  The doc's old claim that the IMU "shares PA5-PA7" cannot be trusted:
  `heli_ghidra_output/MPU6500_CONFIG.md` and the function Ghidra named
  `IMU_Init_Configure` turned out to be the CC2500 radio's init byte-for-byte
  (same sync/IOCFG table already used in sfhss.c), misattributed to the IMU —
  exactly the kind of Ghidra-name error that already burned us once on the
  CC2500 pinout, and on top of that this decompile's function is SPI-shaped,
  not I2C, so it couldn't have described this board's wiring at all. Only the
  *register values* in `IMU_ConfigureAllRegs`/`IMU_ReadGyro` (real MPU6500
  addresses: 0x6B, 0x19, 0x1A, 0x1B, 0x1C, 0x23, 0x37, 0x38, 0x6A, 0x6C, gyro
  read from 0x43) are trustworthy — those are just register addresses,
  independent of whether the wire protocol carrying them is SPI or I2C.
- [x] **MPU6500 Driver**: init sequence + config lifted from the real (verified
  by register semantics) `IMU_ConfigureAllRegs` values: reset, PLL clock,
  1kHz sample rate, 188Hz DLPF, ±2000°/s gyro, ±16g accel, FIFO/interrupts off.
  `MPU6500_CalibrateGyro(200)` averages 200 samples (~200ms) at boot; airframe
  must be still.
- [x] **Data Processing**: `MPU6500_GetSample()` converts raw LSB to deg/s
  (÷16.4, FS_SEL=3) and g (÷2048, AFS_SEL=3), gyro bias-corrected.
- [x] **On-hardware check**: DONE 2026-07-04. `g_mpu_addr=0x68`, `g_mpu_ok=1`,
  live pitch/roll/yaw_rate reading real, plausible values continuously
  (matches the board sitting on a table). **The chip isn't actually an
  MPU6500 — it's an ICM-20689** (WHO_AM_I=0x98, not MPU6500's 0x70). Same
  pinout/registers/protocol (ICM-20689 is InvenSense's pin/register-compatible
  successor), so nothing else needed to change — `MPU_IS_WHO_AM_I()` now
  accepts either value. Root-caused via a temporary per-step ACK diagnostic
  (added, used, then removed) rather than guessing: address ACK, register ACK,
  and restart-read ACK were all 1 from the very first live test — the I2C bus
  itself worked immediately once wired correctly, the only bug was rejecting
  a valid-but-unexpected WHO_AM_I value. i2c_delay() bumped from ~1us to
  several us per half-cycle before this was found, ruling out timing first;
  confirmed working at that setting, not yet re-tightened.
- **EXIT CRITERIA**: Clean, noise-filtered orientation data in SRAM — **met**.

### Phase 4: Stabilization Logic
*Goal: Implement the "Brain" (PID).*
- [x] **Orientation Filter**: implemented 2026-07-04 (src/orientation.c) as a
  complementary filter (gyro-integrated angle blended with an accel tilt
  estimate), NOT the original's full quaternion fusion — its gain constants
  (CONST_PITCH_GAIN/CONST_ROLL_GAIN) are opaque float symbols with no literal
  value recoverable from the decompile, and this MCU (Cortex-M0, no FPU,
  no libm) can't cheaply do atan2f/sqrtf, so the accel tilt estimate uses a
  small-angle approximation (valid near hover, degrades above ~30° bank).
  **Pitch/roll axis sign CONFIRMED 2026-07-04** by hand-tilting the airframe:
  nose-up gives +pitch_deg, right-side-down gives +roll_deg. Pitch's raw
  gyro/accel sign was backwards and had to be inverted in orientation.c to
  get there; roll's raw sign was already correct. **Yaw fully verified**:
  gyro_z_dps's sign was confirmed 2026-07-04 (positive = nose swinging
  right/clockwise from above, by slowly rotating the whole board), and the
  tail-rotor direction was confirmed 2026-07-05 by the user with the tail
  motor spinning (props off): raising the tail ESC's tail_us spins the nose
  that same +yaw_rate direction, so the heading-hold loop is negative
  feedback. See the comment in stabilize.c's Stabilize_Compute() yaw section.
- [x] **PID Controller**: implemented 2026-07-04 (src/stabilize.c). Roll/pitch
  run an attitude-hold PID (setpoint = level); yaw runs a rate (heading-hold)
  PID like a classic RC-heli tail gyro, since the tail output drives a
  fixed-pitch tail rotor ESC (not a servo) — rudder stick sets a desired yaw
  rate, gyro Z is the feedback. Gains are conservative, explicitly untuned
  placeholders (see STAB_*_KP/KI/KD in stabilize.h).
- [x] **Flight Mixer**: implemented 2026-07-04, reintegrated pre-mix
  2026-07-05. The 120° CCPM swash mix (+ per-servo DIR flags) lives in main.c
  and runs on raw TX stick axes. Stabilization corrections come back from
  Stabilize_Compute() as deltas on those SAME virtual pitch/roll axes and are
  added BEFORE the mix, so the mix routes and mirrors a correction exactly
  like a stick nudge — including mirroring it through the reversed rear-right
  servo. (The earlier post-mix version added straight to servo microseconds
  and got the reversed servo wrong; that is why it was disconnected during
  the CCPM refactor. Pre-mix injection fixes it with no matrix inversion.)
- [x] **STAB_ENABLE = 1** (src/stabilize.h) — stabilization ACTIVE. Enabled
  2026-07-05 once the last dangerous blocker (tail-rotor direction) was
  user-confirmed. IMU_POLL_ENABLE also flipped back to 1 (the radio
  degradation once blamed on IMU I2C traffic was the S-FHSS pair bug, now
  fixed). Corrections auto-disengage (PIDs reset, no windup) whenever the
  motor is idle or signal is lost, and the heading-hold tail only runs when
  g_mpu_ok. **Still bench-check before flight**: the swash correction
  *direction* (rotor OFF, motor above idle — tilt the airframe, confirm the
  swash moves to counteract, not amplify). If an axis amplifies, flip
  STAB_PITCH_CORR_SIGN / STAB_ROLL_CORR_SIGN in stabilize.h.
- **EXIT CRITERIA**: Servo response compensates for board tilt — code
  complete and active. Pitch/roll sensor signs + yaw direction confirmed on
  the bench. Open: bench-verify swash correction *direction* (rotor off),
  then real gain tuning.

### Phase 5: Integration & Flight
*Goal: Safe first flight.*
- Deferred per instruction: **ARM/DISARM and tilt-lockout are not implemented**
  (motors currently run whenever throttle-cut is not engaged and the
  radio has signal — no separate arming gate).
- [x] **Swashplate reset when motor off**: implemented 2026-07-04 (main.c).
  When throttle-cut is engaged OR the raw throttle channel is below
  `THROTTLE_IDLE_THRESH_US` (placeholder 1050us — the TX's real idle endpoint
  hasn't been measured on hardware), front/rear-L/rear-R are forced to
  PWM_CENTER (neutral/level), overriding both raw stick swash input and any
  stabilization correction. Separate from total-signal-loss failsafe, which
  already forces every channel including swash to PWM_MIN per the earlier
  "all zero" requirement — this covers ordinary flight with signal present
  but the pilot at idle/motor stopped, where the swash sitting at a stray
  stick position or a stale stabilization correction serves no purpose.
- [ ] **Ground Tests**: Verify all directions and failsafes.
- [ ] **Maiden Flight**: Real-time PID tuning.
- **EXIT CRITERIA**: Stable hover.

### Phase 6: Transmitter Bind (PLANNED 2026-07-15)
*Goal: work with any S-FHSS transmitter, not just ours — and ONLY with the one
the user bound, not whichever TX powers up first.*

**Current behavior (src/sfhss.c):** `start_bind()` sets `txaddr = -1` and
`parse_packet()` adopts the first txaddr heard (`if (txaddr < 0) txaddr = pkt`).
So the RX already auto-binds to ANY S-FHSS TX on every power-up. Two gaps:
1. Nothing persists — a power cycle re-binds from scratch.
2. First-heard wins — at a field with several Futaba-protocol TXs powered on,
   the RX can latch onto a neighbor.

**Plan:**
- [ ] **Flash config page**: reserve the last 1K page (0x08007C00) for
  `{magic, txaddr, freqOffset(FSCTRL0), crc}`. Stock precedent: the original
  firmware stored IMU calibration in flash at offset 0x3C00, so in-app flash
  writes are proven safe on this part. Write with IRQs masked between radio
  slots (flash stalls the bus; a page erase is ~20-40ms → do it only in bind
  mode, never in flight).
- [ ] **Boot logic**: config valid → skip adoption, accept only the stored
  txaddr (drop others in `parse_packet` as today via the txaddr filter).
  Config empty → bind mode.
- [ ] **Rebind trigger without buttons**: triple power-cycle counter — bump a
  flash counter at boot, clear it after ~3s of uptime; three boots inside 3s
  ⇒ enter bind mode. (Board has no free button; the power switch is the only
  user control. Alternative if this proves flaky: bind when throttle stick
  held full + rudder full right for 2s — but that requires an already-bound
  TX, so it can't be the only path.)
- [ ] **Bind procedure**: camp CHANNR 0 (as FINDING does), first valid frame
  gives txaddr + hopcode (hopcode arrives in EVERY packet — no hop table to
  store), run the existing freqOffset search, then store config and blink the
  LED for confirmation. Reuse the existing FINDING/BINDED machinery — bind is
  just "FINDING with persistence".
- [ ] **LED language**: bind mode = fast blink; bound+connected = solid;
  bound+no signal = slow blink.
- **EXIT CRITERIA**: fresh board + any S-FHSS TX binds once; after that,
  power-cycling reconnects to the stored TX only, with a second TX live nearby.

### Phase 7: Stock-parity ground gestures (PLANNED 2026-07-21)

Stock firmware had two ground-only stick gestures in `Control_AutoCalibrate`
(heli_ghidra_output/heli_decompiled.c @ 0x1C80) that we should reimplement —
the pilot's muscle memory expects them, and both matter for field use without
a laptop:

- [ ] **Stick-gesture IMU calibration**: stock trigger = throttle low (<20) +
  both cyclic axes in a corner (< -200) + ch4 deflected (> 21) → full
  gyro+accel recalibration (`IMU_CalibrateFull(1,1)`), same gesture the K110
  manuals document as "both sticks down-and-inward". Ours currently calibrates
  gyro only at boot on a still airframe; a field recal gesture removes the
  power-cycle-on-a-level-surface dance. Gate it HARD on motor off + collective
  low; save offsets to the flash config page from Phase 6.
- [ ] **Level/trim save gesture**: stock trigger = throttle low + rudder held
  (< -200) + mode switch in a specific position, held past a timeout, second
  activation writes current attitude to flash as the level reference and
  reloads it. For us: capture the current orientation-filter attitude as the
  "true level" trim (kills the wandering hover trim after crashes/reassembly)
  into the same flash config page. Needs the Phase 6 flash page first.
- Both gestures must be inert in flight: require motor stopped + collective
  low + on-ground state, and blink the LED to acknowledge (ties into the
  Phase 6 LED language).

## 🗺 Cyclic control-law map (reference, 2026-07-17)

How the world builds cyclic stabilization, decoded/collected while rebuilding
mode 2. The load-bearing rule everything below agrees on: **nobody drives
servos straight from an angle PID** — an angle loop has no damping term that
isn't differentiated noise. Attitude control is always a CASCADE: angle error
-> commanded body rate -> the rate loop talks to the actuators. (Our one-day
direct angle-PID (2026-07-16) violated this and paid with a 6-8Hz limit cycle
and an antiphase "swash one way, machine pulled the other".)

**Rotorflight** (FBL-replacement lineage: BeastX/iKON/Spirit culture):
- Base mode with nothing enabled = **RATE (acro)** — and it IS the same law in
  hover and aerobatics. "Normal" vs "idle-up" is a TX-side concept: throttle /
  collective curves, governor headspeed profile — never the control law.
- Angle/Horizon exist as optional safety modes (Betaflight inheritance),
  Rescue as a bailout. Heli pilots mostly never engage them.
- Rate-loop garnish we ported: iterm relax, error decay, B-term, stop gain.

**ArduPilot tradheli** (autopilot lineage):
- Base assisted mode = **Stabilize = attitude**: stick POSITION = target lean
  angle, release = level. Acro is a separate rate mode most users never enter.
- Implementation: AC_AttitudeControl -> angle error x P (sqrt controller) ->
  target rate, PLUS the shaped target's own rate fed forward -> rate PID.
  Default ANG_RLL_P/ANG_PIT_P = 4.5 (we adopted it for STAB_ATT_ANGLE_P).

**Stock V977 firmware** (decompiled, Control_MainLoop read line-by-line):
- Attitude with a MOVING target: quaternion error, stick STEERS the target
  (gated steps in Control_AxisCorrection), `Math_QuaternionIdentity()` snaps
  the target level at throttle minimum AND on mode change.
- Per axis: `output = (GAIN x angle_err + stick) / (|stick|*6 + 900)` — raw
  stick feedforward IS in the mix, adaptive softening divisor, and **no I, no
  D anywhere on cyclic**. Their damping budget lives in the quaternion rate
  content and the airframe itself.
- The idle-up branch NEGATES the corrections under a flight flag — stock
  supported inverted flight at the control-law level.
- Feel consequence: an integrating target under a rate-feel stick. The pilot
  flew it for years convinced it was rate — distinguishable only by gust
  response, absence of drift, and the throttle-low level-snap (the very gesture
  in his muscle memory that betrayed a target's existence).

**Ours** (src/stabilize.h STAB_CYCLIC_RATE_MODE):
- 0 = self-level (stick fights a spring to horizon). Kept for reference.
- 1 = rate, Rotorflight-style, flight-proven. Keeps decay + softening.
- 2 = stock-attitude CASCADE (flight verdict 2026-07-17: **"работает
  приемлемо"** — the current baseline): stick steers the target (deadband
  20us; slew 0.3 -> ~93 deg/s full stick; one-sided leash 25 deg — stick
  cannot wind the target away, the airframe never drags it; clamp +-35;
  **capture-on-release** — on the deflected->released edge the target grabs
  the ACTUAL angle, so the bank stays where the machine is, no spring-back;
  throttle-low reset = stock's QuaternionIdentity). Outer: angle error x
  ANGLE_P(4.5), clamped 180dps, plus the stick's slew fed forward
  (ArduPilot-style). Inner: mode 1's rate PID VERBATIM (shared code), ground
  gate on its integral by COLLECTIVE (head speed is flat here — only blade
  pitch knows ground from hover), decay and softening excluded in mode 2.

**The plan this map serves**: normal = mode 2, idle-up = mode 1 — i.e.
ArduPilot Stabilize in the base, Rotorflight acro on the switch, matching how
both worlds actually fly. Stock's own idle-up inversion is precedent that even
the toy switched control behavior on that flag. Mode switch source: data[4]
(idle-up detector, pending). When building it, switch BOTH the collective curve
AND the cyclic mode 2->1; the shared inner loop makes that a setpoint-source
swap, not a controller swap.

## 📝 Critical Technical Notes
- **TIM2/TIM3 PWM**: system clock is 48MHz — PLL from the 16MHz HSE crystal
  since 2026-07-16 (PLL from HSI between 0c3b14e and then; the 8MHz era note
  that used to live here is obsolete). The frequency did not change with the
  HSE move, only its accuracy, so every µs constant below still holds. PSC=47 →
  1MHz timer tick, ARR=8000 → 125Hz, CCR written directly in microseconds.
  The stock firmware used the same PSC=47 and the same 48MHz off the same
  crystal.

- **Main motor signal path (decoded from stock dump 2026-07-15).** The stock
  firmware computed main throttle as a duty 0..1024 and emitted it in TWO
  formats in parallel, because one family firmware serves both board variants:
  1. Fast-PWM duty (clamp 1024 = TIM3 ARR; stock TIM3: PSC=2, ARR=0x400 →
     ~15.6kHz) via RAM shadows 0x20000040/42 — the drive for onboard brushed
     motor keys.
  2. The SAME duty converted `duty*1000/1024 + 1000` µs into **TIM2_CCR1 =
     PA0** — i.e. an ESC/servo-style 1000-2000µs signal on PPM channel-1,
     which also goes to the CH1 connector.
  Supporting evidence: IMU_CalibrateFull's literal pool holds exactly
  TIM3_CCR3 (PB0, tail key) + TIM2_CCR1 (PA0) — the two motor outputs it
  silences during calibration. Ghidra's "PWM_Tail_*" names are misleading;
  the µs path (TAIL_CENTER=1000/TAIL_MAX=2000 constants) is the main throttle.
  **Current state after the brushless conversion:** we drive the main ESC from
  PA0 (OUT_MAIN = TIM2_CCR1, 125Hz servo PWM) plugged into the PPM CH1
  connector — the same signal the stock firmware already emitted there. The
  old brushed-key branch (FET stage, now desoldered) is dead. TIM1 in stock
  was init'd as a timebase only (PSC=47, no CCER/outputs) — it never drove
  PA9-PA12, so those stay free-pin candidates.
  **Open check:** stock kept TWO fast-duty shadows (0x20000040[0] and [1]) —
  the second may have fed TIM3_CCR4 = PB1 as a separate main brushed key. The
  "PB0+PB1 joined on board" verdict rests only on a glitch observation from
  the test_step2 era. Now that the keys are desoldered, beep PB0↔PB1: if they
  do NOT ring, PB1 is a free hardware PWM pin (TIM3_CH4) — first candidate
  for the deferred tail-reverse bridge.
- **IMU polling must not starve the radio** (measured 2026-07-05 over SWD).
  The bit-bang I2C read is a blocking busy-wait: a full 14-byte accel+gyro
  burst costs ~4.8ms, a 6-byte gyro-only burst ~2.5ms. `i2c_delay` is sized to
  clear the weak internal pull-up's ~4us rise time and CANNOT be shortened
  without external pull-ups, so it stays at 4. If `IMU_PERIOD_US` <= the read
  cost, the read runs back-to-back every loop and `SFHSS_Poll` collapses to
  ~185 loops/s — the refresh regression. Fix (main.c): `IMU_PERIOD_US=10000`
  (100Hz) + gyro-only reads (`MPU6500_GetGyroSample`, 6B) every tick, full
  accel read only 1 tick in `IMU_ACCEL_EVERY`(8) (~12Hz — plenty for gravity
  drift-correction). Restored ~1234 loops/s (poll ~every 810us); common blind
  window is the 2.5ms gyro read, well under the 6.8ms S-FHSS frame interval.
  Don't lower `IMU_PERIOD_US` below ~2x the gyro read cost or read accel every
  tick, and don't re-enable a per-loop full 14-byte read.
- **TI2500 Init**: Requires a specific 40-byte register sequence to enable RX.
- **Boot Mode**: BOOT0 must be LOW for Flash boot.
- **Ghidra function names in `heli_ghidra_output/` are not reliable pin/role
  labels** — two separate instances now (CC2500 pinout, and the IMU
  "Init_Configure" function actually being the CC2500 config table) turned
  out to be misattributed. Trust register-level protocol semantics (address +
  value pairs that match a real datasheet) from that dump; verify anything
  about *which pin* physically carries a signal against real hardware over
  SWD, the same way PARTNUM/VERSION were confirmed for the radio.
- **OpenOCD `reset halt` can time out on this board** even with known-good
  firmware — seen 2026-07-04 after ST-Link was unplugged/replugged and the
  board was continuity-tested with a multimeter. Symptom: "timed out while
  waiting for target halted", reproducible across multiple attempts and even
  against a previously-verified-working image, with the target landing in
  `Handler HardFault` at a different RAM address each time (consistent with
  transient electrical instability, not a code bug). Workaround: `halt` alone
  (without a preceding `reset`) connects fine; flash with `halt` +
  `flash write_image erase` + `verify_image` + `reset run` instead of
  `reset halt`. In this case the instability was transient and cleared after
  reseating/power-cycling — if it recurs, suspect the same rather than
  re-bisecting firmware.
