# S-FHSS sync ritual — design (2026-07-23)

Status: DESIGN, nothing implemented. Grounded in the 2026-07-23 bisect
(`logs/2026-07-23_connect_freeze/`): the freeze/resync storms entered with
the flywheel commit 8de6f3f; 6afbba3 and earlier never stormed. HEAD keeps
the flywheel (0.1% steady loss vs 5-6% pre-flywheel) — the fix is to make
its acquisition and recovery trustworthy, not to revert it.

## Why the flywheel storms

Three independent defects, all observed on the bench the same day:

1. **One-frame anchor.** Every received frame hard-re-anchors the grid:
   `slotStart = rtime` (±DATA2 correction). `rtime` is the *drain* time, and
   the main loop polls with gaps up to ~3.1ms (`maxPollGap`, IMU bursts). One
   late drain teleports the whole grid milliseconds late; every subsequent
   blind hop then tunes after the TX has already transmitted. Pure silence,
   no bad packets — exactly what the loggers recorded.
2. **Hardcoded interval.** `interval = 6801us`, never measured against the
   TX. Any real mismatch accumulates linearly over blind slots. (At the
   flywheel's birth the MCU still ran on HSI ±1% — 68us of drift per blind
   slot, storms guaranteed. HSE cut the clock error but the assumption
   "our microsecond == TX microsecond" is still unverified per-TX.)
3. **Flat 64-slot death march.** After a miss the receiver blind-hops up to
   64 slots (~435ms) on the possibly-poisoned grid before falling back —
   just short of the 500ms failsafe. Storm waves = march, resync, brief
   catch, march again. Losses arrive in quanta of 64 in every log.

## The ritual

### Phase ladder

```
FINDING (camp ch0)
   └─ first valid frame ──► SYNC  (reactive, learn the TX)
                              └─ interval locked ──► CONNECTED (flywheel)
                                                        ├─ miss ladder (below)
                                                        └─ sustained loss ──► BINDED/camp
```

> **S2 outcome (2026-07-23, measured on the bench).** Implemented as a
> diagnostic and it settled the question: this TX's real slot period is
> **6803us**, so the hardcoded 6801 is accurate to ~2us — better than a live
> measurement achieves (the combined-drain `cand` route biased the median ~24us
> low; the clean DATA1-only route is accurate but its prompt reads are too
> scarce to lock reliably). Since the bisect already pinned the freezes on the
> anchor teleport (S1), not on interval error, S2 does **NOT** feed `interval`:
> operation keeps the trusted constant, and `intervalMeas` is exposed over SWD
> only as a drift / foreign-TX watch. The reactive-SYNC-phase machinery below
> was therefore dropped — S1's anchor gate already removes the storm risk that
> a separate reactive phase was meant to avoid, and no measured interval is
> worth switching phases for. S3 (a slow in-CONNECTED EMA) remains the path if
> real TX drift ever shows up in `intervalMeas`.

### SYNC — learn before you freewheel (NOT SHIPPED — see S2 outcome above)

Reactive hopping exactly like the pre-flywheel era (hop on frame, timeout
per slot, no grid): tolerable 5% loss for ~0.2s, zero storm risk.

Collect DATA1-to-DATA1 deltas of *adjacent* slots (gap == 1). Accept a pair
only if both reads were prompt: `delta` within [6600, 7000]us AND the poll
gap around each read < 300us (poll cadence is already instrumented). Take
the **median of 16** accepted deltas → `intervalMeas`. Median kills the
late-drain outliers; 16 samples ≈ 20-25 slots ≈ 150-170ms of SYNC.

Lock `interval = intervalMeas` and enter CONNECTED. Keep `intervalMeas`
across resyncs — it is a property of this TX, not of this acquisition; on
re-entry SYNC only needs 4 confirming pairs (fast path, ~30ms) unless the
median disagrees by >8us, which re-triggers the full 16.

### CONNECTED — gated anchors, slow trim

Prediction first: the grid already knows when DATA1 should arrive. Compare
each received frame against it: `err = rtime - predictedSlotStart`.

- `|err| < 300us` (prompt read): full re-anchor, as today, and feed a slow
  EMA that trims `interval` by fractions of a us (tracks TX crystal drift
  with temperature).
- `300us <= |err| < 2500us` (late drain — the IMU-burst case): **use the
  data, don't move the clock.** Channels update; grid gets nudged by err/8
  only. A 3ms-late poll can no longer teleport the grid.
- `|err| >= 2500us`: data still used, grid untouched, `statAnchorRej++`.

### Miss ladder — fail fast, fall soft

Replaces the flat 64:

- **miss 1..7**: blind-hop on the grid, as today. Covers ordinary fades.
- **miss 8..23**: *hold* — stop hopping, park on the channel the grid
  predicts for 2 slots ahead, receiver stationary while the TX cycles
  toward it (≤204ms). A grid that is only slightly off still intersects
  the TX here; first frame re-anchors (gated) and resumes. Cost of being
  wrong: nothing — we were deaf anyway.
- **miss 24 (~163ms)**: full fallback to BINDED/camp ch0 + `statResync++`.
  Recovery budget stays well inside the 500ms failsafe window instead of
  brushing against it.

### TX-appears-while-running (the proven storm trigger)

After latched signal loss, the first frame does NOT jump straight to
CONNECTED with a one-frame anchor (today's behavior, storms every time the
TX is switched on). It re-enters SYNC fast path: 4 confirming pairs,
~30ms, then freewheel. Boot and TX-on become the same ritual.

## What this predicts (falsifiable on the bench)

- TX-on cycles: no resync storms; link settles < 0.5s. (Today: rsy 2→19.)
- Board power cycles: same; the init-race fix already covers the deaf boots.
- Steady loss stays at flywheel levels (~0.1%), because CONNECTED logic is
  unchanged for prompt frames.
- `statAnchorRej` > 0 correlating with IMU bursts will directly show the
  poison that used to teleport the grid.
- Type-A episodes (only hop 0-3 decode, CRC noise on them): if they were
  march/camp artifacts they disappear; if they persist, they are a real
  RF phenomenon and get hunted separately with rcvByCh/FREQEST already in.

## Implementation order — one flight variable at a time

1. **Stage S1 — anchor gating + miss ladder.** No new state machine, ~30
   lines in `SFHSS_Poll`. Highest confidence/effort ratio; kills the
   teleport and shortens the march. Bench: TX-cycle protocol, compare rsy.
2. **Stage S2 — SYNC phase with measured interval.** New phase + median
   machinery. Bench: same protocol + read `intervalMeas` (how far from
   6801 is this TX really?).
3. **Stage S3 — slow interval EMA in CONNECTED.** Only if S2 shows the TX
   meaningfully off-nominal or drifting.

Diagnostics to carry through all stages: `statAnchorRej`, `intervalMeas`,
`statHold` (ladder stage-2 entries), existing rcvByCh/ringDelta/FREQEST.
