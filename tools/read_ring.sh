#!/bin/bash
# Scanner readout: dump the last-16-frames ring + parity counters.
# Pair with firmware_scanner.bin (SFHSS_PAIR_MODE=3): it parks on one channel
# and logs every valid frame it sees there. Run 3-4 times over ~30s with the
# TX on to learn the channel's true visit schedule.
#
# Offsets re-derived 2026-07-07 from firmware_scanner.elf DWARF (g_sfhss at
# 0x20000000, struct size 0xf8). NB: earlier scripts used data@0x16/stats@0x48
# — stale since lastFrameSystick (+4) was added.
#
# How to read the output:
#   ringIdx     next slot to be written; newest entry = (ringIdx-1) & 15
#   ringDelta   us since the PREVIOUS valid frame (u32 x16)
#   ringCmd     raw 6-bit cmd; bit0: 0=ch1-4 frame, 1=ch5-8 frame (u8 x16)
#   ringCh      hop index the frame arrived on (u8 x16; constant when parked)
# Expected signatures on a parked channel:
#   standard pair: deltas ~6801 / ~415000 alternating, cmd parity 0,1,0,1
#   trailer model: deltas ~13600 or ~15100 inside a visit, ~196000+ between
openocd -f interface/stlink.cfg -f target/stm32f0x.cfg \
 -c "init" \
 -c "echo {--- phase ch hopcode nextSlot (phase 3 = parked/scanning) ---}" \
 -c "mdb 0x20000000 4" \
 -c "echo {--- statRcv ---}"          -c "mdw 0x2000004c 1" \
 -c "echo {--- cntEven cntOdd ---}"   -c "mdw 0x20000084 2" \
 -c "echo {--- ringIdx ---}"          -c "mdb 0x200000ec 1" \
 -c "echo {--- ringDelta[16] us ---}" -c "mdw 0x2000008c 16" \
 -c "echo {--- ringCmd[16] ---}"      -c "mdb 0x200000cc 16" \
 -c "echo {--- ringCh[16] ---}"       -c "mdb 0x200000dc 16" \
 -c "echo {--- data[0..7] us ---}"    -c "mdh 0x2000001a 8" \
 -c "echo {--- oddData[0..3] us (last ch5-8 frame, failsafe-flagged included) ---}" \
 -c "mdh 0x200000f6 4" \
 -c "echo {--- oddCmd (bit0=odd bit2=failsafe) ---}" -c "mdb 0x200000fe 1" \
 -c "shutdown"
