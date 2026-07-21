#!/bin/bash
# Read just the left-stick channels for CP throttle/pitch curve calibration.
#   data[2] throttle   @ 0x2000001e   (reversed: min-stick 1930 .. max-stick 1125)
#   data[5] collective @ 0x20000024   (~1327 .. ~1725)
# Offsets re-derived 2026-07-07 from DWARF: data[] starts at +0x1a, NOT +0x16
# (the first version of this script was 4 bytes off and read roll/rudder).
# Hold the left stick at a fixed position, run this, note both values.
openocd -f interface/stlink.cfg -f target/stm32f0x.cfg \
 -c "init" \
 -c "echo {--- throttle data[2] / collective data[5] (hex usec) ---}" \
 -c "mdh 0x2000001e 1" \
 -c "mdh 0x20000024 1" \
 -c "shutdown"
