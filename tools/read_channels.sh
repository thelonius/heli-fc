#!/bin/bash
# Dump all 8 S-FHSS channels (hex usec, center 1520=0x5F0) for the idle-up
# mode-switch hunt. data[] starts at 0x2000001a (re-derived 2026-07-07 from
# DWARF — the first version read from 0x16 and was 4 bytes off):
#   [0]roll [1]pitch [2]throttle [3]rudder [4]aux [5]collective [6]? [7]?
# Run once per TX mode/switch position and diff which channel moved.
openocd -f interface/stlink.cfg -f target/stm32f0x.cfg \
 -c "init" \
 -c "echo {--- data[0..7]: roll pitch THR rudder aux COLL ?6 ?7 (hex us) ---}" \
 -c "mdh 0x2000001a 8" \
 -c "shutdown"
