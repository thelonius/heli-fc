#!/bin/sh
# YAW BLACKBOX dump (2026-07-17). Run AFTER landing, board still on the SAME
# power-up — the log lives in RAM and one power cycle erases it.
#
#   tools/bb_dump.sh [firmware.elf]
#
# Addresses are pulled from the ELF with nm, so symbol drift across rebuilds
# is handled — BUT the ELF must be the exact build that is flashed on the
# board (same rule as verify_image). Default: src/firmware.elf next to this
# repo's tools/.
#
# Output: CSV, oldest record first, one line per 0.25s of STAB-ENGAGED time
# (the recorder pauses at throttle idle, so idle gaps are invisible in t_s).
set -e
DIR=$(cd "$(dirname "$0")" && pwd)
ELF=${1:-$DIR/../src/firmware.elf}

BB_LOG=$(arm-none-eabi-nm "$ELF" | awk '$3=="g_bb_log"{print "0x"$1}')
BB_HEAD=$(arm-none-eabi-nm "$ELF" | awk '$3=="g_bb_head"{print "0x"$1}')
BB_FMT=$(arm-none-eabi-nm "$ELF" | awk '$3=="g_bb_fmt"{print "0x"$1}')
if [ -z "$BB_LOG" ] || [ -z "$BB_HEAD" ] || [ -z "$BB_FMT" ]; then
    echo "g_bb_log/g_bb_head/g_bb_fmt not in $ELF — built with STAB_BLACKBOX=0?" >&2
    exit 1
fi

# Expected sentinel from the header — must match STAB_BB_FMT. Kept here (not
# derived) so a stale ELF that still defines the symbol can't self-approve.
BB_FMT_EXPECT=0xBB000003

openocd -f interface/stlink.cfg -f target/stm32f0x.cfg \
    -c "set BB_LOG $BB_LOG" -c "set BB_HEAD $BB_HEAD" \
    -c "set BB_FMT $BB_FMT" -c "set BB_FMT_EXPECT $BB_FMT_EXPECT" \
    -f "$DIR/bb_dump.tcl"
