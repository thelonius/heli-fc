#!/bin/sh
# HIGH-RATE GYRO LOG dump (2026-07-19, the ~4Hz oscillation hunt). Run AFTER
# landing, power NOT cycled. Needs the RATE_LOG=1 build (main.c); the format
# sentinel refuses a mismatched board. Ring = last ~7.5s of engaged time,
# 3 body rates @50Hz.
#
#   tools/rate_dump.sh [firmware.elf]        # CSV to stdout
#   tools/rate_dump.sh | tools/rate_fft.py   # straight to spectrum
set -e
DIR=$(cd "$(dirname "$0")" && pwd)
ELF=${1:-$DIR/../src/firmware.elf}

addr() { arm-none-eabi-nm "$ELF" | awk -v s="$1" '$3==s{print "0x"$1}'; }
RL_LOG=$(addr g_rl_log); RL_HEAD=$(addr g_rl_head); RL_FMT=$(addr g_rl_fmt)
if [ -z "$RL_LOG" ] || [ -z "$RL_HEAD" ] || [ -z "$RL_FMT" ]; then
    echo "g_rl_* not in $ELF — built with RATE_LOG=0?" >&2
    exit 1
fi

openocd -f interface/stlink.cfg -f target/stm32f0x.cfg \
    -c "set RL_LOG $RL_LOG" -c "set RL_HEAD $RL_HEAD" \
    -c "set RL_FMT $RL_FMT" -c "set RL_FMT_EXPECT 0xBB000104" \
    -f "$DIR/rate_dump.tcl"
