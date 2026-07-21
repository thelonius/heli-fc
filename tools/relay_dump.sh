#!/bin/sh
# TAIL RELAY AUTOTUNE — trigger and/or dump (2026-07-17). Needs the bench
# build: TAIL_RELAY_TEST=1 in main.c, STAB_BLACKBOX=0 in stabilize.h. Main
# motor is forced off by the test itself; the heli must be free to yaw (hang
# it by the rotor head). ST-Link survives because the main rotor never spins.
#
#   tools/relay_dump.sh --run [firmware.elf]   # start the experiment, wait
#                                              # ~7s for it to finish, dump
#   tools/relay_dump.sh [firmware.elf]         # dump only (already ran)
#
# Then:  tools/relay_dump.sh --run | tools/relay_fit.py
# The ELF must be the exact flashed build (addresses come from nm).
set -e
DIR=$(cd "$(dirname "$0")" && pwd)
RUN=0
if [ "$1" = "--run" ]; then RUN=1; shift; fi
ELF=${1:-$DIR/../src/firmware.elf}

addr() { arm-none-eabi-nm "$ELF" | awk -v s="$1" '$3==s{print "0x"$1}'; }
GO=$(addr g_relay_go); N=$(addr g_relay_n)
RATE=$(addr g_relay_rate); HI=$(addr g_relay_hi); MEAN=$(addr g_relay_mean)
if [ -z "$GO" ]; then
    echo "g_relay_* not in $ELF — built with TAIL_RELAY_TEST=0?" >&2
    exit 1
fi

openocd -f interface/stlink.cfg -f target/stm32f0x.cfg \
    -c "set RUN $RUN" -c "set RELAY_GO $GO" -c "set RELAY_CNT $N" \
    -c "set RELAY_RATE $RATE" -c "set RELAY_HI $HI" -c "set RELAY_MEAN $MEAN" \
    -f "$DIR/relay_dump.tcl"
