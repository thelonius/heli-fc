#!/bin/sh
# TAIL WORKING-POINT probe readout (2026-07-19). Run AFTER landing with the
# power still ON — the counters live in RAM and cover the whole power-up.
#
#   tools/tail_probe.sh [firmware.elf]
#
# Chasing: "tail oscillations appear as the battery discharges, not on a fresh
# pack". Fly a FRESH pack, land, read; fly the SAME pack down, land, read.
# Compare:
#   ema rises a lot + sat_ms > 0  -> COMMAND ceiling (loop asking past 2000us);
#                                    gain scheduling on the working point fits
#   ema rises + sat_ms == 0       -> THRUST limited (full command, motor cannot
#                                    deliver at that voltage); only lower gain
#   ema flat                      -> sag is NOT moving the tail's operating
#                                    point; look elsewhere (rotor RPM droop)
set -e
DIR=$(cd "$(dirname "$0")" && pwd)
ELF=${1:-$DIR/../src/firmware.elf}

addr() { arm-none-eabi-nm "$ELF" | awk -v s="$1" '$3==s{print "0x"$1}'; }
EMA=$(addr g_tail_ema_us); MAX=$(addr g_tail_max_us); SAT=$(addr g_tail_sat_ms)
if [ -z "$EMA" ]; then
    echo "g_tail_* not in $ELF — wrong build?" >&2
    exit 1
fi

OUT=$(openocd -f interface/stlink.cfg -f target/stm32f0x.cfg \
    -c "init" \
    -c "echo P=[lindex [read_memory $EMA 32 1] 0],[lindex [read_memory $MAX 32 1] 0],[lindex [read_memory $SAT 32 1] 0]" \
    -c "shutdown" 2>&1)

echo "$OUT" | grep -q '^P=' || { echo "$OUT" >&2; exit 1; }

echo "$OUT" | grep '^P=' | python3 -c '
import struct, sys, re
m = re.search(r"P=(\S+),(\S+),(\S+)", sys.stdin.read())
f = lambda h: struct.unpack("<f", struct.pack("<I", int(h, 0)))[0]
ema, mx, sat = f(m.group(1)), f(m.group(2)), int(m.group(3), 0)
print(f"tail EMA  = {ema:7.1f} us   (hover working point; 2000 = rail)")
print(f"tail max  = {mx:7.1f} us   (>2000 means the loop asked past the rail)")
print(f"saturated = {sat:7d} ms   (time pinned at the rail)")
print()
head = 2000.0 - ema
print(f"headroom above the working point: {head:.0f} us of {1000.0:.0f} total span")
if sat > 0:
    print("-> COMMAND ceiling reached: the loop wanted more than the rail allows.")
elif ema > 1600:
    print("-> working point HIGH but never railed: likely thrust-limited by voltage.")
else:
    print("-> working point still low; sag is not pushing the tail toward its rail.")
'
