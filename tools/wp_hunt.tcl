# Watchpoint hunt for the g_sfhss corruption (2026-07-17).
# Arms a DWT write-watchpoint on g_sfhss.txaddr (0x20000004) AFTER the link is
# up (the only legit write happens once, at first acquisition), then waits.
# When the rogue store fires, the core halts and pc/lr name the culprit.
# Board must have the TX on so the link connects during the settle window.

init

proc rd32 {addr} { return [expr {[lindex [read_memory $addr 32 1] 0] & 0xffffffff}] }

reset run
sleep 8000

set t [rd32 0x20000004]
echo [format "txaddr after settle: 0x%08x (expect 0x6d0d = connected)" $t]
echo [format "phase=%d statRcv=%d" \
    [expr {[lindex [read_memory 0x20000000 8 1] 0] & 0xff}] [rd32 0x2000004c]]

halt
wp 0x20000004 4 w
resume
echo "--- watchpoint armed on 0x20000004 (txaddr), waiting up to 10 min ---"

if {[catch { wait_halt 600000 } err]} {
    echo "wait_halt ended without halt: $err"
    echo [format "txaddr now: 0x%08x" [rd32 0x20000004]]
    shutdown
}

echo "--- HALTED: rogue write caught ---"
reg pc
reg lr
reg sp
reg
echo "g_sfhss head after write:"
mdb 0x20000000 16
echo [format "statRcv=%d statLost=%d" [rd32 0x2000004c] [rd32 0x20000050]]

shutdown
