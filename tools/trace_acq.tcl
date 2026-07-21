# Acquisition tracer: watches the S-FHSS phase machine while the link comes up.
#
# Motivates: pilot reports several quick drop/recover cycles at acquisition
# before the link settles (2026-07-16). Phase leaves CONNECTED only via
# skipCount >= FALLBACK_COUNT (64 blind hops ~= 435ms), so every LED flicker is
# a real statResync episode — this script proves how many and what precedes them.
#
# Field addresses (g_sfhss @ 0x20000000; -fshort-enums => phase is 1 byte):
#   phase 0x20000000 (0=START_BIND 1=FINDING 2=BINDED 3=CONNECTED — see sfhss.h)
#   skipCount 0x20000018 (u16)   rssi 0x2000002a (u8)
#   statRcv 0x2000004c  statLost 0x20000050  statResync 0x20000144
#
# Usage:  openocd -f interface/stlink.cfg -f target/stm32f0x.cfg -f trace_acq.tcl
#         (set WIN_MS to change the 25s default)
# Reproduce the event INSIDE the window: TX off -> wait -> TX on.

init

if {![info exists WIN_MS]} { set WIN_MS 25000 }
set STEP_MS 100

proc rd8  {addr} { return [expr {[lindex [read_memory $addr 8  1] 0] & 0xff}] }
proc rd16 {addr} { return [expr {[lindex [read_memory $addr 16 1] 0] & 0xffff}] }
proc rd32 {addr} { return [expr {[lindex [read_memory $addr 32 1] 0] & 0xffffffff}] }

proc phname {p} {
    switch $p {
        0 { return "START_BIND" }
        1 { return "FINDING" }
        2 { return "BINDED" }
        3 { return "CONNECTED" }
        default { return "?$p" }
    }
}

echo "--- tracing ${WIN_MS} ms. Turn the TX OFF, wait ~2s, then ON. ---"

set prevPhase -1
set rsy0 [rd32 0x20000144]
set prevRsy $rsy0
set maxSkip 0
set t 0

while {$t < $WIN_MS} {
    set ph   [rd8  0x20000000]
    set skip [rd16 0x20000018]
    set rsy  [rd32 0x20000144]
    set rssi [rd8  0x2000002a]

    # CC2500 raw RSSI byte -> dBm (datasheet: offset 71, /2, two's complement)
    set r [expr {$rssi > 127 ? $rssi - 256 : $rssi}]
    set dbm [expr {$r / 2 - 71}]

    if {$skip > $maxSkip} { set maxSkip $skip }

    if {$ph != $prevPhase} {
        echo [format "%6d ms  phase -> %-10s  skip=%-3d rssi=%d dBm" \
                $t [phname $ph] $skip $dbm]
        set prevPhase $ph
    }
    if {$rsy != $prevRsy} {
        echo [format "%6d ms  *** RESYNC #%d (64 blind hops ~= 435ms of silence)" \
                $t [expr {$rsy - $rsy0}]]
        set prevRsy $rsy
    }

    sleep $STEP_MS
    set t [expr {$t + $STEP_MS}]
}

echo "--- done ---"
echo [format "resyncs during window = %d" [expr {[rd32 0x20000144] - $rsy0}]]
echo [format "max skipCount seen    = %d  (fallback fires at 64)" $maxSkip]
echo [format "slots rcv=%d lost=%d" [rd32 0x2000004c] [rd32 0x20000050]]

shutdown
