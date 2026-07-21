# Live loss monitor (2026-07-17): prints every statLost increment with context —
# did attempts tick during the gap (garbage vs silence), RSSI, skipCount.
# Usage: openocd -f interface/stlink.cfg -f target/stm32f0x.cfg -f mon_loss.tcl
# Addresses: g_sfhss @ 0x20000000, build 1cf9bde (offsets stable per sfhss.h).

init

if {![info exists WIN_MS]} { set WIN_MS 30000 }
set STEP_MS 100

proc rd8  {addr} { return [expr {[lindex [read_memory $addr 8  1] 0] & 0xff}] }
proc rd16 {addr} { return [expr {[lindex [read_memory $addr 16 1] 0] & 0xffff}] }
proc rd32 {addr} { return [expr {[lindex [read_memory $addr 32 1] 0] & 0xffffffff}] }

set rcv0  [rd32 0x2000004c]
set lost0 [rd32 0x20000050]
set att0  [rd32 0x20000060]
set mrk0  [rd32 0x20000064]
set crc0  [rd32 0x20000068]
set rsy0  [rd32 0x20000144]

set pl $lost0
set pa $att0
set pm $mrk0
set pc $crc0
set t 0
set maxskip 0

echo "--- monitoring ${WIN_MS} ms ---"
while {$t < $WIN_MS} {
    set lost [rd32 0x20000050]
    set att  [rd32 0x20000060]
    set mrk  [rd32 0x20000064]
    set crc  [rd32 0x20000068]
    set skip [rd16 0x20000018]
    set rssi [rd8  0x2000002a]
    set r [expr {$rssi > 127 ? $rssi - 256 : $rssi}]
    set dbm [expr {$r / 2 - 71}]
    if {$skip > $maxskip} { set maxskip $skip }

    if {$lost != $pl} {
        echo [format "%6d ms  +%d lost (skip=%d rssi=%d dBm)  d_att=%d d_noMark=%d d_noCrc=%d" \
            $t [expr {$lost - $pl}] $skip $dbm \
            [expr {$att - $pa}] [expr {$mrk - $pm}] [expr {$crc - $pc}]]
    }
    set pl $lost; set pa $att; set pm $mrk; set pc $crc
    sleep $STEP_MS
    set t [expr {$t + $STEP_MS}]
}

echo "--- window totals ---"
echo [format "rcv +%d  lost +%d  attempts +%d  noMarker +%d  noCrc +%d  resync +%d  maxSkip %d" \
    [expr {[rd32 0x2000004c] - $rcv0}] [expr {[rd32 0x20000050] - $lost0}] \
    [expr {[rd32 0x20000060] - $att0}] [expr {[rd32 0x20000064] - $mrk0}] \
    [expr {[rd32 0x20000068] - $crc0}] [expr {[rd32 0x20000144] - $rsy0}] $maxskip]

# ring of the last 16 valid-frame gaps: delta us | cmd | hop ch (newest = idx-1)
set ri [rd8 0x200000ec]
echo [format "--- ring (idx=%d, oldest->newest) ---" $ri]
for {set i 0} {$i < 16} {incr i} {
    set k [expr {($ri + $i) & 15}]
    echo [format "  d=%7d us  cmd=0x%02x  ch=%2d" \
        [rd32 [expr {0x2000008c + 4*$k}]] [rd8 [expr {0x200000cc + $k}]] [rd8 [expr {0x200000dc + $k}]]]
}

shutdown
