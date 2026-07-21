# Frame-loss monitor for the S-FHSS receiver (flywheel-era).
# Reads the counters twice over a window, in ONE openocd session (per memory:
# reconnect churn corrupts the read), prints delta rates + loss. TX MUST be ON
# for the whole window or nRcv won't grow and the ratio is meaningless.
#
# Semantics since the 2026-07-12 flywheel rewrite:
#   statRcv   slots with >=1 valid frame (counted once per slot)
#   statLost  slots with none (grid hop fired blind)
#   cntEven   DATA1 packets (ch1-4), per packet
#   cntOdd    DATA2 packets (ch5-8), per packet
#   statResync  desync episodes (fell back to rebind) — each one was a freeze
#   maxPollGap  worst gap between SFHSS_Poll entries, us (late-read anomaly)
#
# Field addresses (g_sfhss @ 0x20000000, offsets DWARF-derived 2026-07-12):
#   statRcv  0x2000004c   statLost   0x20000050
#   cntEven  0x20000084   cntOdd     0x20000088
#   statResync 0x20000144 maxPollGap 0x20000148
#
# Usage:  openocd -f interface/stlink.cfg -f target/stm32f0x.cfg -f mon_rate.tcl
#         (optionally set WIN_MS first:  -c "set WIN_MS 8000")

init

if {![info exists WIN_MS]} { set WIN_MS 5000 }

proc rd {addr} { return [expr {[lindex [read_memory $addr 32 1] 0] & 0xffffffff}] }

set rcv0  [rd 0x2000004c]
set lost0 [rd 0x20000050]
set even0 [rd 0x20000084]
set odd0  [rd 0x20000088]
set rsy0  [rd 0x20000144]

echo "--- sampling for ${WIN_MS} ms (keep TX ON) ---"
sleep $WIN_MS

set rcv1  [rd 0x2000004c]
set lost1 [rd 0x20000050]
set even1 [rd 0x20000084]
set odd1  [rd 0x20000088]
set rsy1  [rd 0x20000144]
set gap   [rd 0x20000148]

set dRcv  [expr {$rcv1  - $rcv0}]
set dLost [expr {$lost1 - $lost0}]
set dEven [expr {$even1 - $even0}]
set dOdd  [expr {$odd1  - $odd0}]
set dRsy  [expr {$rsy1  - $rsy0}]
set win_s [expr {$WIN_MS / 1000.0}]
set total [expr {$dRcv + $dLost}]

echo [format "slots: rcv=%d  lost=%d  total=%d" $dRcv $dLost $total]
if {$total > 0} {
    echo [format "slot loss = %.1f %%" [expr {100.0 * $dLost / $total}]]
}
echo [format "packets: DATA1=%.1f/s  DATA2(ch5-8)=%.1f/s" \
        [expr {$dEven / $win_s}] [expr {$dOdd / $win_s}]]
echo [format "resyncs in window = %d (each ~= one freeze)" $dRsy]
echo [format "maxPollGap = %d us (reset: mww 0x20000148 0)" $gap]

shutdown
