# Tail relay-autotune trigger/reader — invoked by relay_dump.sh, which sets
# the RELAY_* addresses from the ELF. Layout must match main.c:
#   g_relay_rate: int16[400] gyro-Z dps x10 @100Hz;  g_relay_hi: uint8[400]
# Timing: 1.5s settle + 4s logged relay; --run waits 7s before reading.
init

set N 400

if {$RUN} {
    echo "starting relay experiment (1.5s settle + 4s relay)..."
    mww $RELAY_GO 1
    sleep 7000
}

set n [lindex [read_memory $RELAY_CNT 16 1] 0]
if {$n == 0} {
    echo "no samples — experiment never ran (trigger with --run)"
    shutdown
}
set go [lindex [read_memory $RELAY_GO 8 1] 0]
if {$go != 0} { echo [format "WARNING: still running (go=%d, n=%d)" $go $n] }

# g_relay_mean (float): decode IEEE754 by hand, tcl has no direct view.
set w [lindex [read_memory $RELAY_MEAN 32 1] 0]
set sign [expr {($w >> 31) ? -1.0 : 1.0}]
set ex   [expr {($w >> 23) & 0xff}]
set man  [expr {$w & 0x7fffff}]
if {$ex == 0} { set mean 0.0 } else {
    set mean [expr {$sign * (1.0 + $man / 8388608.0) * pow(2.0, $ex - 127)}]
}
echo [format "# n=%d drift_mean=%.1f dps (settle-phase pirouette rate)" $n $mean]

set rates [read_memory $RELAY_RATE 16 $n]
set his   [read_memory $RELAY_HI 8 $n]

echo "t_s,rate_dps,hi"
for {set i 0} {$i < $n} {incr i} {
    set r [lindex $rates $i]
    if {$r > 32767} {set r [expr {$r - 65536}]}
    echo [format "%.2f,%.1f,%d" [expr {$i * 0.01}] [expr {$r / 10.0}] \
            [lindex $his $i]]
}
shutdown
