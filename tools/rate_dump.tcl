# High-rate gyro log reader — invoked by rate_dump.sh. Layout per RateRec_t
# in main.c: {int16 roll, int16 pitch, int16 yaw} dps x10, 50Hz, ring of 373.
init

set N 373
set W 3   ;# halfwords per record

proc s16 {v} { return [expr {$v > 32767 ? $v - 65536 : $v}] }

set fmt [lindex [read_memory $RL_FMT 32 1] 0]
if {$fmt != $RL_FMT_EXPECT} {
    echo [format "FORMAT MISMATCH: board 0x%08x, expected 0x%08x — wrong build flashed, NOT decoding" \
            $fmt $RL_FMT_EXPECT]
    shutdown
}

set head [lindex [read_memory $RL_HEAD 16 1] 0]
if {$head == 0} {
    echo "rate log EMPTY (never engaged since power-up)"
    shutdown
}
set cnt   [expr {$head < $N ? $head : $N}]
set start [expr {$head < $N ? 0 : $head % $N}]
echo [format "# records=%d (head=%d%s), 50Hz" $cnt $head \
        [expr {$head >= $N ? ", wrapped" : ""}]]

set raw [read_memory $RL_LOG 16 [expr {$N * $W}]]

echo "t_s,roll_dps,pitch_dps,yaw_dps"
for {set k 0} {$k < $cnt} {incr k} {
    set i [expr {($start + $k) % $N}]
    set b [expr {$i * $W}]
    echo [format "%.2f,%.1f,%.1f,%.1f" [expr {$k * 0.02}] \
            [expr {[s16 [lindex $raw $b]] / 10.0}] \
            [expr {[s16 [lindex $raw [expr {$b+1}]]] / 10.0}] \
            [expr {[s16 [lindex $raw [expr {$b+2}]]] / 10.0}]]
}
shutdown
