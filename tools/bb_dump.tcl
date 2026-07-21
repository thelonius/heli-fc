# Stab-blackbox reader — invoked by bb_dump.sh, which sets BB_LOG/BB_HEAD from
# the ELF. Record layout must match StabBBRec_t in src/stabilize.h (20 bytes =
# 10 halfwords):
#   [0] err_dps10 i16   [1] yaw_i_x100 i16   [2] tail_us u16
#   [3] roll_deg100 i16 [4] pitch_deg100 i16
#   [5] roll_tgt100 i16 [6] pitch_tgt100 i16
#   [7] roll_corr10 i16 [8] pitch_corr10 i16
#   [9] low byte = coll200, high byte = steer (bit0 roll, bit1 pitch)
# yaw_corr_i_us assumes STAB_YAW_KI = 2.0 (the standing tail us the integral
# holds). Cyclic columns are the drift-diagnosis payload: with the stick
# centred (steer=0), a real *_deg with ~0 *_corr = estimate bias; a real *_deg
# with a steady nonzero *_corr = mechanical trim the loop is fighting.
init

set N 112
set W 10   ;# halfwords per record

proc s16 {v} { return [expr {$v > 32767 ? $v - 65536 : $v}] }

# FORMAT TRIPWIRE (added 2026-07-19 after a wrong-firmware read printed a full
# CSV of garbage): the flashed image must carry the exact layout sentinel this
# reader expects. A mismatch means the board runs a different/older build than
# the ELF — abort rather than decode nonsense.
set fmt [lindex [read_memory $BB_FMT 32 1] 0]
if {$fmt != $BB_FMT_EXPECT} {
    echo [format "FORMAT MISMATCH: board has 0x%08x, reader expects 0x%08x" \
            $fmt $BB_FMT_EXPECT]
    echo "  -> the flashed firmware is NOT this ELF's build. Re-flash, or use"
    echo "     the matching ELF. NOT decoding (would be garbage)."
    shutdown
}

set head [lindex [read_memory $BB_HEAD 16 1] 0]
if {$head == 0} {
    echo "blackbox EMPTY (stab never engaged since power-up)"
    shutdown
}
set cnt   [expr {$head < $N ? $head : $N}]
set start [expr {$head < $N ? 0 : $head % $N}]
echo [format "records=%d (head=%d%s)" $cnt $head \
        [expr {$head >= $N ? ", ring wrapped: oldest overwritten" : ""}]]

set raw [read_memory $BB_LOG 16 [expr {$N * $W}]]

echo "t_s,roll_deg,roll_tgt,roll_corr,pitch_deg,pitch_tgt,pitch_corr,steer,coll,yaw_err,yaw_i_us,tail_us"
for {set k 0} {$k < $cnt} {incr k} {
    set i [expr {($start + $k) % $N}]
    set b [expr {$i * $W}]
    set err  [s16 [lindex $raw [expr {$b+0}]]]
    set yi   [s16 [lindex $raw [expr {$b+1}]]]
    set tl   [lindex $raw [expr {$b+2}]]
    set rd   [s16 [lindex $raw [expr {$b+3}]]]
    set pd   [s16 [lindex $raw [expr {$b+4}]]]
    set rt   [s16 [lindex $raw [expr {$b+5}]]]
    set pt   [s16 [lindex $raw [expr {$b+6}]]]
    set rc   [s16 [lindex $raw [expr {$b+7}]]]
    set pc   [s16 [lindex $raw [expr {$b+8}]]]
    set last [lindex $raw [expr {$b+9}]]
    set coll [expr {$last & 0xff}]
    set str  [expr {($last >> 8) & 0xff}]
    echo [format "%.2f,%.2f,%.2f,%.1f,%.2f,%.2f,%.1f,%d,%.3f,%.1f,%.1f,%d" \
            [expr {$k * 0.25}] \
            [expr {$rd/100.0}] [expr {$rt/100.0}] [expr {$rc/10.0}] \
            [expr {$pd/100.0}] [expr {$pt/100.0}] [expr {$pc/10.0}] \
            $str [expr {$coll/200.0}] \
            [expr {$err/10.0}] [expr {$yi/100.0*2.0}] $tl]
}
shutdown
