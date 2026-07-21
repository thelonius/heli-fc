# POST-FLIGHT readout for the failsafe-drop discriminator (2026-07-17).
#
# Run AFTER the flight, board still on the SAME power-up (power was never
# cycled since preflight.tcl):
#   openocd -f interface/stlink.cfg -f target/stm32f0x.cfg -f postflight.tcl
#
# Interpretation:
#   IWDG flag set                     -> watchdog bit: the loop stalled >~400ms
#   POR/PDR flag set                  -> BROWNOUT in flight: power sagged low
#                                        enough to reset the MCU (fresh-pack /
#                                        BEC suspect). Counters also restart.
#   no reset flags, statResync > 0    -> radio-side RESYNC episode(s): 64 blind
#                                        hops (~435ms) + re-acquire (<=204ms) —
#                                        each one brushes the 500ms failsafe.
#   no reset flags, statResync == 0   -> no full desync; if the pilot still saw
#                                        failsafe, statLost should show a long
#                                        continuous run — look deeper.
#   statRcv << expected (~147/s of    -> the MCU REBOOTED mid-flight even if
#   flight time)                         flags look clean (POR wipes flags of
#                                        its own era on deep dips).
#
# Addresses: build flashed 2026-07-16 (verified). g_clk_hse/g_mpu_ok drift on
# rebuild — re-derive with arm-none-eabi-nm.

init

proc rd8  {addr} { return [expr {[lindex [read_memory $addr 8  1] 0] & 0xff}] }
proc rd32 {addr} { return [expr {[lindex [read_memory $addr 32 1] 0] & 0xffffffff}] }

set csr [rd32 0x40021024]
echo [format "RCC_CSR = 0x%08x" $csr]
foreach {bit name} {31 LPWR 30 WWDG 29 IWDG 28 SFT 27 POR/PDR 26 PIN 25 OBL 23 V18} {
    if {$csr & (1 << $bit)} { echo "  reset flag: $name" }
}
if {($csr & 0xfe800000) == 0} { echo "  reset flags: NONE (no MCU reset during flight)" }

echo [format "g_clk_hse=%d g_mpu_ok=%d phase=%d (3=CONNECTED)" \
        [rd8 0x20000160] [rd8 0x2000018e] [rd8 0x20000000]]
echo [format "statRcv=%d statLost=%d statResync=%d maxPollGap=%dus" \
        [rd32 0x2000004c] [rd32 0x20000050] [rd32 0x20000144] [rd32 0x20000148]]
echo "expected statRcv ~= 147 * seconds-since-preflight (TX on). Much less = reboot."
echo "maxPollGap ~3000us is the known norm; a big jump = the loop stalled."

shutdown
