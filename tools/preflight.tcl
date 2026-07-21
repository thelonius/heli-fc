# PRE-FLIGHT arm for the failsafe-drop discriminator (2026-07-17).
#
# Run BEFORE the flight, with the heli powered from its flight pack:
#   openocd -f interface/stlink.cfg -f target/stm32f0x.cfg -f preflight.tcl
#
# What it does:
#   1. Clears the RCC_CSR reset-cause flags (RMVF) so any flag present after
#      the flight was set DURING it.
#   2. Zeroes statRcv/statLost/statResync and maxPollGap so post-flight reads
#      are the flight window, not the whole power-on (established methodology:
#      cumulative counters lie across TX-off pauses).
#
# После посадки НЕ ВЫКЛЮЧАТЬ ПИТАНИЕ БОРТА — снять питание значит стереть
# и флаги, и счётчики. Сесть, газ в ноль, подключить ST-Link, запустить
# postflight.tcl.
#
# Addresses below are for the build flashed 2026-07-16 (verified against
# src/firmware.elf). g_sfhss offsets are stable; g_clk_hse/g_mpu_ok move on
# every rebuild — re-derive with: arm-none-eabi-nm firmware.elf | grep g_

init

proc rd32 {addr} { return [expr {[lindex [read_memory $addr 32 1] 0] & 0xffffffff}] }

set csr [rd32 0x40021024]
echo [format "RCC_CSR before clear: 0x%08x" $csr]
mww 0x40021024 [expr {$csr | 0x01000000}]   ;# RMVF: clear all reset flags
echo [format "RCC_CSR after  clear: 0x%08x" [rd32 0x40021024]]

echo [format "before zero: statRcv=%d statLost=%d statResync=%d maxPollGap=%d" \
        [rd32 0x2000004c] [rd32 0x20000050] [rd32 0x20000144] [rd32 0x20000148]]
mww 0x2000004c 0   ;# statRcv
mww 0x20000050 0   ;# statLost
mww 0x20000144 0   ;# statResync
mww 0x20000148 0   ;# maxPollGap
echo "counters zeroed."
echo "--- armed. Fly. After landing DO NOT power-cycle; run postflight.tcl ---"

shutdown
