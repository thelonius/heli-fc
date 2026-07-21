#!/bin/bash
# Build-artifact flashing and SWD helpers for the STM32F031 flight controller.
#
# Usage: ./tools/flash.sh [flash|verify|read-gpio|read-tim3|set-tim3]
#
# The image is resolved relative to THIS SCRIPT (../src), not the current
# directory, on purpose: taking "firmware.bin" from wherever you happen to
# stand is how a stale full-flash artifact once got written to a live aircraft.
# Override deliberately with FW_DIR=/path/to/dir if you really mean to.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FW_DIR="${FW_DIR:-$SCRIPT_DIR/../src}"
ELF="$FW_DIR/firmware.elf"
BIN="$FW_DIR/firmware.bin"

OOCD=(openocd -f interface/stlink.cfg -f target/stm32f0x.cfg)

check_image() {
    if [ ! -f "$BIN" ]; then
        echo "Error: $BIN not found. Run 'make' in src/ first." >&2
        exit 1
    fi
    local size
    size=$(wc -c < "$BIN" | tr -d ' ')
    if [ "$size" -eq 32768 ]; then
        echo "REFUSING: $BIN is exactly 32768 bytes." >&2
        echo "That is a full-flash image, not a build of this firmware (~17 KB)." >&2
        echo "You are almost certainly about to flash a stale artifact." >&2
        exit 1
    fi
    echo "Image: $BIN ($size bytes)"
}

case "${1:-flash}" in
    flash)
        check_image
        echo "=== Erase, flash, verify, run ==="
        # Verify runs while the target is HALTED and inside the same session,
        # with reset run after it. See the warning under 'verify' below.
        "${OOCD[@]}" \
            -c "init" \
            -c "reset halt" \
            -c "flash write_image erase $BIN 0x08000000" \
            -c "verify_image $ELF" \
            -c "reset run" \
            -c "shutdown"
        echo "=== Done. Now check g_clk_hse and g_mpu_ok (see docs/FLASHING.md) ==="
        ;;

    verify)
        check_image
        # WARNING: verify_image downloads a CRC stub into the work area, which
        # stm32f0x.cfg places at 0x20000000 with backup disabled — directly over
        # the receiver's state struct — and never restores it. Verifying a
        # RUNNING board leaves the radio deaf until reset, with symptoms that
        # look exactly like broken RF hardware. So we halt first and reset run
        # afterwards, in the same session, accepting the loss of RAM state.
        echo "=== Verify (halts the target, then resets it) ==="
        "${OOCD[@]}" \
            -c "init" \
            -c "halt" \
            -c "verify_image $ELF" \
            -c "reset run" \
            -c "shutdown"
        echo "=== Verification OK (target was reset) ==="
        ;;

    read-gpio)
        echo "=== GPIO state (safe on a running board: plain memory reads) ==="
        "${OOCD[@]}" \
            -c "init" \
            -c "echo {GPIOA_MODER:}" -c "mdw 0x48000000 1" \
            -c "echo {GPIOA_ODR:}"   -c "mdw 0x48000014 1" \
            -c "echo {GPIOB_MODER:}" -c "mdw 0x48000400 1" \
            -c "echo {GPIOB_ODR:}"   -c "mdw 0x48000414 1" \
            -c "shutdown"
        ;;

    read-tim3)
        echo "=== TIM3 registers (tail output timer) ==="
        "${OOCD[@]}" \
            -c "init" \
            -c "echo {CR1:}"  -c "mdw 0x40000400 1" \
            -c "echo {PSC:}"  -c "mdw 0x40000428 1" \
            -c "echo {ARR:}"  -c "mdw 0x4000042c 1" \
            -c "echo {CCR3:}" -c "mdw 0x4000043c 1" \
            -c "echo {CCR4:}" -c "mdw 0x40000440 1" \
            -c "shutdown"
        ;;

    set-tim3)
        # Diagnostic only: force TIM3 to the 125 Hz configuration by hand.
        echo "=== Forcing TIM3 to PSC=47 ARR=8000 CCR3=1333 ==="
        "${OOCD[@]}" \
            -c "init" -c "reset halt" \
            -c "mww 0x40000428 47" \
            -c "mww 0x4000042c 8000" \
            -c "mww 0x4000043c 1333" \
            -c "reset run" -c "shutdown"
        ;;

    *)
        cat <<'USAGE'
Usage: ./tools/flash.sh [flash|verify|read-gpio|read-tim3|set-tim3]

  flash      Erase, flash, verify (halted), and run. Default.
  verify     Verify flash against the ELF. Halts and resets the target —
             never verify a running board, it clobbers the radio's RAM state.
  read-gpio  Dump GPIO mode/output registers. Safe on a running board.
  read-tim3  Dump the tail timer's registers. Safe on a running board.
  set-tim3   Diagnostic: force TIM3 to 125 Hz by hand.

For receiver state and live link rate use the dedicated tools instead:
  ./tools/read_sfhss.sh     decoded S-FHSS state and channel data
  ./tools/mon_rate.tcl      live frame rate (deltas; cumulative counters lie)

Environment:
  FW_DIR   directory holding firmware.bin/.elf (default: ../src next to this
           script). Override only if you know why.
USAGE
        ;;
esac
