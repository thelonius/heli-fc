#!/bin/bash
# Flash firmware to STM32F031 via OpenOCD
# Usage: ./flash.sh [verify|flash|test]

set -e

ELF="firmware.elf"
BIN="firmware.bin"

if [ ! -f "$BIN" ]; then
    echo "Error: $BIN not found. Run 'make' first."
    exit 1
fi

case "${1:-flash}" in
    verify)
        echo "=== Verifying firmware ==="
        openocd -f interface/stlink.cfg -f target/stm32f0x.cfg \
            -c "init" \
            -c "verify_image $ELF" \
            -c "shutdown"
        echo "=== Verification OK ==="
        ;;

    flash)
        echo "=== Erasing & Flashing $BIN ==="
        openocd -f interface/stlink.cfg -f target/stm32f0x.cfg \
            -c "init" \
            -c "reset halt" \
            -c "flash write_image erase $BIN 0x08000000" \
            -c "reset run" \
            -c "shutdown"
        echo "=== Flash complete, running ==="
        ;;

    read-tim3)
        echo "=== Reading TIM3 registers ==="
        openocd -f interface/stlink.cfg -f target/stm32f0x.cfg \
            -c "init" \
            -c "reset halt" \
            -c "mdw 0x40000400 1"  -c "echo CR1:" \
            -c "mdw 0x40000428 1"  -c "echo PSC:" \
            -c "mdw 0x4000042c 1"  -c "echo ARR:" \
            -c "mdw 0x4000043c 1"  -c "echo CCR3:" \
            -c "mdw 0x40000440 1"  -c "echo CCR4:" \
            -c "reset run" \
            -c "shutdown"
        ;;

    read-gpio)
        echo "=== Reading GPIO state ==="
        openocd -f interface/stlink.cfg -f target/stm32f0x.cfg \
            -c "init" \
            -c "reset halt" \
            -c "echo GPIOA_MODER:" \
            -c "mdw 0x48000000 1" \
            -c "echo GPIOA_ODR:" \
            -c "mdw 0x48000014 1" \
            -c "echo GPIOB_MODER:" \
            -c "mdw 0x48000400 1" \
            -c "echo GPIOB_ODR:" \
            -c "mdw 0x48000414 1" \
            -c "echo PB0 (TIM3_CH3) state:" \
            -c "mdw 0x48000410 1" \
            -c "reset run" \
            -c "shutdown"
        ;;

    set-tim3)
        echo "=== Manually setting TIM3 for 125Hz ==="
        openocd -f interface/stlink.cfg -f target/stm32f0x.cfg \
            -c "init" \
            -c "reset halt" \
            -c "mww 0x40000428 47"    \
            -c "mww 0x4000042c 8000"  \
            -c "mww 0x4000043c 1333"  \
            -c "echo TIM3 PSC=47 ARR=8000 CCR3=1333 written" \
            -c "reset run" \
            -c "shutdown"
        ;;

    read-rc)
        echo "=== Reading RC state ==="
        openocd -f interface/stlink.cfg -f target/stm32f0x.cfg \
            -c "init" \
            -c "reset halt" \
            -c "echo g_rc_channels (8 × uint16):" \
            -c "mdh 0x20000000 8" \
            -c "echo g_packets_received:" \
            -c "mdw 0x20000010 1" \
            -c "echo g_errors:" \
            -c "mdw 0x20000014 1" \
            -c "echo GPIOA_MODER:" \
            -c "mdw 0x48000000 1" \
            -c "echo GPIOA_ODR:" \
            -c "mdw 0x48000014 1" \
            -c "reset run" \
            -c "shutdown"
        ;;

    *)
        echo "Usage: $0 [flash|verify|read-tim3|read-gpio|read-rc|set-tim3]"
        echo ""
        echo "  flash      - Erase and flash firmware (default)"
        echo "  verify     - Verify firmware matches ELF"
        echo "  read-tim3  - Read TIM3 register values"
        echo "  read-gpio  - Read GPIO state"
        echo "  read-rc    - Read RC channels + packet count + errors"
        echo "  set-tim3   - Manually set TIM3 to 125Hz via OpenOCD"
        ;;
esac
