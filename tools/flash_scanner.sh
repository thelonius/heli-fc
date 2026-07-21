#!/bin/bash
# Flash the SCANNER build (SFHSS_PAIR_MODE=3): parks on one hop channel and
# ring-logs every frame. NOT flyable — no hopping, most channels time out.
# Restore with ./flash.sh flash (production firmware.bin stays untouched).
set -e
cd "$(dirname "$0")"
[ -f firmware_scanner.bin ] || { echo "firmware_scanner.bin not found"; exit 1; }
echo "=== Flashing SCANNER build ==="
openocd -f interface/stlink.cfg -f target/stm32f0x.cfg \
    -c "init" \
    -c "reset halt" \
    -c "flash write_image erase firmware_scanner.bin 0x08000000" \
    -c "reset run" \
    -c "shutdown"
echo "=== Scanner running. TX on, wait ~30s, then ./read_ring.sh (3-4 times) ==="
