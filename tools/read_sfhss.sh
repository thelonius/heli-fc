#!/bin/bash
# Read S-FHSS receiver state over SWD. g_sfhss is pinned at 0x20000000
# (linker .sfhss_state section). Field offsets re-derived 2026-07-07 from the
# ELF's DWARF debug info (arm-none-eabi-objdump --dwarf=info firmware.elf)
# after lastFrameSystick shifted everything past +0x10 by 4 bytes. Never
# recompute with host-side offsetof() — -fshort-enums makes SFHSS_Phase_t
# 1 byte on the ARM build. If the struct changes, re-derive:
#   arm-none-eabi-objdump --dwarf=info firmware.elf | awk '
#     /DW_TAG_member/ {m=1; n=""}
#     m && /DW_AT_name/ {n=$NF}
#     m && /DW_AT_data_member_location/ {print n, $NF; m=0}'
openocd -f interface/stlink.cfg -f target/stm32f0x.cfg \
 -c "init" \
 -c "echo {--- phase(0 startbind 1 find 2 binded 3 CONNECTED) ch hopcode nextSlot ---}" \
 -c "mdb 0x20000000 4" \
 -c "echo {--- txaddr (0xFFFFFFFF = none) ---}" -c "mdw 0x20000004 1" \
 -c "echo {--- data ch1-8, usec (center 1520=0x5F0) ---}" -c "mdh 0x2000001a 8" \
 -c "echo {--- rssi ---}" -c "mdb 0x2000002a 1" \
 -c "echo {--- stats: Rcv Lost BadPkt Rebind Failsafe Attempts NoMarker NoCrc Overflow ---}" \
 -c "mdw 0x2000004c 9" \
 -c "echo {--- lastRxbytes lastFifoStatus ---}" -c "mdb 0x2000007f 2" \
 -c "echo {--- lastpkt[15] raw FIFO ---}" -c "mdb 0x20000070 15" \
 -c "echo {--- cntEven cntOdd ---}" -c "mdw 0x20000084 2" \
 -c "shutdown"
