MEMORY
{
    FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 32K
    SRAM (rwx) : ORIGIN = 0x20000000, LENGTH = 4K
}

SECTIONS
{
    .text :
    {
        KEEP(*(.isr_vector))
        *(.text*)
        *(.rodata*)
    } > FLASH

    /* Pin the receiver state to the very start of RAM (0x20000000) so its
     * fields keep stable absolute addresses across rebuilds — SWD debug reads
     * never need recomputing. NOLOAD: not zeroed by the .bss loop, so
     * SFHSS_Init() must clear it explicitly. */
    .sfhss_state 0x20000000 (NOLOAD) :
    {
        KEEP(*(.sfhss_state))
    } > SRAM

    .data :
    {
        _sdata = .;
        *(.data*)
        _edata = .;
    } > SRAM AT > FLASH
    _sidata = LOADADDR(.data);

    .bss :
    {
        _sbss = .;
        *(.bss*)
        *(COMMON)
        _ebss = .;
    } > SRAM

    _estack = ORIGIN(SRAM) + LENGTH(SRAM);
}
