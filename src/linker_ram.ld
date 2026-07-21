MEMORY
{
    SRAM (rwx) : ORIGIN = 0x20000000, LENGTH = 4K
}

SECTIONS
{
    .text :
    {
        KEEP(*(.isr_vector))
        *(.text*)
        *(.rodata*)
    } > SRAM

    .data :
    {
        _sdata = .;
        *(.data*)
        _edata = .;
    } > SRAM

    .bss :
    {
        _sbss = .;
        *(.bss*)
        *(COMMON)
        _ebss = .;
    } > SRAM

    _estack = ORIGIN(SRAM) + LENGTH(SRAM);
}
