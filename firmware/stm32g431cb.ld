/* STM32G431CB: 128 KB flash, 32 KB SRAM1+SRAM2 contiguous, 10 KB CCM SRAM.
 * The control loop and its data live in CCM: zero-wait-state and no contention
 * with DMA on the main bus. */
ENTRY(Reset_Handler)

MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 128K
    RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 32K
    CCM   (rwx) : ORIGIN = 0x10000000, LENGTH = 10K
}

_estack = ORIGIN(RAM) + LENGTH(RAM);

SECTIONS
{
    .isr_vector : { KEEP(*(.isr_vector)) } > FLASH

    .text : {
        *(.text*)
        *(.rodata*)
        . = ALIGN(4);
    } > FLASH

    _sidata = LOADADDR(.data);
    .data : {
        _sdata = .;
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } > RAM AT > FLASH

    .bss (NOLOAD) : {
        _sbss = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
    } > RAM

    /* hot state: see main.c, section attribute "ccm" (zeroed by Reset_Handler) */
    .ccm (NOLOAD) : {
        _sccm = .;
        *(.ccm*)
        . = ALIGN(4);
        _eccm = .;
    } > CCM

    /DISCARD/ : { *(.ARM.exidx*) *(.comment) }
}
