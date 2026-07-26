/*
 * Загрузчик: первые 16 КБ флеша. Приложение линкуется отдельным скриптом с
 * origin BOOT_APP_BASE = 0x08004000, поэтому пересечься они не могут.
 *
 * Верхние 16 байт ОЗУ не отдаём линкеру: там лежит магия «остаться в
 * загрузчике», которую приложение пишет перед перезагрузкой (BOOT_MAGIC_ADDR).
 */
MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 16K
    RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 128K - 16
}

_estack = ORIGIN(RAM) + LENGTH(RAM);

SECTIONS
{
    .isr_vector : { KEEP(*(.isr_vector)) } > FLASH

    .text :
    {
        *(.text*)
        *(.rodata*)
        . = ALIGN(4);
    } > FLASH

    _sidata = LOADADDR(.data);

    .data :
    {
        _sdata = .;
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } > RAM AT > FLASH

    .bss (NOLOAD) :
    {
        _sbss = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
    } > RAM

    /DISCARD/ : { *(.ARM.exidx*) }
}
