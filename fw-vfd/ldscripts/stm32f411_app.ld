/*
 * Приложение: флеш после 16 КБ загрузчика. Верхние 16 байт ОЗУ не отдаём
 * линкеру — там магия «остаться в загрузчике» (BOOT_MAGIC_ADDR), которую
 * приложение пишет перед перезагрузкой.
 */
MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08004000, LENGTH = 512K - 16K
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

    .ARM.extab : { *(.ARM.extab*) } > FLASH
    .ARM.exidx : { *(.ARM.exidx*) } > FLASH

    .data :
    {
        _sdata = .;
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } > RAM AT > FLASH

    _sidata = LOADADDR(.data);      /* после объявления секции: до неё LOADADDR = 0 */

    .bss (NOLOAD) :
    {
        _sbss = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
    } > RAM

    /* newlib просит куда-то положить кучу; snprintf ею не пользуется, но
       символы нужны для линковки */
    .heap (NOLOAD) :
    {
        . = ALIGN(8);
        _sheap = .;
        . = . + 1K;
        _eheap = .;
    } > RAM
}
