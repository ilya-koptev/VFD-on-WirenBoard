/*
 * Стартовый код: таблица векторов, инициализация памяти, вход в main.
 * Своя, а не вендорская — загрузчику нужны только сброс и SysTick, а лишние
 * килобайты в начале флеша нам дороги.
 */
#include <stdint.h>

extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _estack;

int main(void);
void SysTick_Handler(void);

static void default_handler(void)
{
    while (1) { }               /* необработанное прерывание: стоим, сработает вотчдог */
}

void Reset_Handler(void)
{
    /* .data из флеша в ОЗУ */
    uint32_t *src = &_sidata, *dst = &_sdata;
    while (dst < &_edata) *dst++ = *src++;
    /* .bss в нули */
    for (dst = &_sbss; dst < &_ebss; dst++) *dst = 0;
    main();
    while (1) { }
}

void USART2_IRQHandler(void)    __attribute__((weak, alias("default_handler")));
void NMI_Handler(void)          __attribute__((weak, alias("default_handler")));
void HardFault_Handler(void)    __attribute__((weak, alias("default_handler")));
void MemManage_Handler(void)    __attribute__((weak, alias("default_handler")));
void BusFault_Handler(void)     __attribute__((weak, alias("default_handler")));
void UsageFault_Handler(void)   __attribute__((weak, alias("default_handler")));
void SVC_Handler(void)          __attribute__((weak, alias("default_handler")));
void DebugMon_Handler(void)     __attribute__((weak, alias("default_handler")));
void PendSV_Handler(void)       __attribute__((weak, alias("default_handler")));

/* Системные векторы плюс те периферийные, что реально нужны. Индекс вектора =
   16 + номер IRQ; USART2_IRQn = 38, поэтому его место — 54. Остальные нули:
   в загрузчике периферийных прерываний нет вовсе, в приложении только консоль. */
__attribute__((section(".isr_vector"), used))
void (*const vector_table[55])(void) = {
    (void (*)(void))&_estack,
    Reset_Handler,
    NMI_Handler,
    HardFault_Handler,
    MemManage_Handler,
    BusFault_Handler,
    UsageFault_Handler,
    0, 0, 0, 0,
    SVC_Handler,
    DebugMon_Handler,
    0,
    PendSV_Handler,
    SysTick_Handler,
    [16 + 38] = USART2_IRQHandler,
};
