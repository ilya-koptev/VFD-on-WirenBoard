/*
 * Линии стекла. Уровни при старте — безопасные по примечанию 3 даташита:
 * BLK в гашении, пока не пошёл скан, иначе при поданном VDD2 стекло светит тем,
 * что осталось в сдвиговом регистре.
 */
#include "wbmcu_system.h"

/* режимы вывода: 00 вход, 01 выход, 10 альтернативная функция */
static void mode_set(GPIO_TypeDef *p, uint32_t no, uint32_t mode)
{
    p->MODER = (p->MODER & ~(3UL << (no * 2))) | (mode << (no * 2));
}

static void speed_very_high(GPIO_TypeDef *p, uint32_t no)
{
    p->OSPEEDR |= (3UL << (no * 2));
}

void pins_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;

    /* Сначала уровни, потом переключаем на выход — чтобы на линии не мелькнуло
       обратное состояние: у BLK это означало бы вспышку неподготовленного кадра. */
    pin_high(BLK_PORT, BLK_PIN_NO);         /* гашение */
    pin_low(LAT_PORT, LAT_PIN_NO);
    pin_high(EF_PORT, EF_PIN_NO);           /* накал включён */

    mode_set(BLK_PORT, BLK_PIN_NO, 1);
    mode_set(LAT_PORT, LAT_PIN_NO, 1);
    mode_set(EF_PORT, EF_PIN_NO, 1);
    speed_very_high(BLK_PORT, BLK_PIN_NO);
    speed_very_high(LAT_PORT, LAT_PIN_NO);

    /* HV (PB9) не драйвим: плата поднимает его сама, оставляем входом */
    mode_set(GPIOB, 9, 0);

    /* Обратка регистра SO1 (PA10) и SO2 (PB5): входы с подтяжкой к земле.
       Сама цепь данных подтянута к земле резистором 1 кОм на плате — без него
       низкий уровень на дальнем конце не проходит порог VIL 0.7 В и биты
       размазываются по регистру. */
    mode_set(GPIOA, 10, 0);
    GPIOA->PUPDR = (GPIOA->PUPDR & ~(3UL << 20)) | (2UL << 20);
    mode_set(GPIOB, 5, 0);
    GPIOB->PUPDR = (GPIOB->PUPDR & ~(3UL << 10)) | (2UL << 10);
}

void filament_enable(int on)
{
    if (on) pin_high(EF_PORT, EF_PIN_NO);
    else    pin_low(EF_PORT, EF_PIN_NO);
}
