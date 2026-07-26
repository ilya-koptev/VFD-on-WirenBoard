#pragma once
/*
 * Общий заголовок прошивки: CMSIS плюс своя работа с периферией, без вендорского
 * HAL — как в wb-embedded-controller. HAL занимал около 12 КБ флеша и прятал
 * тайминги стекла за уровнями абстракции, а нам важно видеть каждый регистр.
 *
 * Распиновка стенда (NUCLEO-F411RE):
 *   CLK = PA5 (SPI1_SCK)   SIN = PA7 (SPI1_MOSI)   консоль = PA2/PA3 (USART2)
 *   LAT = PB6              BLK = PB4               EF = PB8 (накал, активен HIGH)
 *   HV  = PB9 — вход, не драйвим; SO1 = PA10, SO2 = PB5 — обратка регистра.
 */
#include "stm32f4xx.h"
#include <stdint.h>

/* ---------- выводы стекла ---------- */
#define LAT_PORT   GPIOB
#define LAT_PIN_NO 6
#define BLK_PORT   GPIOB
#define BLK_PIN_NO 4
#define EF_PORT    GPIOB
#define EF_PIN_NO  8

/* Быстрая работа с линией: BSRR атомарен и не требует чтения регистра */
static inline void pin_high(GPIO_TypeDef *p, uint32_t no) { p->BSRR = (1UL << no); }
static inline void pin_low(GPIO_TypeDef *p, uint32_t no)  { p->BSRR = (1UL << (no + 16)); }

/* ---------- тактирование, время ---------- */
void     rcc_init(void);            /* HSI -> PLL 100 МГц, кэш и предвыборка флеша */
void     systick_init(void);        /* тик 1 кГц для отсчётов миллисекунд */
uint32_t systick_ms(void);
void     delay_ms(uint32_t ms);
void     dwt_init(void);            /* счётчик тактов ядра для микросекунд */
void     delay_us(uint32_t us);
uint32_t cycles_now(void);          /* текущее значение счётчика тактов */
void     wait_cycles_from(uint32_t t0, uint32_t need);

/* ---------- линии стекла ---------- */
void pins_init(void);
void filament_enable(int on);

/* ---------- SPI: сдвиг регистра стекла ---------- */
void spi_init(uint32_t prescaler_div);   /* 2,4,8,...,256; рабочая точка — 8 */
void spi_send(const uint8_t *d, int n);
uint32_t spi_clock_hz(void);

/* ---------- консоль и шина на одном UART ---------- */
void uart_init(uint32_t baud);
void uart_push(const char *d, int n);    /* неблокирующая передача через кольцо */
int  uart_get(uint8_t *b);                /* принятый байт, 0 если нет */
int  uart_tx_busy(void);                  /* есть ли что-то в очереди передачи */
