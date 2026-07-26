/*
 * SPI1 — сдвиг 240 бит слота в регистр стекла. Настройки те, на которых стенд
 * работает: мастер, 8 бит, CPOL=0/CPHA=0 (такт покоится низким, данные по
 * фронту), первым уходит старший бит, NSS программный.
 *
 * Рабочая точка — предделитель 8, то есть 12.5 МГц от APB2 100 МГц. Даташит
 * стекла разрешает такт от 400 нс, но столь быстрый сдвиг допустим: критично
 * не время такта, а уровни, а низкий уровень обеспечен подтяжкой SIN к земле.
 */
#include "wbmcu_system.h"

static uint32_t spi_div = 8;

void spi_init(uint32_t prescaler_div)
{
    uint32_t br = 0;                            /* BR: 0 = /2, 1 = /4, 2 = /8 ... */
    for (uint32_t d = 2; d < prescaler_div && br < 7; d <<= 1) br++;
    spi_div = 2UL << br;

    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    /* PA5 = SCK, PA7 = MOSI, альтернативная функция 5 */
    GPIOA->MODER = (GPIOA->MODER & ~((3UL << 10) | (3UL << 14))) |
                   (2UL << 10) | (2UL << 14);
    GPIOA->OSPEEDR |= (3UL << 10) | (3UL << 14);
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~((0xFUL << 20) | (0xFUL << 28))) |
                    (5UL << 20) | (5UL << 28);

    SPI1->CR1 = 0;                              /* выключить перед настройкой */
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI |
                (br << SPI_CR1_BR_Pos);
    SPI1->CR2 = 0;
    SPI1->CR1 |= SPI_CR1_SPE;
}

void spi_send(const uint8_t *d, int n)
{
    for (int i = 0; i < n; i++) {
        while (!(SPI1->SR & SPI_SR_TXE)) { }
        *(volatile uint8_t *)&SPI1->DR = d[i];
    }
    while (!(SPI1->SR & SPI_SR_TXE)) { }
    while (SPI1->SR & SPI_SR_BSY) { }           /* дождаться ухода последнего бита */
}

uint32_t spi_clock_hz(void) { return SystemCoreClock / spi_div; }
