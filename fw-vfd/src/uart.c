/*
 * USART2 (PA2/PA3) — консоль и шина Modbus на одной линии, 115200 8N1.
 *
 * Приём и передача через кольцевые буферы по прерыванию: блокирующая передача
 * съедала время между кадрами дисплея и рвала разбор входящих байтов, а на
 * двухбайтовых русских символах в строку влезал мусор.
 */
#include "wbmcu_system.h"

#define RXSZ 256
#define TXSZ 256

static volatile uint8_t  rxbuf[RXSZ], txbuf[TXSZ];
static volatile uint16_t rxhead, rxtail, txhead, txtail;

void USART2_IRQHandler(void)
{
    uint32_t sr = USART2->SR;

    if (sr & USART_SR_RXNE) {
        uint8_t b = (uint8_t)USART2->DR;
        uint16_t nx = (uint16_t)((rxhead + 1) % RXSZ);
        if (nx != rxtail) { rxbuf[rxhead] = b; rxhead = nx; }   /* переполнение молча теряем */
    }

    if ((sr & USART_SR_TXE) && (USART2->CR1 & USART_CR1_TXEIE)) {
        if (txtail != txhead) {
            USART2->DR = txbuf[txtail];
            txtail = (uint16_t)((txtail + 1) % TXSZ);
        } else {
            USART2->CR1 &= ~USART_CR1_TXEIE;
        }
    }

    if (sr & (USART_SR_ORE | USART_SR_NE | USART_SR_FE)) (void)USART2->DR;  /* сбросить ошибки */
}

void uart_init(uint32_t baud)
{
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    /* PA2 = TX, PA3 = RX, альтернативная функция 7 */
    GPIOA->MODER = (GPIOA->MODER & ~((3UL << 4) | (3UL << 6))) | (2UL << 4) | (2UL << 6);
    GPIOA->PUPDR = (GPIOA->PUPDR & ~((3UL << 4) | (3UL << 6))) | (1UL << 4) | (1UL << 6);
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~((0xFUL << 8) | (0xFUL << 12))) |
                    (7UL << 8) | (7UL << 12);

    uint32_t pclk1 = SystemCoreClock / 2;        /* APB1 = SYSCLK / 2 */
    USART2->CR1 = 0;
    USART2->BRR = (pclk1 + baud / 2) / baud;     /* с округлением */
    USART2->CR2 = 0;                             /* один стоп-бит */
    USART2->CR3 = 0;
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;

    NVIC_SetPriority(USART2_IRQn, 1);
    NVIC_EnableIRQ(USART2_IRQn);
}

void uart_push(const char *d, int n)
{
    for (int i = 0; i < n; i++) {
        uint16_t nx = (uint16_t)((txhead + 1) % TXSZ);
        while (nx == txtail) { }                 /* кольцо полно — ждём передатчик */
        txbuf[txhead] = (uint8_t)d[i];
        txhead = nx;
    }
    USART2->CR1 |= USART_CR1_TXEIE;
}

int uart_get(uint8_t *b)
{
    if (rxtail == rxhead) return 0;
    *b = rxbuf[rxtail];
    rxtail = (uint16_t)((rxtail + 1) % RXSZ);
    return 1;
}

int uart_tx_busy(void) { return txtail != txhead; }
