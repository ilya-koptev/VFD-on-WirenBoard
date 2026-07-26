/*
 * Тактирование и время. Значения те же, на которых стенд работает: HSI 16 МГц,
 * PLL до 100 МГц, APB1 = 50 МГц, APB2 = 100 МГц, флеш с тремя тактами ожидания.
 *
 * Кэш инструкций, кэш данных и предвыборка обязательны: без них тесные циклы
 * вывода на 3 тактах ожидания флеша замедляются в разы.
 */
#include "wbmcu_system.h"

uint32_t SystemCoreClock = 16000000UL;      /* до вызова rcc_init работаем на HSI */

void rcc_init(void)
{
    /* FPU включаем первым делом: собираемся с аппаратной плавающей точкой (её
       использует вращение куба), и первая же FP-инструкция при выключенном
       сопроцессоре даёт HardFault с битом NOCP. У HAL это делал SystemInit. */
    SCB->CPACR |= (3UL << 20) | (3UL << 22);            /* полный доступ к CP10/CP11 */
    __DSB();
    __ISB();

    RCC->APB1ENR |= RCC_APB1ENR_PWREN;                  /* доступ к регулятору */
    PWR->CR |= PWR_CR_VOS;                              /* режим 1: разрешает 100 МГц */

    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN |
                 FLASH_ACR_LATENCY_3WS;

    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY)) { }

    /* PLLM=16 -> 1 МГц на входе VCO, PLLN=400 -> VCO 400 МГц, PLLP=/4 -> 100 МГц.
       Источник — HSI (бит PLLSRC = 0), PLLQ=8 нужен только для USB. */
    RCC->PLLCFGR = (16UL << RCC_PLLCFGR_PLLM_Pos) |
                   (400UL << RCC_PLLCFGR_PLLN_Pos) |
                   (1UL << RCC_PLLCFGR_PLLP_Pos) |      /* 01 = делить на 4 */
                   (8UL << RCC_PLLCFGR_PLLQ_Pos);
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) { }

    /* AHB /1, APB1 /2 (предел 50 МГц), APB2 /1 */
    RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2)) |
                RCC_CFGR_PPRE1_DIV2;

    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) { }

    SystemCoreClock = 100000000UL;
}

/* ---------- миллисекунды ---------- */
static volatile uint32_t ms_ticks;

void SysTick_Handler(void) { ms_ticks++; }

void systick_init(void)
{
    SysTick->LOAD = SystemCoreClock / 1000U - 1U;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk |
                    SysTick_CTRL_ENABLE_Msk;
}

uint32_t systick_ms(void) { return ms_ticks; }

void delay_ms(uint32_t ms)
{
    uint32_t t0 = ms_ticks;
    while (ms_ticks - t0 < ms) { }
}

/* ---------- микросекунды на счётчике тактов ядра ----------
   DWT есть у Cortex-M4. При переносе на GD32E230 (Cortex-M23) его не будет —
   там эти функции лягут на аппаратный таймер, вызовы останутся те же. */
void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t cycles_now(void) { return DWT->CYCCNT; }

void delay_us(uint32_t us)
{
    uint32_t t0 = DWT->CYCCNT;
    uint32_t need = us * (SystemCoreClock / 1000000U);
    while ((DWT->CYCCNT - t0) < need) { }
}

/* Догнать заданный период от метки: так выравнивается период слота и кадр
   не дрожит от прерываний и разной длины кода в слотах. */
void wait_cycles_from(uint32_t t0, uint32_t need)
{
    while ((DWT->CYCCNT - t0) < need) { }
}
