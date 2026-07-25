/*
 * MN12832L (Noritake Itron, 128x32 chip-in-glass) на NUCLEO-F411RE.
 * Чистая реализация по даташиту Doc.Ref 3772 Iss.1:
 *
 *   Сдвиговый регистр 240 бит:  биты 1..192  = пиксели, 32 строки x 6 анодов
 *                                              в порядке 'afbecd' (строка N = биты 6N-5..6N)
 *                               биты 193..236 = выбор сеток G1..G44
 *                               биты 237..240 = не используются
 *   Мультиплекс: сетки включаются ПАРАМИ С ПЕРЕКРЫТИЕМ (G1+G2, потом G2+G3, ...),
 *                слот ~100 мкс, кадр 44 слота < 10 мс.
 *   Данные слотов чередуются 'abc' / 'def': в слоте пишется одна триада из шести
 *                анодных позиций = 3 столбца одной сетки (44 x 3 = 132 >= 128).
 *   Тайминги: CLK cycle >= 400 нс (макс 2.5 МГц), CLK->LAT >= 250 нс,
 *             LAT high >= 300 нс, BLK hold >= 10 мкс, SIN setup 40 нс.
 *   Безопасность: при остановке скана с поданным VDD2 держать BLK HIGH.
 *
 * Распиновка (как подключено на стенде):
 *   CLK = PA5 (D13, SPI1_SCK)      SIN = PA7 (D11, SPI1_MOSI)
 *   LAT = PB6 (D10)                BLK = PB4 (D5)
 *   EF  = PB8 (D15, активен HIGH)  HV  = PB9 (D14) - НЕ драйвим, плата поднимает сама
 *   SO1 = PA10 (D2), SO2 = PB5 (D4) - входы для обратного чтения регистра (если подключены)
 *
 * Консоль USART2 (PA2/PA3) -> ST-Link VCP, 115200. Команды - см. help.
 */
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>
#include "logo.h"

#define LAT_PORT GPIOB
#define LAT_PIN  GPIO_PIN_6
#define BLK_PORT GPIOB
#define BLK_PIN  GPIO_PIN_4
#define EF_PORT  GPIOB
#define EF_PIN   GPIO_PIN_8
/* GCP — счётный клок ШИМ-декодера (даташит, прим.4). На разъёме модуля НЕ выведен;
   выводим на свободную ногу PC7 (D9), чтобы драйв был датащит-полным, как только
   линию удастся подпаять к выводу стекла. */
#define GCP_PORT GPIOC
#define GCP_PIN  GPIO_PIN_7

#define SO1_PORT GPIOA
#define SO1_PIN  GPIO_PIN_10
#define SO2_PORT GPIOB
#define SO2_PIN  GPIO_PIN_5

#define MAX_SLOTS 44
#define MAX_BYTES 60          /* 480 бит, если у платы две цепи каскадом */
#define RAW_BYTES 80          /* до 640 бит для измерения длины цепи */

UART_HandleTypeDef huart2;
SPI_HandleTypeDef  hspi1;

/* ---------- кадровый буфер 128x32, 1 бит/точка, MSB = левый столбец ---------- */
static uint8_t fb[32][16];

/* ---------- рантайм-конфиг (крутится командой cfg, чтобы не перешивать) ------ */
static struct {
    uint8_t slots;   /* сколько слотов в кадре (44 по даташиту)                  */
    uint8_t gstep;   /* на сколько сеток двигаться за слот (1 = перекрытие)      */
    uint8_t dbl;     /* 1 = выбирать пару сеток (бит g и g+1)                    */
    uint8_t triad;   /* 0 = afbecd -> {0,2,4}/{5,3,1}; 1 = {0,1,2}/{3,4,5}       */
    uint8_t par;     /* 1 = поменять триады местами по чётности слота            */
    uint8_t mir;     /* 1 = зеркалить X                                          */
    uint8_t miry;    /* 1 = зеркалить Y (карта строк идёт снизу вверх)            */
    uint8_t rev;     /* 1 = перевернуть порядок 240 бит в потоке (бит1 последним) */
    uint8_t bytes;   /* 30 или 60 байт на слот                                    */
    uint8_t dup;     /* при 60 байтах: 0 = данные+нули, 1 = данные дважды, 2 = нули+данные */
    uint8_t mode;    /* 0 = гасим-сдвигаем-латчим; 1 = сдвиг во время показа (по даташиту) */
    uint16_t on_us;  /* доп. время показа в слоте, мкс                            */
    uint16_t blk_us; /* длительность гашения, мкс (>=10 по даташиту)              */
    int16_t  gfix;   /* -1 = гнать все слоты, иначе только этот слот              */
    int16_t  probe;  /* -1 = обычный рендер, иначе только этот бит регистра 1..240 */
    uint8_t  scan;   /* 1 = скан идёт, 0 = стоп (BLK удерживаем HIGH)             */
    uint16_t hold;   /* мс статики без тактов после показа (0 = не держать)       */
    uint8_t  blkpol; /* 0 = HIGH гасит (по даташиту), 1 = наоборот                */
    uint8_t  latpol; /* 0 = импульс HIGH, 1 = импульс LOW                        */
    uint8_t  nolat;  /* 1 = не дёргать LAT вообще                                 */
    uint8_t  gcp;    /* сколько импульсов GCP выдавать за слот (даташит: 5-6)      */
    uint16_t bbdly;  /* доп. задержка на бит в битбенге, мкс (замедление CLK)      */
    uint8_t  rtz;    /* 1 = return-to-zero: сразу после такта гасим линию данных.
                        Даташит требует VIL <= 0.7 В — жёстко; если земля/линия
                        подсаживают ноль, единица «протягивается» в следующие
                        разряды. RTZ даёт линии почти весь бит на спад.        */
    uint16_t setus;  /* время установки данных до фронта CLK, мкс (0 = ~60 нс)     */
    uint16_t rddly;  /* задержка фазы такта при ЧТЕНИИ через SO1, мкс (для sord):
                        позволяет отделить смаз записи от медленного спада SO1 */
    uint16_t slotus; /* ФИКСИРОВАННЫЙ период слота, мкс (0 = как получится).
                        Выравнивает кадр: убирает дрожание от прерываний и от
                        разной длины кода в слотах. Даташит: слот ~100 мкс.     */
    uint16_t clkns;  /* длительность полки такта, нс (даташит: >=200, период >=400) */
    uint8_t  revrow; /* 1 = реверс шести бит внутри строки: afbecd -> dcebfa
                        (так у mariosgit; у нас проверяем оба варианта)          */
} cfg = { 44, 1, 1, 0, 0, 1, 1, 0, 30, 0, 1, 0, 12, -1, -1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 2, 0, 250, 0 };

/* Роли LAT/BLK можно менять в рантайме: подписи на разъёме могут не совпадать с реальностью */
static uint16_t pin_lat = LAT_PIN, pin_blk = BLK_PIN;

static void blk_blank(void);
static void blk_show(void);
static void lat_pulse(void);

static uint8_t slotbuf[MAX_SLOTS][MAX_BYTES];
static volatile uint8_t dirty = 1;

/* --- сырой поток для измерения топологии цепи: блок единиц [rs, rs+rn) в потоке rt бит --- */
static uint8_t rawbuf[RAW_BYTES];
static struct { uint8_t en, manual; uint16_t rs, rn, rt; } raw = { 0, 0, 0, 48, 600 };

/* ---------- консоль ---------- */
#define RXSZ 256
static volatile uint8_t rxbuf[RXSZ];
static volatile uint16_t rxhead = 0, rxtail = 0;
static uint8_t rxbyte;

void SysTick_Handler(void)  { HAL_IncTick(); }
void USART2_IRQHandler(void){ HAL_UART_IRQHandler(&huart2); }
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *h)
{
    if (h->Instance == USART2) {
        uint16_t nx = (rxhead + 1) % RXSZ;
        if (nx != rxtail) { rxbuf[rxhead] = rxbyte; rxhead = nx; }
        HAL_UART_Receive_IT(&huart2, &rxbyte, 1);
    }
}
static int rx_get(uint8_t *b)
{
    if (rxtail == rxhead) return 0;
    *b = rxbuf[rxtail]; rxtail = (rxtail + 1) % RXSZ; return 1;
}
static void up(const char *s) { HAL_UART_Transmit(&huart2, (uint8_t *)s, strlen(s), HAL_MAX_DELAY); }
static void upf(const char *f, ...)
{
    char b[128]; va_list a; va_start(a, f);
    int n = vsnprintf(b, sizeof b, f, a); va_end(a);
    if (n > 0) HAL_UART_Transmit(&huart2, (uint8_t *)b, (uint16_t)n, HAL_MAX_DELAY);
}

/* ---------- микросекундные задержки на DWT ---------- */
static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}
static void udelay(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000U);
    while ((DWT->CYCCNT - start) < ticks) { }
}

/* Подождать N тактов процессора (для полок такта короче микросекунды) */
static inline void cyc_wait(uint32_t cycles)
{
    uint32_t t0 = DWT->CYCCNT;
    while ((DWT->CYCCNT - t0) < cycles) { }
}

/* Догнать фиксированный период слота: убирает дрожание кадра */
static void slot_wait(uint32_t t0)
{
    if (!cfg.slotus) return;
    uint32_t need = cfg.slotus * (SystemCoreClock / 1000000U);
    while ((DWT->CYCCNT - t0) < need) { }
}

static void Error_Handler(void) { while (1) { } }

/* Разгон ядра: HSI 16 МГц -> PLL -> 100 МГц. Нужен, чтобы битбенг успевал
   выдавать бит за ~0.9 мкс (кадр 44 слота x 240 бит < 10 мс = 100 Гц).
   APB2 = 100 МГц, APB1 = 50 МГц, флеш 3 такта ожидания. */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* Без кэша инструкций и предвыборки на 3 тактах ожидания флеша тесный цикл
       битбенга тормозит в разы — это и съедало разгон. */
    __HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
    __HAL_FLASH_DATA_CACHE_ENABLE();
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    osc.PLL.PLLM = 16;      /* 16 МГц / 16 = 1 МГц на входе VCO */
    osc.PLL.PLLN = 400;     /* VCO = 400 МГц                     */
    osc.PLL.PLLP = RCC_PLLP_DIV4;   /* SYSCLK = 100 МГц          */
    osc.PLL.PLLQ = 8;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();

    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                    RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_3) != HAL_OK) Error_Handler();
}

/* ---------- инициализация периферии ---------- */
static void GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};

    g.Mode = GPIO_MODE_OUTPUT_PP; g.Pull = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Pin  = LAT_PIN | BLK_PIN | EF_PIN;
    HAL_GPIO_Init(GPIOB, &g);

    /* BLK HIGH = гашение (безопасное состояние по прим.3 даташита), LAT low, накал ON */
    HAL_GPIO_WritePin(BLK_PORT, BLK_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LAT_PORT, LAT_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EF_PORT,  EF_PIN,  GPIO_PIN_SET);

    /* HV не трогаем — вход, Hi-Z */
    g.Mode = GPIO_MODE_INPUT; g.Pull = GPIO_NOPULL; g.Pin = GPIO_PIN_9;
    HAL_GPIO_Init(GPIOB, &g);

    /* GCP на PC7 */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    g.Mode = GPIO_MODE_OUTPUT_PP; g.Pull = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Pin = GCP_PIN; HAL_GPIO_Init(GCP_PORT, &g);
    HAL_GPIO_WritePin(GCP_PORT, GCP_PIN, GPIO_PIN_RESET);

    /* обратка регистра */
    g.Mode = GPIO_MODE_INPUT; g.Pull = GPIO_PULLDOWN;
    g.Pin = SO1_PIN; HAL_GPIO_Init(SO1_PORT, &g);
    g.Pin = SO2_PIN; HAL_GPIO_Init(SO2_PORT, &g);
}

static uint32_t spi_presc = SPI_BAUDRATEPRESCALER_8;   /* 16 МГц / 8 = 2 МГц (<=2.5 МГц) */

static void SPI1_Init(void)
{
    __HAL_RCC_SPI1_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Pin = GPIO_PIN_5 | GPIO_PIN_7; g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_VERY_HIGH; g.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &g);

    hspi1.Instance               = SPI1;
    hspi1.Init.Mode              = SPI_MODE_MASTER;
    hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;    /* CLK покоится LOW, данные по фронту */
    hspi1.Init.CLKPhase          = SPI_PHASE_1EDGE;
    hspi1.Init.NSS               = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = spi_presc;
    hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;    /* первым уходит бит 1 регистра */
    hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) Error_Handler();
}

static void USART2_Init(void)
{
    __HAL_RCC_USART2_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Pin = GPIO_PIN_2 | GPIO_PIN_3; g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_PULLUP; g.Speed = GPIO_SPEED_FREQ_HIGH; g.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &g);

    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = 115200;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
    HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    HAL_UART_Receive_IT(&huart2, &rxbyte, 1);
}

/* ---------- графика в кадровом буфере ---------- */
static void fb_clear(void) { memset(fb, 0x00, sizeof fb); dirty = 1; }
static void fb_fill(void)  { memset(fb, 0xFF, sizeof fb); dirty = 1; }
static void fb_set(int x, int y, int on)
{
    if (x < 0 || x >= 128 || y < 0 || y >= 32) return;
    if (on) fb[y][x >> 3] |=  (0x80 >> (x & 7));
    else    fb[y][x >> 3] &= ~(0x80 >> (x & 7));
    dirty = 1;
}
static int fb_get(int x, int y) { return (fb[y][x >> 3] >> (7 - (x & 7))) & 1; }
static void fb_hline(int y) { for (int x = 0; x < 128; x++) fb_set(x, y, 1); }
static void fb_vline(int x) { for (int y = 0; y < 32;  y++) fb_set(x, y, 1); }
static void fb_border(void) { fb_hline(0); fb_hline(31); fb_vline(0); fb_vline(127); }
static void fb_rect(int x, int y, int w, int h, int on)
{
    for (int dy = 0; dy < h; dy++)
        for (int dx = 0; dx < w; dx++) fb_set(x + dx, y + dy, on);
}
/* Линия по Брезенхэму — нужна для каркасной графики */
static void fb_line(int x0, int y0, int x1, int y1)
{
    int dx = abs(x1 - x0), dy = -abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1, err = dx + dy;
    for (;;) {
        fb_set(x0, y0, 1);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

/* Логотип Wiren Board во весь экран (данные в logo.h) */
static uint32_t xrand(void);        /* определён ниже, у куба */
static float logo_x = -32.0f, logo_y = 2.0f;      /* левый верхний угол лого */
static float logo_vx = 0.55f, logo_vy = 0.22f;

/* Лого со смещением: то, что вышло за поле, просто обрезается */
static void fb_logo_at(int ox, int oy)
{
    fb_clear();
    for (int y = 0; y < LOGO_H; y++) {
        int fy = oy + y;
        if (fy < 0 || fy >= 32) continue;
        for (int x = 0; x < LOGO_W; x++) {
            int fx = ox + x;
            if (fx < 0 || fx >= 128) continue;
            if ((LOGO[y][x >> 3] >> (7 - (x & 7))) & 1) fb_set(fx, fy, 1);
        }
    }
}

static void fb_logo(void) { fb_logo_at((128 - LOGO_W) / 2, (32 - LOGO_H) / 2); }

/* Плавное блуждание лого. Границы подобраны так, чтобы картинка вылезала
   за поле примерно на треть своей ширины с каждой стороны. */
static void logo_move(void)
{
    logo_vx += ((int)(xrand() % 200) - 100) / 2600.0f;
    logo_vy += ((int)(xrand() % 200) - 100) / 4200.0f;
    if (logo_vx >  0.8f) logo_vx =  0.8f;
    if (logo_vx < -0.8f) logo_vx = -0.8f;
    if (logo_vy >  0.30f) logo_vy =  0.30f;
    if (logo_vy < -0.30f) logo_vy = -0.30f;
    logo_x += logo_vx; logo_y += logo_vy;

    const float xmin = -(float)LOGO_W / 3.0f;            /* треть вылезает слева */
    const float xmax = 128.0f - (float)LOGO_W * 2.0f / 3.0f;
    const float ymin = -4.0f, ymax = 32.0f - (float)LOGO_H + 4.0f;
    if (logo_x < xmin) { logo_x = xmin; logo_vx = -logo_vx; }
    if (logo_x > xmax) { logo_x = xmax; logo_vx = -logo_vx; }
    if (logo_y < ymin) { logo_y = ymin; logo_vy = -logo_vy; }
    if (logo_y > ymax) { logo_y = ymax; logo_vy = -logo_vy; }
}

/* Вращающийся куб: 8 вершин, 12 рёбер, поворот вокруг двух осей,
   перспективная проекция. У F411 есть FPU, так что считаем во float. */
static float cube_ang = 0.0f;
static float cube_cx = 64.0f, cube_cy = 16.0f;   /* центр куба плавает */
static float cube_vx = 0.7f,  cube_vy = 0.35f;
static uint32_t cube_rnd = 0x1234567U;
static uint8_t cube_walk = 1;                    /* 1 = блуждать по полю */

static uint32_t xrand(void)   /* простой генератор, без библиотек */
{
    cube_rnd ^= cube_rnd << 13; cube_rnd ^= cube_rnd >> 17; cube_rnd ^= cube_rnd << 5;
    return cube_rnd;
}

static void cube_move(void)
{
    if (!cube_walk) { cube_cx = 64.0f; cube_cy = 16.0f; return; }
    /* случайный толчок, чтобы траектория не была прямой */
    cube_vx += ((int)(xrand() % 200) - 100) / 900.0f;
    cube_vy += ((int)(xrand() % 200) - 100) / 1400.0f;
    if (cube_vx >  1.4f) cube_vx =  1.4f;
    if (cube_vx < -1.4f) cube_vx = -1.4f;
    if (cube_vy >  0.7f) cube_vy =  0.7f;
    if (cube_vy < -0.7f) cube_vy = -0.7f;
    cube_cx += cube_vx; cube_cy += cube_vy;
    /* отражение от краёв: куб может немного вылезать за поле */
    if (cube_cx < 12.0f)  { cube_cx = 12.0f;  cube_vx = -cube_vx; }
    if (cube_cx > 116.0f) { cube_cx = 116.0f; cube_vx = -cube_vx; }
    if (cube_cy < 4.0f)   { cube_cy = 4.0f;   cube_vy = -cube_vy; }
    if (cube_cy > 28.0f)  { cube_cy = 28.0f;  cube_vy = -cube_vy; }
}
static float cube_scale = 17.0f;      /* немного вылезает за края по краям поворота */

static void fb_cube(float a)
{
    static const signed char V[8][3] = {
        {-1,-1,-1},{1,-1,-1},{1,1,-1},{-1,1,-1},{-1,-1,1},{1,-1,1},{1,1,1},{-1,1,1} };
    static const unsigned char E[12][2] = {
        {0,1},{1,2},{2,3},{3,0},{4,5},{5,6},{6,7},{7,4},{0,4},{1,5},{2,6},{3,7} };
    float ca = cosf(a), sa = sinf(a), cb = cosf(a * 0.61f), sb = sinf(a * 0.61f);
    int px[8], py[8];
    for (int i = 0; i < 8; i++) {
        float x = V[i][0], y = V[i][1], z = V[i][2];
        float x1 =  x * ca + z * sa;          /* поворот вокруг Y */
        float z1 = -x * sa + z * ca;
        float y1 =  y * cb - z1 * sb;         /* поворот вокруг X */
        float z2 =  y * sb + z1 * cb;
        float k  = 3.0f / (3.6f + z2);        /* перспектива */
        px[i] = (int)(cube_cx + x1 * k * cube_scale);
        py[i] = (int)(cube_cy + y1 * k * cube_scale);
    }
    fb_clear();
    for (int i = 0; i < 12; i++)
        fb_line(px[E[i][0]], py[E[i][0]], px[E[i][1]], py[E[i][1]]);
}

static uint8_t anim = 0;              /* 0 = статика, 1 = куб, 2 = лого */
static uint8_t demo_on = 1;           /* 1 = сам чередует куб и лого по 15 с */
#define DEMO_MS 15000U

static void fb_checker(int sz)
{
    for (int y = 0; y < 32; y++)
        for (int x = 0; x < 128; x++)
            fb_set(x, y, ((x / sz) + (y / sz)) & 1);
}

static const uint8_t FONT5x7[][5] = {
 {0,0,0,0,0},{0x3E,0x51,0x49,0x45,0x3E},{0,0x42,0x7F,0x40,0},{0x42,0x61,0x51,0x49,0x46},{0x21,0x41,0x45,0x4B,0x31},
 {0x18,0x14,0x12,0x7F,0x10},{0x27,0x45,0x45,0x45,0x39},{0x3C,0x4A,0x49,0x49,0x30},{0x01,0x71,0x09,0x05,0x03},
 {0x36,0x49,0x49,0x49,0x36},{0x06,0x49,0x49,0x29,0x1E},{0x7E,0x11,0x11,0x11,0x7E},{0x7F,0x49,0x49,0x49,0x36},
 {0x3E,0x41,0x41,0x41,0x22},{0x7F,0x41,0x41,0x22,0x1C},{0x7F,0x49,0x49,0x49,0x41},{0x7F,0x09,0x09,0x09,0x01},
 {0x3E,0x41,0x49,0x49,0x7A},{0x7F,0x08,0x08,0x08,0x7F},{0x00,0x41,0x7F,0x41,0x00},{0x20,0x40,0x41,0x3F,0x01},
 {0x7F,0x08,0x14,0x22,0x41},{0x7F,0x40,0x40,0x40,0x40},{0x7F,0x02,0x0C,0x02,0x7F},{0x7F,0x04,0x08,0x10,0x7F},
 {0x3E,0x41,0x41,0x41,0x3E},{0x7F,0x09,0x09,0x09,0x06},{0x3E,0x41,0x51,0x21,0x5E},{0x7F,0x09,0x19,0x29,0x46},
 {0x46,0x49,0x49,0x49,0x31},{0x01,0x01,0x7F,0x01,0x01},{0x3F,0x40,0x40,0x40,0x3F},{0x1F,0x20,0x40,0x20,0x1F},
 {0x3F,0x40,0x38,0x40,0x3F},{0x63,0x14,0x08,0x14,0x63},{0x07,0x08,0x70,0x08,0x07},{0x61,0x51,0x49,0x45,0x43}};

static void fb_char(int x, int y, char c)
{
    int idx = -1;
    if (c == ' ') idx = 0;
    else if (c >= '0' && c <= '9') idx = 1 + (c - '0');
    else if (c >= 'A' && c <= 'Z') idx = 11 + (c - 'A');
    else if (c >= 'a' && c <= 'z') idx = 11 + (c - 'a');
    if (idx < 0) return;
    for (int col = 0; col < 5; col++) {
        uint8_t cd = FONT5x7[idx][col];
        for (int row = 0; row < 7; row++) fb_set(x + col, y + row, (cd >> (6 - row)) & 1);
    }
}
/* Символ и строка в ориентации стенда: (x, y) — ВИДИМАЯ позиция.
   Этот дисплей переворачивает кадр по вертикали, поэтому пишем с flip по Y,
   а горизонталь оставляем как есть. */
static void fb_char180(int x, int y, char c)
{
    int idx = -1;
    if (c == ' ') idx = 0;
    else if (c >= '0' && c <= '9') idx = 1 + (c - '0');
    else if (c >= 'A' && c <= 'Z') idx = 11 + (c - 'A');
    else if (c >= 'a' && c <= 'z') idx = 11 + (c - 'a');
    if (idx < 0) return;
    for (int col = 0; col < 5; col++) {
        uint8_t cd = FONT5x7[idx][col];
        for (int row = 0; row < 7; row++)
            if ((cd >> (6 - row)) & 1) fb_set(x + col, 31 - (y + row), 1);
    }
}

static void fb_str180(int x, int y, const char *t)
{
    while (*t) { fb_char180(x, y, *t++); x += 6; }
}

static void fb_str(int x, int y, const char *s) { while (*s) { fb_char(x, y, *s++); x += 6; } }

/* ---------- сборка слотов сдвигового регистра ---------- */
static inline void reg_bit(uint8_t *buf, int bit1, int nbytes240)
{
    /* bit1 — номер бита по даташиту, 1..240 */
    int b = cfg.rev ? (241 - bit1) : bit1;
    int idx = b - 1;
    if (idx < 0 || idx >= 240) return;
    /* при 60 байтах данные могут лежать во второй половине */
    int base = 0;
    if (nbytes240 == 60 && cfg.dup == 2) base = 30;
    buf[base + (idx >> 3)] |= (0x80 >> (idx & 7));
}

static void build_slot(int k)
{
    uint8_t *buf = slotbuf[k];
    memset(buf, 0, MAX_BYTES);

    int g = (k * cfg.gstep) % 44;                 /* базовая сетка слота, 0-based */

    /* --- пиксели --- */
    static const int T_AFBECD_A[3] = {0, 2, 4};   /* позиции a,b,c в порядке afbecd */
    static const int T_AFBECD_D[3] = {5, 3, 1};   /* позиции d,e,f (обратный ход)   */
    static const int T_LIN_A[3]    = {0, 1, 2};
    static const int T_LIN_D[3]    = {3, 4, 5};
    const int *tA = cfg.triad ? T_LIN_A : T_AFBECD_A;
    const int *tD = cfg.triad ? T_LIN_D : T_AFBECD_D;
    int odd = (k & 1) ^ (cfg.par ? 1 : 0);
    const int *off = odd ? tD : tA;

    if (cfg.probe >= 1 && cfg.probe <= 192) {
        reg_bit(buf, cfg.probe, cfg.bytes);
    } else {
        int col0 = g * 3;
        for (int row = 0; row < 32; row++) {
            for (int i = 0; i < 3; i++) {
                int x = col0 + i;
                if (x >= 128) continue;
                int xx = cfg.mir  ? (127 - x)   : x;
                int yy = cfg.miry ? (31 - row)  : row;
                int slot = cfg.revrow ? (5 - off[i]) : off[i];   /* afbecd -> dcebfa */
                if (fb_get(xx, yy)) reg_bit(buf, row * 6 + slot + 1, cfg.bytes);
            }
        }
    }

    /* --- выбор сеток: бит 193+g, плюс сосед при dbl --- */
    if (cfg.probe >= 193 && cfg.probe <= 236) {
        reg_bit(buf, cfg.probe, cfg.bytes);
    } else {
        reg_bit(buf, 193 + g, cfg.bytes);
        if (cfg.dbl) reg_bit(buf, 193 + ((g + 1) % 44), cfg.bytes);
    }

    /* --- дублирование во вторую цепь (SIN2), если плата каскадом --- */
    if (cfg.bytes == 60 && cfg.dup == 1) memcpy(buf + 30, buf, 30);
}

/* ---------- битбенг с произвольным назначением ролей пинам ----------
   Подписи на разъёме могут не соответствовать реальным входам стекла, поэтому
   роли (data/clock/latch/blank) назначаются в рантайме командой perm. */
static GPIO_TypeDef *const BBPORT[4] = { GPIOA, GPIOA, GPIOB, GPIOB };
static const uint16_t     BBPIN[4]   = { GPIO_PIN_5, GPIO_PIN_7, GPIO_PIN_6, GPIO_PIN_4 };
static const char        *BBNAME[4]  = { "PA5", "PA7", "PB6", "PB4" };
static uint8_t bb_on = 0;
static uint8_t r_dat = 1, r_clk = 0, r_lat = 2, r_blk = 3;   /* как подписано на плате */

static inline void bb_set(uint8_t i, int v)
{
    BBPORT[i]->BSRR = v ? BBPIN[i] : (uint32_t)BBPIN[i] << 16;
}

/* Крутизна фронтов на 4 линиях. Медленные фронты = меньше звона и наводки
   CLK -> SIN на дюпонах; проверяем, не из-за них ли единица «протягивается». */
static uint32_t bb_slew = GPIO_SPEED_FREQ_VERY_HIGH;

static void bb_enable(int on)
{
    GPIO_InitTypeDef g = {0};
    if (on) {
        HAL_SPI_DeInit(&hspi1);
        g.Mode = GPIO_MODE_OUTPUT_PP; g.Pull = GPIO_NOPULL; g.Speed = bb_slew;
        g.Pin = GPIO_PIN_5 | GPIO_PIN_7; HAL_GPIO_Init(GPIOA, &g);
        g.Pin = GPIO_PIN_6 | GPIO_PIN_4; HAL_GPIO_Init(GPIOB, &g);
        bb_set(r_clk, 0);
    } else {
        SPI1_Init();
    }
    bb_on = on ? 1 : 0;
}

static void bb_send(const uint8_t *buf, int nbits)
{
    /* SysTick (1 кГц) влезал в середину сдвига и растягивал такт — от кадра к кадру
       менялось, какие биты защёлкнутся, отсюда дрожание картинки. Глушим только его,
       USART оставляем живым, чтобы не терять символы консоли. */
    uint32_t st_ctrl = SysTick->CTRL;
    SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;

    /* Полки такта задаём в ТАКТАХ ПРОЦЕССОРА, а не в nop'ах: ядро теперь 100 МГц,
       и nop'ы дали бы 40 нс вместо даташитных 200 нс. cfg.clkns — длительность
       каждой полки в наносекундах (по умолчанию 250 нс -> период 500 нс). */
    /* Всё, что можно, выносим ИЗ цикла: на 100 МГц каждое обращение к полям
       конфига и индексация массивов стоили ~1.7 мкс на бит — больше самих полок. */
    const uint32_t mhz = SystemCoreClock / 1000000U;
    uint32_t half = mhz * cfg.clkns / 1000U;
    if (half < 2) half = 2;
    const uint32_t setc = cfg.setus ? cfg.setus * mhz : half;
    const uint32_t holdc = half / 4 + 2;                  /* SIN hold >= 50 нс */
    const uint32_t bbc = cfg.bbdly ? cfg.bbdly * mhz : 0;
    const uint8_t rtz = cfg.rtz;

    GPIO_TypeDef *const dp = BBPORT[r_dat];
    GPIO_TypeDef *const cp = BBPORT[r_clk];
    const uint32_t dset = BBPIN[r_dat], dclr = (uint32_t)BBPIN[r_dat] << 16;
    const uint32_t cset = BBPIN[r_clk], cclr = (uint32_t)BBPIN[r_clk] << 16;

    for (int i = 0; i < nbits; i++) {
        dp->BSRR = ((buf[i >> 3] >> (7 - (i & 7))) & 1) ? dset : dclr;
        cyc_wait(setc);                   /* данные до фронта: >= 40 нс по даташиту */
        cp->BSRR = cset;
        cyc_wait(half);                   /* CLK high >= 200 нс */
        cp->BSRR = cclr;
        cyc_wait(holdc);
        if (rtz) dp->BSRR = dclr;         /* дальше линия падает весь остаток бита */
        cyc_wait(half);                   /* CLK low >= 200 нс */
        if (bbc) cyc_wait(bbc);
    }

    SysTick->CTRL = st_ctrl;
}

/* LAT/BLK: в битбенге — по назначенным ролям, иначе — по подписям разъёма */
static void blk_blank(void)
{
    int lvl = cfg.blkpol ? 0 : 1;
    if (bb_on) bb_set(r_blk, lvl);
    else HAL_GPIO_WritePin(GPIOB, pin_blk, lvl ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
static void blk_show(void)
{
    int lvl = cfg.blkpol ? 1 : 0;
    if (bb_on) bb_set(r_blk, lvl);
    else HAL_GPIO_WritePin(GPIOB, pin_blk, lvl ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
static void lat_pulse(void)
{
    if (cfg.nolat) return;
    int hi = cfg.latpol ? 0 : 1;
    if (bb_on) { bb_set(r_lat, hi); udelay(1); bb_set(r_lat, !hi); }
    else {
        HAL_GPIO_WritePin(GPIOB, pin_lat, hi ? GPIO_PIN_SET : GPIO_PIN_RESET);
        udelay(1);
        HAL_GPIO_WritePin(GPIOB, pin_lat, hi ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }
}

/* GCP: счётный клок ШИМ-декодера. По даташиту идёт пачкой в каждом цикле сетки. */
static void gcp_burst(void)
{
    for (int i = 0; i < cfg.gcp; i++) {
        HAL_GPIO_WritePin(GCP_PORT, GCP_PIN, GPIO_PIN_SET);
        udelay(1);
        HAL_GPIO_WritePin(GCP_PORT, GCP_PIN, GPIO_PIN_RESET);
        udelay(1);
    }
}

/* Сырой поток: rt бит, единицы на позициях [rs, rs+rn). Позиция = порядок ухода бита
   в линию (0 = уходит первым). Нужен, чтобы ИЗМЕРИТЬ длину цепи и карту полей. */
static void raw_build(void)
{
    if (raw.manual) return;             /* поток собран вручную командами rz/rb/rc */
    memset(rawbuf, 0, sizeof rawbuf);
    for (int i = raw.rs; i < raw.rs + raw.rn && i < raw.rt && i < RAW_BYTES * 8; i++)
        rawbuf[i >> 3] |= (0x80 >> (i & 7));
}

static void rebuild(void)
{
    for (int k = 0; k < cfg.slots; k++) build_slot(k);
    raw_build();
    dirty = 0;
}

/* Прогон нулями — вытолкнуть из цепи всё, что там лежит (BLK держим HIGH). */
static void flush_zeros(int nbits)
{
    uint8_t z[30] = {0};
    HAL_GPIO_WritePin(BLK_PORT, BLK_PIN, GPIO_PIN_SET);
    for (int left = nbits; left > 0; left -= 240)
        HAL_SPI_Transmit(&hspi1, z, 30, 100);
}

/* ---------- один кадр мультиплекса ---------- */
static void scan_frame(void)
{
    if (!cfg.scan) {
        HAL_GPIO_WritePin(BLK_PORT, BLK_PIN, GPIO_PIN_SET);   /* безопасный стоп */
        return;
    }
    if (raw.en) {   /* статичный сырой поток: гасим -> сдвиг rt бит -> латч -> показ */
        blk_blank();
        udelay(cfg.blk_us);                          /* BLK hold >= 10 мкс ДО сдвига */
        if (bb_on) bb_send(rawbuf, raw.rt);
        else       HAL_SPI_Transmit(&hspi1, rawbuf, (raw.rt + 7) / 8, 200);
        udelay(1);
        udelay(cfg.blk_us);
        lat_pulse();
        blk_show();
        udelay(cfg.on_us ? cfg.on_us : 200);
        if (cfg.hold) HAL_Delay(cfg.hold);      /* статика БЕЗ тактов CLK */
        return;
    }

    /* mode 3 — ТОЧНАЯ последовательность из MULTIPLEX TIMING даташита:
       сдвиг 240 бит слота (в это время светится предыдущий слот) -> пауза CLK ->
       импульс LATCH -> импульс BLK (гашение >=10 мкс) -> пачка GCP -> следующий слот.
       Т.е. LATCH идёт ПЕРЕД гашением, а показ = время сдвига следующего слота. */
    if (cfg.mode == 3) {
        for (int k = 0; k < cfg.slots; k++) {
            if (cfg.gfix >= 0 && k != cfg.gfix) continue;
            uint32_t t0 = DWT->CYCCNT;

            if (bb_on) bb_send(slotbuf[k], cfg.bytes * 8);
            else       HAL_SPI_Transmit(&hspi1, slotbuf[k], cfg.bytes, 100);

            udelay(1);                       /* CLK -> LAT >= 250 нс */
            lat_pulse();                     /* новые данные становятся активными */

            blk_blank();                     /* импульс BLK ПОСЛЕ защёлки */
            udelay(cfg.blk_us);              /* >= 10 мкс */
            blk_show();

            gcp_burst();
            if (cfg.on_us) udelay(cfg.on_us);
            slot_wait(t0);                   /* выровнять период слота */
        }
        return;
    }

    /* mode 2 — «гашение защёлкой»: на этой плате аноды ЗАЩЁЛКИВАЮТСЯ, а выбор сеток
       следует за регистром вживую, поэтому сеточный бит по пути подсвечивает все
       сетки, через которые проходит (шлейф). Лечение: сначала защёлкнуть нули в
       аноды (поле погасло), потом вдвинуть данные слота — транзит невидим, — и
       защёлкнуть уже финальное состояние. */
    if (cfg.mode == 2) {
        static const uint8_t zeros[MAX_BYTES] = {0};
        for (int k = 0; k < cfg.slots; k++) {
            if (cfg.gfix >= 0 && k != cfg.gfix) continue;

            blk_blank();
            udelay(cfg.blk_us);
            if (bb_on) bb_send(zeros, cfg.bytes * 8);
            else       HAL_SPI_Transmit(&hspi1, (uint8_t *)zeros, cfg.bytes, 100);
            udelay(1);
            lat_pulse();                       /* аноды = 0, поле погашено */

            if (bb_on) bb_send(slotbuf[k], cfg.bytes * 8);
            else       HAL_SPI_Transmit(&hspi1, slotbuf[k], cfg.bytes, 100);
            udelay(1);
            lat_pulse();                       /* финальное состояние слота */

            blk_show();
            gcp_burst();
            if (cfg.on_us) udelay(cfg.on_us);
        }
        return;
    }

    for (int k = 0; k < cfg.slots; k++) {
        if (cfg.gfix >= 0 && k != cfg.gfix) continue;
        uint32_t t0 = DWT->CYCCNT;

        if (cfg.mode == 0) {
            blk_blank();                                      /* гасим до сдвига... */
            udelay(cfg.blk_us);                               /* ...и ДАЁМ BLK HOLD >= 10 мкс */
        }

        if (bb_on) bb_send(slotbuf[k], cfg.bytes * 8);
        else       HAL_SPI_Transmit(&hspi1, slotbuf[k], cfg.bytes, 100);

        udelay(1);                                            /* CLK -> LAT >= 250 нс */
        if (cfg.mode == 1) blk_blank();
        udelay(cfg.blk_us);                                   /* BLK hold >= 10 мкс */
        lat_pulse();
        blk_show();
        gcp_burst();                                          /* даташит: GCP за цикл сетки */
        if (cfg.on_us) udelay(cfg.on_us);
        slot_wait(t0);                                        /* выровнять период слота */
    }
}

/* ---------- обратное чтение цепи по SOUT1/SOUT2 (битбенг) ---------- */
static void so_test(int nclk)
{
    GPIO_InitTypeDef g = {0};
    HAL_SPI_DeInit(&hspi1);
    g.Mode = GPIO_MODE_OUTPUT_PP; g.Pull = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Pin = GPIO_PIN_5 | GPIO_PIN_7; HAL_GPIO_Init(GPIOA, &g);

    HAL_GPIO_WritePin(BLK_PORT, BLK_PIN, GPIO_PIN_SET);       /* скан стоит -> BLK HIGH */

    /* 1) промываем цепь нулями, потом единицами — проверка самого наличия контакта */
    for (int phase = 0; phase < 2; phase++) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, phase ? GPIO_PIN_SET : GPIO_PIN_RESET);
        for (int i = 0; i < nclk; i++) {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);   udelay(1);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); udelay(1);
        }
        upf("after %s: SO1=%d SO2=%d\r\n", phase ? "ones " : "zeros",
            HAL_GPIO_ReadPin(SO1_PORT, SO1_PIN), HAL_GPIO_ReadPin(SO2_PORT, SO2_PIN));
    }
    /* вернуть нули перед измерением длины */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
    for (int i = 0; i < nclk; i++) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);   udelay(1);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); udelay(1);
    }
    /* 2) один '1', дальше нули; ищем, на каком такте он вылезет из SOUT1/SOUT2 */
    int f1 = -1, f2 = -1;
    for (int i = 0; i < nclk; i++) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, (i == 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        udelay(1);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);   udelay(1);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); udelay(1);
        int s1 = HAL_GPIO_ReadPin(SO1_PORT, SO1_PIN);
        int s2 = HAL_GPIO_ReadPin(SO2_PORT, SO2_PIN);
        if (s1 && f1 < 0) f1 = i + 1;
        if (s2 && f2 < 0) f2 = i + 1;
    }
    upf("SO1 first-1 at clk %d, SO2 first-1 at clk %d (of %d)\r\n", f1, f2, nclk);
    up(f1 < 0 && f2 < 0 ? "  -> обратка молчит: SO1/SO2 не подключены или цепь не тактируется\r\n"
                        : "  -> длина цепи = номер такта; равные значения = SIN идёт на оба регистра\r\n");
    SPI1_Init();
    dirty = 1;
}

/* ---------- обратное чтение цепи по SO1: длина + целостность ----------
   Гоним известный паттерн, потом вычитываем поток с SO1, гоня нули.
   Смещение, на котором паттерн вылезает, = длина цепи; побитное совпадение
   = данные доходят без потерь на текущей скорости такта (cfg bbdly). */
static void so_read(int nbits, int write_spi)
{
    static uint8_t cap[80];
    static const uint8_t pat[30] = {
        0xA5, 0x5A, 0xFF, 0x00, 0x01, 0x80, 0x33, 0xCC, 0x0F, 0xF0,
        0x11, 0x22, 0x44, 0x88, 0x77, 0xEE, 0x12, 0x34, 0x56, 0x78,
        0x9A, 0xBC, 0xDE, 0xF0, 0xAA, 0x55, 0xC3, 0x3C, 0x69, 0x96 };
    static const uint8_t zer[30] = {0};

    /* если поток собран вручную (rz/rb), гоняем его — удобно для точечных тестов */
    const uint8_t *src = raw.manual ? rawbuf : pat;

    if (write_spi) {          /* пишем аппаратным SPI, читаем всё равно битбенгом */
        if (bb_on) bb_enable(0);
        blk_blank();
        HAL_SPI_Transmit(&hspi1, (uint8_t *)zer, 30, 100);
        HAL_SPI_Transmit(&hspi1, (uint8_t *)zer, 30, 100);
        HAL_SPI_Transmit(&hspi1, (uint8_t *)src, 30, 100);
        bb_enable(1);
    } else {
        if (!bb_on) bb_enable(1);
        blk_blank();                               /* скан стоит -> BLK в гашение */
        bb_send(zer, 240); bb_send(zer, 240);      /* промыть цепь нулями */
        bb_send(src, 240);                         /* известный паттерн */
    }

    if (nbits > (int)sizeof(cap) * 8) nbits = sizeof(cap) * 8;
    memset(cap, 0, sizeof cap);
    /* Чтение делаем ЗАВЕДОМО медленно и симметрично (как в so_test, который даёт
       стабильные 240), чтобы мерить именно надёжность ЗАПИСИ на скорости cfg.bbdly. */
    uint32_t rd = cfg.rddly ? cfg.rddly : 2;
    for (int i = 0; i < nbits; i++) {              /* вычитываем, гоня нули */
        bb_set(r_dat, 0);
        udelay(rd);
        bb_set(r_clk, 1);
        udelay(rd);
        bb_set(r_clk, 0);
        udelay(rd);
        if (HAL_GPIO_ReadPin(SO1_PORT, SO1_PIN)) cap[i >> 3] |= (0x80 >> (i & 7));
    }

    int first = -1;
    for (int i = 0; i < nbits; i++)
        if (cap[i >> 3] & (0x80 >> (i & 7))) { first = i; break; }

    upf("sord: bbdly=%u, вычитано %d бит, первая 1 на такте %d\r\n", cfg.bbdly, nbits, first);
    up("sent: "); for (int i = 0; i < 30; i++) upf("%02X", src[i]); up("\r\n");
    up("recv: "); for (int i = 0; i < (nbits + 7) / 8; i++) upf("%02X", cap[i]); up("\r\n");
    if (first < 0) up("  -> SO1 молчит: провод не подключён либо цепь не тактируется\r\n");
    else upf("  -> сдвиг паттерна = %d бит; если данные целы, это длина цепи\r\n", first - 0);
    dirty = 1;
}

/* ---------- разбор команд ---------- */
static void cfg_set(const char *k, int v)
{
    if      (!strcmp(k, "slots"))  cfg.slots  = (uint8_t)v;
    else if (!strcmp(k, "gstep"))  cfg.gstep  = (uint8_t)v;
    else if (!strcmp(k, "dbl"))    cfg.dbl    = (uint8_t)v;
    else if (!strcmp(k, "triad"))  cfg.triad  = (uint8_t)v;
    else if (!strcmp(k, "par"))    cfg.par    = (uint8_t)v;
    else if (!strcmp(k, "mir"))    cfg.mir    = (uint8_t)v;
    else if (!strcmp(k, "miry"))   cfg.miry   = (uint8_t)v;
    else if (!strcmp(k, "rev"))    cfg.rev    = (uint8_t)v;
    else if (!strcmp(k, "bytes"))  cfg.bytes  = (v == 60) ? 60 : 30;
    else if (!strcmp(k, "dup"))    cfg.dup    = (uint8_t)v;
    else if (!strcmp(k, "mode"))   cfg.mode   = (uint8_t)v;
    else if (!strcmp(k, "on"))     cfg.on_us  = (uint16_t)v;
    else if (!strcmp(k, "blk"))    cfg.blk_us = (uint16_t)v;
    else if (!strcmp(k, "hold"))   cfg.hold   = (uint16_t)v;
    else if (!strcmp(k, "blkpol")) cfg.blkpol = (uint8_t)v;
    else if (!strcmp(k, "latpol")) cfg.latpol = (uint8_t)v;
    else if (!strcmp(k, "nolat"))  cfg.nolat  = (uint8_t)v;
    else if (!strcmp(k, "gcp"))    cfg.gcp    = (uint8_t)v;
    else if (!strcmp(k, "bbdly"))  cfg.bbdly  = (uint16_t)v;
    else if (!strcmp(k, "slotus")) cfg.slotus = (uint16_t)v;
    else if (!strcmp(k, "rtz"))    cfg.rtz    = (uint8_t)v;
    else if (!strcmp(k, "rddly"))  cfg.rddly  = (uint16_t)v;
    else if (!strcmp(k, "setus"))  cfg.setus  = (uint16_t)v;
    else if (!strcmp(k, "clkns"))  cfg.clkns  = (uint16_t)v;
    else if (!strcmp(k, "revrow")) cfg.revrow = (uint8_t)v;
    else { up("cfg? \r\n"); return; }
    dirty = 1;
    upf("cfg %s=%d\r\n", k, v);
}

static void print_st(void)
{
    upf("slots=%d gstep=%d dbl=%d triad=%d par=%d mir=%d rev=%d bytes=%d dup=%d mode=%d on=%u blk=%u gfix=%d probe=%d scan=%d\r\n"
        "hold=%u blkpol=%d latpol=%d nolat=%d raw=%d s=%u n=%u t=%u\r\n",
        cfg.slots, cfg.gstep, cfg.dbl, cfg.triad, cfg.par, cfg.mir, cfg.rev,
        cfg.bytes, cfg.dup, cfg.mode, cfg.on_us, cfg.blk_us, cfg.gfix, cfg.probe, cfg.scan,
        cfg.hold, cfg.blkpol, cfg.latpol, cfg.nolat, raw.en, raw.rs, raw.rn, raw.rt);
}

static void handle(char *line)
{
    /* Текст с пробелами разбираем до strtok: "t1 привет мир" -> строка 1.
       t1..t4 — номер строки (шрифт 5x7, 4 строки по 21 символу), cls — очистить. */
    if ((line[0] == 't' || line[0] == 'T') && line[1] >= '1' && line[1] <= '4' &&
        (line[2] == ' ' || line[2] == 0)) {
        int row = line[1] - '1';
        const char *txt = (line[2] == 0) ? "" : line + 3;
        anim = 0; demo_on = 0;
        for (int y = row * 8; y < row * 8 + 8; y++)        /* чистим только свою строку */
            for (int x = 0; x < 128; x++) fb_set(x, 31 - y, 0);
        fb_str180(1, row * 8, txt);
        upf("строка %d: %s\r\n", row + 1, txt);
        return;
    }
    if (!strncmp(line, "cls", 3)) { anim = 0; demo_on = 0; fb_clear(); up("очищено\r\n"); return; }
    char *c = strtok(line, " ");
    if (!c) return;
    char *a = strtok(NULL, " "), *b = strtok(NULL, " "), *d = strtok(NULL, " ");

    if      (!strcmp(c, "clr"))    { fb_clear(); up("ok\r\n"); }
    else if (!strcmp(c, "fill"))   { fb_fill();  up("ok\r\n"); }
    else if (!strcmp(c, "border")) { fb_clear(); fb_border(); up("ok\r\n"); }
    else if (!strcmp(c, "chk"))    { fb_clear(); fb_checker(a ? atoi(a) : 4); up("ok\r\n"); }
    else if (!strcmp(c, "hline"))  { if (a) { fb_clear(); fb_hline(atoi(a)); up("ok\r\n"); } }
    else if (!strcmp(c, "vline"))  { if (a) { fb_clear(); fb_vline(atoi(a)); up("ok\r\n"); } }
    else if (!strcmp(c, "logo"))   {   /* logo — по центру, logo m — плавает по полю */
        demo_on = 0;
        if (a && a[0] == 'm') { anim = 2; up("лого плавает\r\n"); }
        else { anim = 0; fb_logo(); up("логотип Wiren Board\r\n"); }
    }
    else if (!strcmp(c, "cube"))   {   /* cube [масштаб] — вращающийся куб */
        if (a) cube_scale = (float)atoi(a);
        anim = 1; up("куб пошёл\r\n");
    }
    else if (!strcmp(c, "demo"))   { demo_on = (a && a[0] == '0') ? 0 : 1;
                                     upf("демо=%d (куб и лого по 15 с)\r\n", demo_on); }
    else if (!strcmp(c, "walk"))   { cube_walk = (a && a[0] == '0') ? 0 : 1;
                                     upf("блуждание=%d\r\n", cube_walk); }
    else if (!strcmp(c, "stop"))   { anim = 0; demo_on = 0; up("анимация стоп\r\n"); }
    else if (!strcmp(c, "cross"))  { fb_clear(); fb_hline(8); fb_vline(30); up("ok\r\n"); }
    else if (!strcmp(c, "rect"))   {   /* rect X Y W H — залитый прямоугольник */
        char *e = strtok(NULL, " ");
        if (a && b && d && e) { fb_clear(); fb_rect(atoi(a), atoi(b), atoi(d), atoi(e), 1); up("ok\r\n"); }
        else up("rect X Y W H\r\n");
    }
    else if (!strcmp(c, "box"))    {   /* box [W H] — маленький прямоугольник в центре */
        int bw = a ? atoi(a) : 24, bh = b ? atoi(b) : 12;
        fb_clear();
        fb_rect((128 - bw) / 2, (32 - bh) / 2, bw, bh, 1);
        upf("box %dx%d в центре\r\n", bw, bh);
    }
    else if (!strcmp(c, "iv"))     {   /* инверсия кадра: проверка «плотная картинка резче» */
        for (int y = 0; y < 32; y++) for (int i = 0; i < 16; i++) fb[y][i] = (uint8_t)~fb[y][i];
        dirty = 1; up("ok\r\n");
    }
    else if (!strcmp(c, "px"))     { if (a && b) { fb_set(atoi(a), atoi(b), d ? atoi(d) : 1); up("ok\r\n"); } }
    else if (!strcmp(c, "one"))    { if (a && b) { fb_clear(); fb_set(atoi(a), atoi(b), 1); up("ok\r\n"); } }
    else if (!strcmp(c, "str"))    { if (a) { fb_clear(); fb_str(b ? atoi(a) : 2, 12, b ? b : a); up("ok\r\n"); } }
    else if (!strcmp(c, "text"))   { fb_clear(); fb_border(); fb_str(34, 12, "VFD TEST"); up("ok\r\n"); }
    else if (!strcmp(c, "cfg"))    { if (a && b) cfg_set(a, atoi(b)); else print_st(); }
    else if (!strcmp(c, "gfix"))   { cfg.gfix = a ? atoi(a) : -1; upf("gfix=%d\r\n", cfg.gfix); }
    else if (!strcmp(c, "gall"))   { cfg.gfix = -1; up("ok\r\n"); }
    else if (!strcmp(c, "probe"))  { cfg.probe = a ? atoi(a) : -1; dirty = 1; upf("probe=%d\r\n", cfg.probe); }
    else if (!strcmp(c, "noprobe")){ cfg.probe = -1; dirty = 1; up("ok\r\n"); }
    else if (!strcmp(c, "scan"))   { cfg.scan = (a && a[0] == '0') ? 0 : 1; upf("scan=%d\r\n", cfg.scan); }
    else if (!strcmp(c, "ef"))     { HAL_GPIO_WritePin(EF_PORT, EF_PIN, (a && a[0] == '0') ? GPIO_PIN_RESET : GPIO_PIN_SET); up("ok\r\n"); }
    else if (!strcmp(c, "bb"))     { bb_enable(!a || a[0] != '0'); upf("bb=%d\r\n", bb_on); }
    else if (!strcmp(c, "adc"))    {   /* adc [N] — измерить НАПРЯЖЕНИЕ на линии через PA0 (A0).
                                          Отвечает на вопрос из даташита: садится ли ноль
                                          ниже VIL = 0.7 В. Провод: точка замера -> A0. */
        int n = a ? atoi(a) : 4000;
        if (n < 100) n = 100;
        if (n > 20000) n = 20000;
        __HAL_RCC_ADC1_CLK_ENABLE();
        GPIO_InitTypeDef g = {0};
        g.Mode = GPIO_MODE_ANALOG; g.Pull = GPIO_NOPULL; g.Pin = GPIO_PIN_0;
        HAL_GPIO_Init(GPIOA, &g);
        ADC1->CR2 = 0; ADC1->SQR3 = 0;              /* канал 0 = PA0 */
        ADC1->SMPR2 = 0;                            /* 3 такта — самая быстрая выборка */
        ADC1->CR2 |= ADC_CR2_ADON;
        udelay(10);
        uint32_t mn = 4095, mx = 0, sum = 0;
        uint32_t hist_lo = 0;                       /* сколько отсчётов ниже 0.7 В */
        for (int i = 0; i < n; i++) {
            ADC1->CR2 |= ADC_CR2_SWSTART;
            while (!(ADC1->SR & ADC_SR_EOC)) { }
            uint32_t v = ADC1->DR & 0xFFF;
            if (v < mn) mn = v;
            if (v > mx) mx = v;
            sum += v;
            if (v * 3300U / 4095U < 700U) hist_lo++;
        }
        upf("АЦП на A0, %d отсчётов: мин %lu мВ, сред %lu мВ, макс %lu мВ\r\n", n,
            (unsigned long)(mn * 3300U / 4095U), (unsigned long)((sum / n) * 3300U / 4095U),
            (unsigned long)(mx * 3300U / 4095U));
        upf("ниже VIL 0.7 В: %lu%% отсчётов (норма даташита VIL <= 0.7 В)\r\n",
            (unsigned long)(hist_lo * 100U / n));
    }
    else if (!strcmp(c, "sq"))     {   /* sq [мс] — меандр на PA7 обычным GPIO, 2 кГц.
                                          Проверяет: (1) щуп реально на PA7,
                                          (2) умеет ли нога тянуть вниз быстро. */
        int ms = a ? atoi(a) : 300;
        HAL_SPI_DeInit(&hspi1);
        GPIO_InitTypeDef g = {0};
        g.Mode = GPIO_MODE_OUTPUT_PP; g.Pull = GPIO_NOPULL;
        g.Speed = GPIO_SPEED_FREQ_VERY_HIGH; g.Pin = GPIO_PIN_7;
        HAL_GPIO_Init(GPIOA, &g);
        uint32_t t0 = HAL_GetTick();
        while (HAL_GetTick() - t0 < (uint32_t)ms) {
            GPIOA->BSRR = GPIO_PIN_7;        udelay(250);
            GPIOA->BSRR = (uint32_t)GPIO_PIN_7 << 16; udelay(250);
        }
        GPIOA->BSRR = (uint32_t)GPIO_PIN_7 << 16;
        SPI1_Init();
        upf("меандр 2 кГц на PA7 %d мс, потом SPI обратно\r\n", ms);
    }
    else if (!strcmp(c, "spd"))    {   /* spd N — делитель SPI (APB2 = 100 МГц): 8 -> 12.5 МГц */
        int n = a ? atoi(a) : 8;
        static const struct { int div; uint32_t presc; } tab[] = {
            {2, SPI_BAUDRATEPRESCALER_2}, {4, SPI_BAUDRATEPRESCALER_4},
            {8, SPI_BAUDRATEPRESCALER_8}, {16, SPI_BAUDRATEPRESCALER_16},
            {32, SPI_BAUDRATEPRESCALER_32}, {64, SPI_BAUDRATEPRESCALER_64},
            {128, SPI_BAUDRATEPRESCALER_128}, {256, SPI_BAUDRATEPRESCALER_256} };
        for (unsigned i = 0; i < sizeof tab / sizeof tab[0]; i++)
            if (tab[i].div == n) spi_presc = tab[i].presc;
        bb_enable(0);                  /* уходим с битбенга на аппаратный SPI */
        upf("SPI /%d = %lu Гц, bb=%d\r\n", n,
            (unsigned long)(HAL_RCC_GetPCLK2Freq() / n), bb_on);
    }
    else if (!strcmp(c, "slew"))   {   /* slew 0..3: 0 = самые медленные фронты */
        static const uint32_t sp[4] = { GPIO_SPEED_FREQ_LOW, GPIO_SPEED_FREQ_MEDIUM,
                                        GPIO_SPEED_FREQ_HIGH, GPIO_SPEED_FREQ_VERY_HIGH };
        int i = a ? atoi(a) : 3;
        bb_slew = sp[i & 3];
        bb_enable(1);
        upf("slew=%d\r\n", i & 3);
    }
    else if (!strcmp(c, "perm"))   {   /* perm d c l b — роли по индексам 0=PA5 1=PA7 2=PB6 3=PB4 */
        if (a && strlen(a) >= 4) {
            r_dat = a[0] - '0'; r_clk = a[1] - '0'; r_lat = a[2] - '0'; r_blk = a[3] - '0';
            if (!bb_on) bb_enable(1); else bb_set(r_clk, 0);
        }
        upf("perm dat=%s clk=%s lat=%s blk=%s\r\n", BBNAME[r_dat & 3], BBNAME[r_clk & 3],
            BBNAME[r_lat & 3], BBNAME[r_blk & 3]);
    }
    else if (!strcmp(c, "pins"))   {   /* pins 0 = как подписано, 1 = LAT/BLK местами */
        int sw = a ? atoi(a) : 0;
        pin_lat = sw ? BLK_PIN : LAT_PIN;
        pin_blk = sw ? LAT_PIN : BLK_PIN;
        upf("pins swap=%d (lat=PB%d blk=PB%d)\r\n", sw, sw ? 4 : 6, sw ? 6 : 4);
    }
    else if (!strcmp(c, "hv"))     {   /* hv 0|1|z — PB9: логический уровень или Hi-Z */
        GPIO_InitTypeDef g = {0};
        g.Pin = GPIO_PIN_9; g.Pull = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_LOW;
        if (a && a[0] == 'z') { g.Mode = GPIO_MODE_INPUT; HAL_GPIO_Init(GPIOB, &g); up("hv=z\r\n"); }
        else {
            g.Mode = GPIO_MODE_OUTPUT_PP; HAL_GPIO_Init(GPIOB, &g);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, (a && a[0] == '0') ? GPIO_PIN_RESET : GPIO_PIN_SET);
            upf("hv=%c\r\n", (a && a[0] == '0') ? '0' : '1');
        }
    }
    else if (!strcmp(c, "so"))     { so_test(a ? atoi(a) : 600); }
    else if (!strcmp(c, "sord"))   {   /* sord [бит] [spi] — вторым словом переключаем запись на SPI */
        so_read(a ? atoi(a) : 560, (b && b[0] == 's') ? 1 : 0);
    }
    else if (!strcmp(c, "raw"))    {
        if (a) raw.rs = (uint16_t)atoi(a);
        if (b) raw.rn = (uint16_t)atoi(b);
        if (d) raw.rt = (uint16_t)atoi(d);
        if (raw.rt > RAW_BYTES * 8) raw.rt = RAW_BYTES * 8;
        raw.en = 1; raw.manual = 0; dirty = 1;   /* выходим из ручного режима rz/rb */
        upf("raw s=%u n=%u t=%u\r\n", raw.rs, raw.rn, raw.rt);
    }
    else if (!strcmp(c, "noraw"))  { raw.en = 0; raw.manual = 0; dirty = 1; up("ok\r\n"); }
    else if (!strcmp(c, "rz"))     {   /* rz [total] — обнулить поток, ручной режим */
        if (a) raw.rt = (uint16_t)atoi(a);
        if (raw.rt > RAW_BYTES * 8) raw.rt = RAW_BYTES * 8;
        memset(rawbuf, 0, sizeof rawbuf);
        raw.en = 1; raw.manual = 1;
        upf("rz t=%u\r\n", raw.rt);
    }
    else if (!strcmp(c, "rb") || !strcmp(c, "rc")) {   /* rb/rc s n — взвести/сбросить биты */
        int s0 = a ? atoi(a) : 0, n0 = b ? atoi(b) : 1, set = (c[1] == 'b');
        for (int i = s0; i < s0 + n0 && i < RAW_BYTES * 8; i++) {
            if (set) rawbuf[i >> 3] |=  (0x80 >> (i & 7));
            else     rawbuf[i >> 3] &= ~(0x80 >> (i & 7));
        }
        raw.en = 1; raw.manual = 1;
        up("ok\r\n");
    }
    else if (!strcmp(c, "flush"))  { flush_zeros(a ? atoi(a) : 960); up("ok\r\n"); }
    else if (!strcmp(c, "st"))     { print_st(); }
    else if (!strcmp(c, "clk"))    {   /* какие частоты реально получились */
        upf("SYSCLK=%lu HCLK=%lu PCLK1=%lu PCLK2=%lu Гц\r\n",
            (unsigned long)HAL_RCC_GetSysClockFreq(), (unsigned long)HAL_RCC_GetHCLKFreq(),
            (unsigned long)HAL_RCC_GetPCLK1Freq(), (unsigned long)HAL_RCC_GetPCLK2Freq());
        upf("SystemCoreClock=%lu, флеш latency=%lu\r\n",
            (unsigned long)SystemCoreClock, (unsigned long)(FLASH->ACR & FLASH_ACR_LATENCY));
    }
    else if (!strcmp(c, "help"))   {
        up("clr fill border chk N hline Y vline X cross px X Y V one X Y str X TXT text\r\n"
           "cfg [k v] | k: slots gstep dbl triad par mir rev bytes dup mode on blk\r\n"
           "gfix N gall probe N noprobe scan 0|1 ef 0|1 so [nclk] st\r\n");
    }
    else up("?\r\n");
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();       /* 100 МГц: без этого битбенг не выдаёт 100 Гц кадра */
    dwt_init();
    GPIO_Init();
    SPI1_Init();
    USART2_Init();

    /* стартовая рабочая точка стенда: SPI 12.5 МГц, кадр 10 мс = 100 Гц */
    cfg.mode = 0; cfg.mir = 0; cfg.miry = 0;
    cfg.on_us = 180; cfg.slotus = 227;
    fb_logo();

    up("\r\nMN12832L datasheet-driver, 44 slots/pair-overlap, abc/def\r\n");
    print_st();
    up("> ");

    char line[80]; int li = 0; uint8_t ch;
    uint32_t t_anim = 0;
    while (1) {
        if (demo_on) {                                /* демо: 15 с куб, 15 с лого */
            static uint32_t t_scene = 0;
            if (t_scene == 0) { t_scene = HAL_GetTick(); anim = 1; }
            if (HAL_GetTick() - t_scene >= DEMO_MS) {
                t_scene = HAL_GetTick();
                anim = (anim == 1) ? 2 : 1;
            }
        }
        if (anim && HAL_GetTick() - t_anim >= 40) {   /* 25 кадров в секунду */
            t_anim = HAL_GetTick();
            if (anim == 2) { logo_move(); fb_logo_at((int)logo_x, (int)logo_y); }
            else { cube_ang += 0.10f; cube_move(); fb_cube(cube_ang); }
        }
        if (dirty) rebuild();
        scan_frame();
        while (rx_get(&ch)) {
            if (ch == '\r' || ch == '\n') { if (li) { line[li] = 0; handle(line); li = 0; } up("> "); }
            else if (ch == 8 || ch == 127) { if (li) { li--; up("\b \b"); } }
            else if (li < 79) { line[li++] = (char)ch;
                                HAL_UART_Transmit(&huart2, &ch, 1, 100); }
        }
    }
}

