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
#include "fonts.h"
#include "digits.h"

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
#define MAX_BYTES 30          /* 240 бит — длина цепи измерена обраткой SO1 */

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
    uint8_t mode;    /* 0 = гасим-сдвигаем-латчим; 1 = сдвиг во время показа (по даташиту) */
    uint16_t on_us;  /* доп. время показа в слоте, мкс                            */
    uint16_t blk_us; /* длительность гашения, мкс (>=10 по даташиту)              */
    uint16_t slotus; /* ФИКСИРОВАННЫЙ период слота, мкс (0 = как получится).
                        Выравнивает кадр: убирает дрожание от прерываний и от
                        разной длины кода в слотах. Даташит: слот ~100 мкс.     */
    uint8_t  revrow; /* 1 = реверс шести бит внутри строки: afbecd -> dcebfa
                        (так у mariosgit; у нас проверяем оба варианта)          */
} cfg = { 44, 1, 1, 0, 0, 1, 1, 0, 30, 1, 0, 12, 250, 0 };
/* slots gstep dbl triad par mir miry rev bytes mode on_us blk_us slotus revrow */

static void blk_blank(void);
static void blk_show(void);
static void lat_pulse(void);

static uint8_t slotbuf[MAX_SLOTS][MAX_BYTES];
static volatile uint8_t dirty = 1;


/* ---------- консоль ---------- */
#define RXSZ 256
static volatile uint8_t rxbuf[RXSZ];
static volatile uint16_t rxhead = 0, rxtail = 0;
static uint8_t rxbyte;

/* исходящий кольцевой буфер: передача идёт по прерыванию TXE,
   чтобы вывод не тормозил разбор входящих байтов */
#define TXSZ 256
static volatile uint8_t txbuf[TXSZ];
static volatile uint16_t txhead = 0, txtail = 0;

void SysTick_Handler(void)  { HAL_IncTick(); }
void USART2_IRQHandler(void)
{
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TXE) &&
        __HAL_UART_GET_IT_SOURCE(&huart2, UART_IT_TXE)) {
        if (txtail != txhead) {
            huart2.Instance->DR = txbuf[txtail];
            txtail = (uint16_t)((txtail + 1) % TXSZ);
        } else {
            __HAL_UART_DISABLE_IT(&huart2, UART_IT_TXE);
        }
    }
    HAL_UART_IRQHandler(&huart2);
}
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
/* ---------- неблокирующая передача ----------
   Блокирующий HAL_UART_Transmit в цикле приёма тормозил разбор входящих байтов,
   и на русских буквах (два байта на символ) в строку влезал мусор. Теперь всё
   исходящее складывается в кольцевой буфер и уходит по прерыванию TXE. */

static void tx_push(const char *d, int n)
{
    for (int i = 0; i < n; i++) {
        uint16_t nx = (uint16_t)((txhead + 1) % TXSZ);
        while (nx == txtail) { }                  /* буфер полон — ждём передатчик */
        txbuf[txhead] = (uint8_t)d[i];
        txhead = nx;
    }
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_TXE);
}

static void up(const char *s) { tx_push(s, (int)strlen(s)); }
static void upf(const char *f, ...)
{
    char b[128]; va_list a; va_start(a, f);
    int n = vsnprintf(b, sizeof b, f, a); va_end(a);
    if (n > 0) tx_push(b, n);
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

/* Догнать фиксированный период слота: убирает дрожание кадра */
static void slot_wait(uint32_t t0)
{
    if (!cfg.slotus) return;
    uint32_t need = cfg.slotus * (SystemCoreClock / 1000000U);
    while ((DWT->CYCCNT - t0) < need) { }
}

/* ---------- история команд ----------
   Последние 10 введённых строк, стрелки вверх и вниз листают их, как в оболочке.
   Терминал присылает стрелку тремя байтами: ESC, '[', 'A' или 'B'. */
#define HIST_N 5
static char hist[HIST_N][40];
static uint8_t hist_cnt = 0;      /* сколько всего запомнено */
static int8_t hist_sel = -1;      /* -1 = набираем новое, иначе индекс в истории */

static void hist_add(const char *l)
{
    if (!l[0]) return;
    if (hist_cnt && !strcmp(hist[0], l)) return;      /* не дублируем подряд */
    for (int i = HIST_N - 1; i > 0; i--) memcpy(hist[i], hist[i - 1], 40);
    strncpy(hist[0], l, 39); hist[0][39] = 0;
    if (hist_cnt < HIST_N) hist_cnt++;
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

/* ---------- часы крупными цифрами ----------
   Два набора цифр (profont29 14x19 и 7Segments 26x32), только 0..9, двоеточие
   и пробел. Ход времени — от системного таймера, батарейного хода у платы нет,
   поэтому после сброса время выставляется заново командой time. */
static uint8_t font_id;                     /* определение ниже, у шрифтов */
static void fb_text(int x, int y, const char *t);

/* дата под часами: месяц словами, строка центрируется мелким шрифтом */
static uint8_t dt_d = 26, dt_mo = 7;
static uint16_t dt_y = 2026;
static const char *MONTHS[12] = { "января", "февраля", "марта", "апреля", "мая", "июня",
                                  "июля", "августа", "сентября", "октября", "ноября", "декабря" };
static const uint8_t MDAYS[12] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

static void date_next(void)                 /* следующий день, с учётом февраля */
{
    uint8_t last = MDAYS[(dt_mo - 1) % 12];
    if (dt_mo == 2 && ((dt_y % 4 == 0 && dt_y % 100 != 0) || dt_y % 400 == 0)) last = 29;
    if (++dt_d > last) { dt_d = 1; if (++dt_mo > 12) { dt_mo = 1; dt_y++; } }
}

static uint8_t clk_h = 12, clk_m = 0, clk_s = 0;
static uint32_t clk_tick = 0;               /* метка последней секунды */
static uint8_t clk_set = 0;                 /* какой набор цифр, см. DIGITS */

static void fb_digits(int x, int y, const char *t)
{
    const digits_t *d = &DIGITS[clk_set];
    for (; *t && x < 128; t++) {
        int gi = (*t >= '0' && *t <= '9') ? *t - '0' : (*t == ':' ? 10 : 11);
        const uint8_t *g = d->data + (size_t)gi * d->h * d->by;
        for (int row = 0; row < d->h; row++)
            for (int col = 0; col < d->w; col++)
                if ((g[row * d->by + (col >> 3)] >> (7 - (col & 7))) & 1)
                    fb_set(x + col, 31 - (y + (d->h - 1 - row)), 1);
        x += d->w + 1;
    }
}

static void clock_tick(void)                /* отсчёт секунд */
{
    uint32_t now = HAL_GetTick();
    while (now - clk_tick >= 1000U) {
        clk_tick += 1000U;
        if (++clk_s >= 60) { clk_s = 0; if (++clk_m >= 60) { clk_m = 0; if (++clk_h >= 24) { clk_h = 0; date_next(); } } }
    }
}

/* Раскладка зависит от набора цифр. Ноль по Y у этого поля ВНИЗУ.
   Набор 0 (14x19) — единственный, под которым снизу остаётся место на дату
   шрифтом 7x13 (19 + 13 = 32 ровно). Остальные наборы выше, поэтому идут без
   даты и центруются по вертикали; секунды показываем только если строка из
   восьми знаков влезает в 128 точек (поле secs в наборе). */
static void clock_draw(void)
{
    const digits_t *d = &DIGITS[clk_set];
    const font_t *f = &FONTS[2];
    char buf[16];
    int per = d->w + 1;
    int with_date = (d->h + f->h <= 32);

    if (d->secs) snprintf(buf, sizeof buf, "%02u:%02u:%02u", clk_h, clk_m, clk_s);
    else         snprintf(buf, sizeof buf, "%02u:%02u", clk_h, clk_m);
    int wid = (int)strlen(buf) * per - 1;

    fb_clear();
    fb_digits((128 - wid) / 2, with_date ? 32 - d->h : (32 - d->h) / 2, buf);
    if (!with_date) return;

    /* дата по центру снизу, месяц словами: "26 июля" */
    snprintf(buf, sizeof buf, "%u %s", dt_d, MONTHS[(dt_mo - 1) % 12]);
    uint8_t save = font_id;
    font_id = 2;
    int tw = 0;
    for (const char *q = buf; *q; q++) if (((uint8_t)*q & 0xC0) != 0x80) tw += f->w + 1;
    fb_text((128 - tw) / 2 > 0 ? (128 - tw) / 2 : 0, 0, buf);
    font_id = save;
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




/* ---------- текст выбранным шрифтом ----------
   Три шрифта на выбор (5x8, 6x10, 7x13), латиница и кириллица.
   Индексы глифов: 0..94 = ASCII 32..126, 95 = Ё, 96..159 = А..я, 160 = ё.
   Дисплей переворачивает кадр по вертикали, поэтому пишем в 31 - y. */
static uint8_t font_id = 1;

static int glyph_index(uint32_t cp)
{
    if (cp >= 32 && cp <= 126) return cp - 32;
    if (cp == 0x401) return 95;             /* Ё */
    if (cp >= 0x410 && cp <= 0x44F) return 96 + (cp - 0x410);
    if (cp == 0x451) return 160;            /* ё */
    return -1;
}

/* Одна буква в видимой позиции (x, y) */
static int fb_glyph(int x, int y, uint32_t cp)
{
    const font_t *f = &FONTS[font_id % 3];
    int gi = glyph_index(cp);
    if (gi < 0) return f->w + 1;
    const uint8_t *g = f->data + (size_t)gi * f->h;
    for (int row = 0; row < f->h; row++)
        for (int col = 0; col < f->w; col++)
            if ((g[row] >> (7 - col)) & 1)
                fb_set(x + col, 31 - (y + (f->h - 1 - row)), 1);
    return f->w + 1;                        /* шаг до следующего символа */
}

/* Строка UTF-8: русская буква приходит двумя байтами, разбираем на месте */
static void fb_text(int x, int y, const char *t)
{
    while (*t) {
        uint32_t cp = (uint8_t)*t++;
        if (cp >= 0xC0 && *t) {             /* двухбайтовая последовательность */
            uint32_t b2 = (uint8_t)*t++;
            cp = ((cp & 0x1F) << 6) | (b2 & 0x3F);
        }
        x += fb_glyph(x, y, cp);
        if (x >= 128) break;
    }
}


/* ---------- сборка слотов сдвигового регистра ---------- */
static inline void reg_bit(uint8_t *buf, int bit1, int nbytes240)
{
    /* bit1 — номер бита по даташиту, 1..240 */
    int b = cfg.rev ? (241 - bit1) : bit1;
    int idx = b - 1;
    if (idx < 0 || idx >= 240) return;
    buf[idx >> 3] |= (0x80 >> (idx & 7));
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

    /* --- выбор сеток: бит 193+g, плюс сосед при dbl --- */
    reg_bit(buf, 193 + g, cfg.bytes);
    if (cfg.dbl) reg_bit(buf, 193 + ((g + 1) % 44), cfg.bytes);

}






/* LAT/BLK: BLK HIGH гасит поле, LATCH переносит регистр в защёлки */
static void blk_blank(void)
{
    HAL_GPIO_WritePin(BLK_PORT, BLK_PIN, GPIO_PIN_SET);      /* HIGH гасит */
}
static void blk_show(void)
{
    HAL_GPIO_WritePin(BLK_PORT, BLK_PIN, GPIO_PIN_RESET);    /* LOW показывает */
}
static void lat_pulse(void)
{
    HAL_GPIO_WritePin(LAT_PORT, LAT_PIN, GPIO_PIN_SET);
    udelay(1);                                              /* LAT high >= 300 нс */
    HAL_GPIO_WritePin(LAT_PORT, LAT_PIN, GPIO_PIN_RESET);
}



static void rebuild(void)
{
    for (int k = 0; k < cfg.slots; k++) build_slot(k);
    dirty = 0;
}


/* ---------- один кадр мультиплекса ---------- */
static void scan_frame(void)
{
    /* Кадр мультиплекса, последовательность из MULTIPLEX TIMING даташита:
       гашение -> сдвиг 240 бит слота -> LATCH -> показ. Период слота выровнен
       по cfg.slotus, поэтому кадр из 44 слотов держит заданные 100 Гц. */
    for (int k = 0; k < cfg.slots; k++) {
        uint32_t t0 = DWT->CYCCNT;

        if (cfg.mode == 0) {
            blk_blank();                                      /* гасим до сдвига... */
            udelay(cfg.blk_us);                               /* ...BLK hold >= 10 мкс */
        }

        HAL_SPI_Transmit(&hspi1, slotbuf[k], cfg.bytes, 100);

        udelay(1);                                            /* CLK -> LAT >= 250 нс */
        if (cfg.mode == 1) blk_blank();
        udelay(cfg.blk_us);
        lat_pulse();
        blk_show();
        if (cfg.on_us) udelay(cfg.on_us);
        slot_wait(t0);                                        /* период слота */
    }
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
    else if (!strcmp(k, "mode"))   cfg.mode   = (uint8_t)v;
    else if (!strcmp(k, "on"))     cfg.on_us  = (uint16_t)v;
    else if (!strcmp(k, "blk"))    cfg.blk_us = (uint16_t)v;
    else if (!strcmp(k, "slotus")) cfg.slotus = (uint16_t)v;
    else if (!strcmp(k, "revrow")) cfg.revrow = (uint8_t)v;
    else { up("cfg? \r\n"); return; }
    dirty = 1;
    upf("cfg %s=%d\r\n", k, v);
}

static void print_st(void)
{
    /* двумя вызовами: в буфер upf (128 байт) обе строки сразу не влезают,
       а обрыв на середине UTF-8 символа даёт мусор в терминале */
    upf("slots=%d gstep=%d dbl=%d triad=%d par=%d mir=%d miry=%d rev=%d revrow=%d\r\n",
        cfg.slots, cfg.gstep, cfg.dbl, cfg.triad, cfg.par, cfg.mir, cfg.miry,
        cfg.rev, cfg.revrow);
    upf("bytes=%d mode=%d on=%u blk=%u slot=%u мкс, кадр %u мкс\r\n",
        cfg.bytes, cfg.mode, cfg.on_us, cfg.blk_us,
        cfg.slotus, (unsigned)(cfg.slotus * cfg.slots));
}

static void handle(char *line)
{
    /* Текст с пробелами разбираем до strtok: "t1 привет мир" -> строка 1.
       t1..t4 — номер строки (шрифт 5x7, 4 строки по 21 символу), cls — очистить. */
    if ((line[0] == 't' || line[0] == 'T') && line[1] >= '1' && line[1] <= '4' &&
        (line[2] == ' ' || line[2] == 0)) {
        const font_t *f = &FONTS[font_id % 3];
        int step = f->h + 1;
        int row = line[1] - '1';
        const char *txt = (line[2] == 0) ? "" : line + 3;
        anim = 0; demo_on = 0;
        for (int y = row * step; y < row * step + step && y < 32; y++)
            for (int x = 0; x < 128; x++) fb_set(x, 31 - y, 0);
        fb_text(1, row * step, txt);
        upf("строка %d шрифтом %s: %s\r\n", row + 1, f->name, txt);
        return;
    }
    /* at X Y ТЕКСТ — вывод из произвольной точки, экран не чистим */
    if (!strncmp(line, "at ", 3)) {
        int x = 0, y = 0, n = 0;
        const char *q = line + 3;
        while (*q == ' ') q++;
        while (*q >= '0' && *q <= '9') { x = x * 10 + (*q++ - '0'); n++; }
        while (*q == ' ') q++;
        while (*q >= '0' && *q <= '9') { y = y * 10 + (*q++ - '0'); n++; }
        while (*q == ' ') q++;
        anim = 0; demo_on = 0;
        if (n) fb_text(x, y, q);
        upf("текст в (%d,%d): %s\r\n", x, y, q);
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
    else if (!strcmp(c, "time"))   {   /* time ЧЧ:ММ:СС — выставить время */
        if (a) {
            unsigned h = 0, m = 0, sec = 0;
            if (sscanf(a, "%u:%u:%u", &h, &m, &sec) >= 2) {
                clk_h = (uint8_t)(h % 24); clk_m = (uint8_t)(m % 60); clk_s = (uint8_t)(sec % 60);
                clk_tick = HAL_GetTick();
            }
        }
        upf("время %02u:%02u:%02u\r\n", clk_h, clk_m, clk_s);
    }
    else if (!strcmp(c, "clock"))  {   /* clock — часы с датой, clock - выключить */
        if (a && a[0] == '-') { anim = 0; up("часы выключены\r\n"); }
        else {
            if (a && a[0] >= '0' && a[0] <= '9') clk_set = (uint8_t)(atoi(a) % DIGITS_COUNT);
            const digits_t *d = &DIGITS[clk_set];
            demo_on = 0; anim = 3; clk_tick = HAL_GetTick();
            upf("часы %02u:%02u:%02u набором %u %s (%dx%d)%s\r\n", clk_h, clk_m, clk_s,
                clk_set, d->name, d->w, d->h,
                (d->h + FONTS[2].h <= 32) ? ", снизу дата" : ", без даты");
        }
    }
    else if (!strcmp(c, "date"))   {   /* date ДД.ММ.ГГГГ */
        if (a) {
            unsigned dd = 0, mm = 0, yy = 0;
            if (sscanf(a, "%u.%u.%u", &dd, &mm, &yy) >= 2) {
                dt_d = (uint8_t)(dd ? dd : 1);
                dt_mo = (uint8_t)((mm >= 1 && mm <= 12) ? mm : 1);
                if (yy) dt_y = (uint16_t)yy;
            }
        }
        upf("дата %u %s %u\r\n", dt_d, MONTHS[(dt_mo - 1) % 12], dt_y);
    }
    else if (!strcmp(c, "font"))   {   /* font 0|1|2 — 5x8, 6x10, 7x13 */
        if (a) font_id = (uint8_t)(atoi(a) % 3);
        const font_t *f = &FONTS[font_id % 3];
        upf("шрифт %d = %s: %d строк по %d символов\r\n", font_id, f->name,
            32 / (f->h + 1), 128 / (f->w + 1));
    }
    else if (!strcmp(c, "pxc"))    {   /* pxc X Y — погасить точку */
        if (a && b) { anim = 0; demo_on = 0; fb_set(atoi(a), 31 - atoi(b), 0); up("ok\r\n"); }
    }
    else if (!strcmp(c, "line"))   {   /* line X0 Y0 X1 Y1 — линия от точки до точки */
        char *e = strtok(NULL, " ");
        if (a && b && d && e) {
            anim = 0; demo_on = 0;
            int x0 = atoi(a), y0 = atoi(b), x1 = atoi(d), y1 = atoi(e);
            fb_line(x0, 31 - y0, x1, 31 - y1);
            upf("линия (%d,%d)-(%d,%d)\r\n", x0, y0, x1, y1);
        } else up("line X0 Y0 X1 Y1\r\n");
    }
    else if (!strcmp(c, "iv"))     {   /* инверсия кадра: проверка «плотная картинка резче» */
        for (int y = 0; y < 32; y++) for (int i = 0; i < 16; i++) fb[y][i] = (uint8_t)~fb[y][i];
        dirty = 1; up("ok\r\n");
    }
    else if (!strcmp(c, "px"))     {   /* px X Y [0|1] — точка в видимых координатах */
        if (a && b) { anim = 0; demo_on = 0;
                      fb_set(atoi(a), 31 - atoi(b), d ? atoi(d) : 1); up("ok\r\n"); }
    }
    else if (!strcmp(c, "str"))    { if (a) { fb_clear(); fb_text(b ? atoi(a) : 2, 12, b ? b : a); up("ok\r\n"); } }
    else if (!strcmp(c, "cfg"))    { if (a && b) cfg_set(a, atoi(b)); else print_st(); }
    else if (!strcmp(c, "ef"))     { HAL_GPIO_WritePin(EF_PORT, EF_PIN, (a && a[0] == '0') ? GPIO_PIN_RESET : GPIO_PIN_SET); up("ok\r\n"); }
    else if (!strcmp(c, "spd"))    {   /* spd N — делитель SPI (APB2 = 100 МГц): 8 -> 12.5 МГц */
        int n = a ? atoi(a) : 8;
        static const struct { int div; uint32_t presc; } tab[] = {
            {2, SPI_BAUDRATEPRESCALER_2}, {4, SPI_BAUDRATEPRESCALER_4},
            {8, SPI_BAUDRATEPRESCALER_8}, {16, SPI_BAUDRATEPRESCALER_16},
            {32, SPI_BAUDRATEPRESCALER_32}, {64, SPI_BAUDRATEPRESCALER_64},
            {128, SPI_BAUDRATEPRESCALER_128}, {256, SPI_BAUDRATEPRESCALER_256} };
        for (unsigned i = 0; i < sizeof tab / sizeof tab[0]; i++)
            if (tab[i].div == n) spi_presc = tab[i].presc;
        upf("SPI /%d = %lu Гц\r\n", n, (unsigned long)(HAL_RCC_GetPCLK2Freq() / n));
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
    else if (!strcmp(c, "st"))     { print_st(); }
    else if (!strcmp(c, "help") || !strcmp(c, "?")) {
        up("\r\n=== MN12832L, 128x32: команды консоли (115200, Enter в конце) ===\r\n");
        up("\r\nТЕКСТ\r\n");
        up("  t1..t4 ТЕКСТ  печать в строку (сколько строк — зависит от шрифта)\r\n");
        up("  at X Y ТЕКСТ  вывод из произвольной точки, экран не чистится\r\n");
        up("  str [X] ТЕКСТ вывод текущим шрифтом с очисткой экрана\r\n");
        up("  font 0|1|2    шрифт: 5x8 (4 строки), 6x10 (3), 7x13 (2), кириллица есть\r\n");
        up("  cls           очистить экран\r\n");
        up("\r\nЧАСЫ\r\n");
        up("  time ЧЧ:ММ:СС выставить время | date ДД.ММ.ГГГГ  выставить дату\r\n");
        up("  clock [N]     часы, N = набор цифр 0..3:\r\n");
        up("                0 = 14x19 с датой снизу, 1 = 13x25 с секундами,\r\n");
        up("                2 = 17x24 жирные, 3 = 17x29 самые высокие\r\n");
        up("  clock -       выключить часы\r\n");
        up("\r\nКАРТИНКИ И АНИМАЦИЯ\r\n");
        up("  logo          логотип Wiren Board по центру | logo m  плавает по полю\r\n");
        up("  cube [N]      вращающийся куб, N = масштаб (по умолчанию 17)\r\n");
        up("  walk 0|1      блуждание куба по полю\r\n");
        up("  demo 0|1      чередование куба и лого по 15 секунд\r\n");
        up("  stop          остановить анимацию\r\n");
        up("\r\nГРАФИКА\r\n");
        up("  clr fill      очистить / залить всё поле | iv  инверсия кадра\r\n");
        up("  px X Y [0|1]  точка | pxc X Y  погасить точку\r\n");
        up("  line X0 Y0 X1 Y1  линия от точки до точки\r\n");
        up("  hline Y       горизонтальная линия | vline X  вертикальная\r\n");
        up("  border        рамка по краю | box [W H]  прямоугольник в центре\r\n");
        up("  rect X Y W H  залитый прямоугольник | chk N  шахматка клеткой N\r\n");
        up("\r\nДРАЙВ ДИСПЛЕЯ\r\n");
        up("  st            показать конфигурацию и время кадра\r\n");
        up("  spd N         делитель SPI от 100 МГц: 8 = 12.5 МГц\r\n");
        up("  ef 0|1        накал | hv 0|1|z  вход HV (ни на что не влияет)\r\n");
        up("  cfg КЛЮЧ ЗНАЧЕНИЕ:\r\n");
        up("    mode 0|1    0 = гасим на сдвиг, 1 = латч в гашении\r\n");
        up("    on МКС      окно показа в слоте | slotus МКС  период слота\r\n");
        up("    blk МКС     длительность гашения (даташит >= 10)\r\n");
        up("    slots N     слотов в кадре (44) | gstep N  шаг сеток (1)\r\n");
        up("    dbl 0|1     пара сеток | triad/par/rev/revrow  раскладка бит\r\n");
        up("    mir/miry    зеркало по X / по Y (на этом стенде 0 и 0)\r\n");
        up("\r\nРАБОЧАЯ ТОЧКА (выставлена при старте): spd 8, mode 0, on 180,\r\n");
        up("slotus 227 -> кадр 10 мс = 100 Гц. Стрелка вверх — прошлые команды.\r\n");
        up("ОБЯЗАТЕЛЬНО: подтяжка линии SIN к земле 1 кОм, иначе биты размазываются.\r\n");
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
    cfg.on_us = 180; cfg.slotus = 227; font_id = 1;
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
            if (anim == 3) { clock_tick(); clock_draw(); }
            else if (anim == 2) { logo_move(); fb_logo_at((int)logo_x, (int)logo_y); }
            else { cube_ang += 0.10f; cube_move(); fb_cube(cube_ang); }
        }
        if (dirty) rebuild();
        scan_frame();
        while (rx_get(&ch)) {
            /* стрелки: ESC [ A вверх, ESC [ B вниз */
            static uint8_t esc = 0;
            if (ch == 27) { esc = 1; continue; }
            if (esc == 1) { esc = (ch == '[') ? 2 : 0; continue; }
            if (esc == 2) {
                esc = 0;
                if (ch == 'A' || ch == 'B') {
                    int8_t ns = hist_sel + (ch == 'A' ? 1 : -1);
                    if (ns < -1) ns = -1;
                    if (ns > (int8_t)hist_cnt - 1) ns = (int8_t)hist_cnt - 1;
                    hist_sel = ns;
                    while (li) { up("\b \b"); li--; }          /* стираем набранное */
                    if (hist_sel >= 0) {
                        strncpy(line, hist[hist_sel], 39); line[39] = 0;
                        li = (int)strlen(line);
                        up(line);
                    }
                }
                continue;
            }
            if (ch == '\r' || ch == '\n') {
                if (li) { line[li] = 0; hist_add(line); handle(line); li = 0; }
                hist_sel = -1;
                up("> ");
            }
            else if (ch == 8 || ch == 127) { if (li) { li--; up("\b \b"); } }
            else if (li < 79) { line[li++] = (char)ch; tx_push((char *)&ch, 1); }
        }
    }
}

