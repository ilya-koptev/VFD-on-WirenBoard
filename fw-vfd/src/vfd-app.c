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
#include "wbmcu_system.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>
#include "logo.h"
#include "fonts.h"
#include "digits.h"
#include "regmap-structs.h"     /* регкарта — общая с загрузчиком и утилитами */
#include "wbfw.h"               /* стандартные адреса ВБ и протокол обновления */

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





static void up(const char *s) { uart_push(s, (int)strlen(s)); }
static void upf(const char *f, ...)
{
    char b[128]; va_list a; va_start(a, f);
    int n = vsnprintf(b, sizeof b, f, a); va_end(a);
    if (n > 0) uart_push(b, n);
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




static uint32_t spi_div = 8;



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

/* Индекс глифа крупного набора по символу. Порядок задан DIGIT_CHARS в
   digits.h, поэтому набор и код не могут разъехаться. Знак градуса приходит в
   UTF-8 двумя байтами (0xC2 0xB0) — съедаем второй сами. Неизвестный знак
   выводим пробелом: в наборах только цифры и знаки при них. */
static int digit_glyph_in(const digits_t *d, const char **pt)
{
    const unsigned char *p = (const unsigned char *)*pt;
    if (p[0] == 0xC2 && p[1] == 0xB0) { (*pt)++; return d->deg; }   /* градус, UTF-8 */
    const char *q = strchr(d->chars, (char)p[0]);
    return (q && *q) ? (int)(q - d->chars) : DIGIT_SPACE_INDEX;
}

/* Наборы цифр доступны как обычные шрифты: 0..2 — текстовые, дальше цифровые.
   Так большое число можно поставить в любую строку, а не только в часы. */
#define FONT_TEXT_COUNT 3
#define FONT_COUNT      (FONT_TEXT_COUNT + DIGITS_COUNT)

static int font_height(int id)
{
    return (id < FONT_TEXT_COUNT) ? FONTS[id].h : DIGITS[id - FONT_TEXT_COUNT].h;
}

static const char *font_name(int id)
{
    return (id < FONT_TEXT_COUNT) ? FONTS[id].name : DIGITS[id - FONT_TEXT_COUNT].name;
}

/* Вывод набором цифр: знаки 0..9, двоеточие и пробел, остальное — пробел */
static void fb_digits_set(int x, int y, const char *t, int set)
{
    const digits_t *d = &DIGITS[set % DIGITS_COUNT];
    for (; *t && x < 128; t++) {
        int gi = digit_glyph_in(d, &t);
        const uint8_t *g = d->data + (size_t)gi * d->h * d->by;
        for (int row = 0; row < d->h; row++)
            for (int col = 0; col < d->w; col++)
                if ((g[row * d->by + (col >> 3)] >> (7 - (col & 7))) & 1)
                    fb_set(x + col, 31 - (y + (d->h - 1 - row)), 1);
        x += d->w + 1;
    }
}

static void fb_digits(int x, int y, const char *t)
{
    const digits_t *d = &DIGITS[clk_set];
    for (; *t && x < 128; t++) {
        int gi = digit_glyph_in(d, &t);
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
    uint32_t now = systick_ms();
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
    /* шрифты 3 и дальше — наборы цифр: те же координаты, только глифы другие */
    if (font_id >= FONT_TEXT_COUNT) {
        fb_digits_set(x, y, t, font_id - FONT_TEXT_COUNT);
        return;
    }
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
    pin_high(BLK_PORT, BLK_PIN_NO);      /* HIGH гасит */
}
static void blk_show(void)
{
    pin_low(BLK_PORT, BLK_PIN_NO);    /* LOW показывает */
}
static void lat_pulse(void)
{
    pin_high(LAT_PORT, LAT_PIN_NO);
    delay_us(1);                                              /* LAT high >= 300 нс */
    pin_low(LAT_PORT, LAT_PIN_NO);
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
        uint32_t t0 = cycles_now();

        if (cfg.mode == 0) {
            blk_blank();                                      /* гасим до сдвига... */
            delay_us(cfg.blk_us);                               /* ...BLK hold >= 10 мкс */
        }

        spi_send(slotbuf[k], cfg.bytes);

        delay_us(1);                                            /* CLK -> LAT >= 250 нс */
        if (cfg.mode == 1) blk_blank();
        delay_us(cfg.blk_us);
        lat_pulse();
        blk_show();
        if (cfg.on_us) delay_us(cfg.on_us);
        wait_cycles_from(t0, cfg.slotus * (SystemCoreClock / 1000000U));                                        /* период слота */
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

/* ---------- состояние, которым управляют и консоль, и шина ---------- */
static uint8_t  fb_inverted = 0;        /* держим отдельно: по шине это флаг, а не действие */
static uint8_t  boot_request = 0;       /* просьба уйти в загрузчик, исполняется в цикле */
static uint16_t mb_unix_hi = 0;         /* старшее слово времени, ждёт младшего */

static void mb_text_redraw(void);       /* определения ниже, у буфера строк */
static void clk_tick_reset(void);
static void mb_clear_all(void);

static void fb_invert_now(void)
{
    for (int y = 0; y < 32; y++)
        for (int i = 0; i < 16; i++) fb[y][i] = (uint8_t)~fb[y][i];
    dirty = 1;
}

static void fb_invert_to(int on)
{
    if (!!on == !!fb_inverted) return;
    fb_inverted = on ? 1 : 0;
    fb_invert_now();
}

/* Режим из регкарты <-> внутреннее состояние анимации */
static uint16_t anim_to_mode(void)
{
    if (demo_on) return VFD_MODE_DEMO;
    switch (anim) {
    case 1:  return VFD_MODE_CUBE;
    case 2:  return VFD_MODE_LOGO;
    case 3:  return VFD_MODE_CLOCK;
    default: return VFD_MODE_TEXT;
    }
}

static void mode_apply(uint16_t m)
{
    demo_on = 0;
    switch (m) {
    /* Режим 0 — это и есть «очистить»: гасим поле и стираем всё содержимое,
       иначе при возврате в режим текста снова вылезли бы прежние строки. */
    case VFD_MODE_BLANK: anim = 0; mb_clear_all(); fb_clear(); dirty = 1; break;
    case VFD_MODE_TEXT:  anim = 0; mb_text_redraw(); break;
    case VFD_MODE_CLOCK: anim = 3; clk_tick_reset(); break;
    case VFD_MODE_LOGO:  anim = 2; break;
    case VFD_MODE_CUBE:  anim = 1; break;
    case VFD_MODE_DEMO:  demo_on = 1; anim = 1; break;
    case VFD_MODE_IMAGE:                        /* окно картинки ещё не написано */
        up("режим картинки пока не реализован\r\n");
        break;
    default: break;
    }
}

/* ---------- Modbus RTU: модуль как устройство Wiren Board ----------
   Живёт на том же UART, что консоль: по USB у стенда только он. Разводятся по
   первому байту — кадр Modbus начинается с адреса 0x01, а в текстовых командах
   такого байта не бывает, поэтому одно другому не мешает.

   Регистры и их адреса берутся из общей регкарты fw-vfd/include/regmap-structs.h,
   стандартные адреса ВБ (129, 131, 250, 290, 330) — из wbfw.h. */
#define MB_ADDR          1
#define MB_GAP_MS        4          /* 3.5 символа; на 115200 с запасом */
#define MB_BUF           300

static uint8_t  mb_buf[MB_BUF];
static uint16_t mb_len = 0;
static uint16_t mb_errors = 0;      /* битые кадры, отдаём в STATUS */

/* Конец кадра НЕ по паузе на линии: главный цикл занят выводом кадра дисплея
   ~10 мс, и байты одного запроса попадают в разные проходы — пауза 3.5 символа
   «истекала» посреди кадра, остаток уходил в консоль. В Modbus RTU длина кадра
   однозначно задана функцией, поэтому ждём ровно столько байт, сколько нужно. */
static uint16_t mb_expected(const uint8_t *d, uint16_t have)
{
    if (have < 2) return 8;                     /* пока не знаем функцию */
    switch (d[1]) {
    case 0x03: case 0x04: case 0x06: return 8;  /* адрес, функция, регистр, значение, CRC */
    case 0x10:                                  /* + байт длины и сами данные */
        if (have < 7) return 9;
        return (uint16_t)(9 + d[6]);
    default: return 8;
    }
}

static uint16_t mb_crc(const uint8_t *d, int n)
{
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < n; i++) {
        crc ^= d[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 1) ? (uint16_t)((crc >> 1) ^ 0xA001) : (uint16_t)(crc >> 1);
    }
    return crc;
}

static void mb_reply(uint8_t *d, int n)
{
    uint16_t crc = mb_crc(d, n);
    d[n] = (uint8_t)(crc & 0xFF);
    d[n + 1] = (uint8_t)(crc >> 8);
    uart_push((const char *)d, n + 2);
}

static void mb_exception(uint8_t fn, uint8_t code)
{
    uint8_t r[5];
    r[0] = MB_ADDR; r[1] = (uint8_t)(fn | 0x80); r[2] = code;
    mb_reply(r, 3);
}

/* Строки текста живут в своём буфере: контроллер пишет их регистрами, а рисуем
   мы по своему такту, чтобы не рвать кадр посреди приёма. */
static char mb_text[TEXT_LINES][TEXT_LEN + 1];
static char mb_params[TEXT_LINES][TEXT_PARAMS_LEN + 1];
static uint8_t mb_text_dirty = 0;

/* Разбор строки параметров: до трёх чисел, разделитель любой — пробел, запятая,
   точка с запятой, что угодно кроме цифр и минуса. Пропущенные числа остаются
   как были, поэтому «10 20» задаёт только координаты, а «,,2» — только шрифт. */
/* Разбор до max чисел из строки; разделитель любой, минус учитывается */
static int nums_parse(const char *s, int *out, int max)
{
    int got = 0;
    while (*s && got < max) {
        while (*s && *s != '-' && (*s < '0' || *s > '9')) s++;
        if (!*s) break;
        int sign = 1;
        if (*s == '-') { sign = -1; s++; }
        if (*s < '0' || *s > '9') continue;
        int v = 0;
        while (*s >= '0' && *s <= '9') v = v * 10 + (*s++ - '0');
        out[got++] = sign * v;
    }
    return got;
}

/* Окружность по середине точки: рисуем контур, заливка тут не нужна */
static void fb_circle(int cx, int cy, int d)
{
    int r = d / 2;
    if (r < 1) { fb_set(cx, 31 - cy, 1); return; }
    int x = r, y = 0, err = 1 - r;
    while (x >= y) {
        const int pts[8][2] = { { cx + x, cy + y }, { cx + y, cy + x },
                                { cx - y, cy + x }, { cx - x, cy + y },
                                { cx - x, cy - y }, { cx - y, cy - x },
                                { cx + y, cy - x }, { cx + x, cy - y } };
        for (int i = 0; i < 8; i++)
            if (pts[i][0] >= 0 && pts[i][0] < 128 && pts[i][1] >= 0 && pts[i][1] < 32)
                fb_set(pts[i][0], 31 - pts[i][1], 1);
        y++;
        if (err < 0) err += 2 * y + 1;
        else { x--; err += 2 * (y - x) + 1; }
    }
}

static int params_parse(const char *s, int *x, int *y, int *font)
{
    int *out[3] = { x, y, font };
    int got = 0;
    while (*s && got < 3) {
        while (*s && *s != '-' && (*s < '0' || *s > '9')) s++;
        if (!*s) break;
        int sign = 1;
        if (*s == '-') { sign = -1; s++; }
        if (*s < '0' || *s > '9') continue;
        int v = 0;
        while (*s >= '0' && *s <= '9') v = v * 10 + (*s++ - '0');
        *out[got] = sign * v;
        got++;
    }
    return got;                             /* сколько чисел реально задано */
}

static void clk_tick_reset(void) { clk_tick = systick_ms(); }

/* Строки рисуем сверху вниз выбранным шрифтом. Ноль по Y у поля внизу, поэтому
   первая строка садится на 32 минус высота шрифта. */
/* Поле из одних пробелов считаем пустым. Иначе очистить строку из интерфейса
   нельзя: пустую посылку драйвер в устройство не пишет, и на экране остаётся
   прежний текст. Плюс сам драйвер записывает обратно тот пробел, которым мы
   отдаём пустое поле, чтобы контрол не исчезал из списка. */
static int blank_only(const char *s)
{
    for (; *s; s++)
        if (*s != ' ') return 0;
    return 1;
}

/* Примитивы: линия, окружность, рамка. Числа берём из тех же текстовых полей,
   что и размещение строк, поэтому вводятся они так же свободно. */
static char mb_graph[GRAPH_LINES][GRAPH_LEN + 1];

/* Прямоугольник последнего нарисованного текста каждой строки: по нему
   стирается прежний текст при построчной перерисовке. */
static struct { int16_t x, y, w, h; } ln_box[TEXT_LINES];

/* Стереть всё содержимое: строки, их параметры и примитивы. Дёргается записью
   1 в регистр очистки — в интерфейсе это кнопка. */
static void mb_clear_all(void)
{
    memset(mb_text, 0, sizeof mb_text);
    memset(mb_params, 0, sizeof mb_params);
    memset(mb_graph, 0, sizeof mb_graph);
    memset(ln_box, 0, sizeof ln_box);
    mb_text_dirty = 1;
}

/* Примитивы копятся: каждая новая запись дорисовывается ПОВЕРХ текущего кадра, а
   не заменяет прежнюю. Поэтому рисуем их не в общей перерисовке, а по факту
   записи — тогда несколько линий подряд складываются в картинку. Стереть всё —
   режим 0. */
static uint8_t mb_graph_new = 0;        /* биты полей, записанных и не нарисованных */

static void mb_graph_draw_one(int g)
{
    int v[4];
    mb_graph[g][GRAPH_LEN] = 0;

    if (g == 0) {
        if (nums_parse(mb_graph[0], v, 4) == 4)
            fb_line(v[0], 31 - v[1], v[2], 31 - v[3]);
    } else if (g == 1) {
        if (nums_parse(mb_graph[1], v, 3) == 3)
            fb_circle(v[0], v[1], v[2]);
    } else {
        if (nums_parse(mb_graph[2], v, 4) == 4) {   /* рамка по двум углам */
            int x0 = v[0] < v[2] ? v[0] : v[2], x1 = v[0] < v[2] ? v[2] : v[0];
            int y0 = v[1] < v[3] ? v[1] : v[3], y1 = v[1] < v[3] ? v[3] : v[1];
            fb_line(x0, 31 - y0, x1, 31 - y0);
            fb_line(x0, 31 - y1, x1, 31 - y1);
            fb_line(x0, 31 - y0, x0, 31 - y1);
            fb_line(x1, 31 - y0, x1, 31 - y1);
        }
    }
}

static void mb_graph_pending(void)
{
    for (int g = 0; g < GRAPH_LINES; g++)
        if (mb_graph_new & (1u << g)) mb_graph_draw_one(g);
    if (mb_graph_new) { mb_graph_new = 0; dirty = 1; }
}

/* ---------- построчная перерисовка ----------
   Порядок работы такой: сперва регистрами выкладывается графика, потом задаются
   координаты и шрифты строк, а дальше меняются только тексты. Поэтому кадр целиком
   НЕ перерисовывается: гасится только прямоугольник прежнего текста этой строки, и
   на его месте рисуется новый. Всё остальное — графика и соседние строки —
   остаётся нетронутым. */
static uint8_t mb_text_new = 0;         /* биты строк, которые надо перерисовать */

/* Ширина строки в точках для выбранного шрифта */
static int text_width(int fnt, const char *s)
{
    if (fnt >= FONT_TEXT_COUNT) {
        const digits_t *d = &DIGITS[fnt - FONT_TEXT_COUNT];
        int n = 0;
        for (const char *q = s; *q; q++)
            if (((uint8_t)*q & 0xC0) != 0x80) n++;      /* двухбайтовые считаем раз */
        return n * (d->w + 1);
    }
    const font_t *f = &FONTS[fnt];
    int w = 0;
    for (const char *q = s; *q; q++)
        if (((uint8_t)*q & 0xC0) != 0x80) w += f->w + 1;
    return w;
}

static void fb_clear_rect(int x, int y, int w, int h)
{
    for (int dy = 0; dy < h; dy++)
        for (int dx = 0; dx < w; dx++) {
            int px = x + dx, py = y + dy;
            if (px >= 0 && px < 128 && py >= 0 && py < 32) fb_set(px, 31 - py, 0);
        }
}

/* Геометрия строки: координаты из её поля параметров либо автоукладка сверху вниз.
   Автоукладка считается по строкам выше, поэтому не зависит от порядка обновления. */
static void line_geom(int ln, int *x, int *y, int *fnt)
{
    *fnt = font_id;
    *x = 0;
    *y = -1;
    int got = params_parse(mb_params[ln], x, y, fnt);
    if (*fnt < 0 || *fnt >= FONT_COUNT) *fnt = font_id;
    if (got >= 2 && *y >= 0) return;                    /* координаты заданы */

    int auto_y = 32;
    for (int i = 0; i <= ln; i++) {
        if (i != ln && blank_only(mb_text[i])) continue;
        int fx = 0, fy = -1, ff = font_id;
        int g = params_parse(mb_params[i], &fx, &fy, &ff);
        if (ff < 0 || ff >= FONT_COUNT) ff = font_id;
        if (g >= 2 && fy >= 0) continue;                /* строка стоит на своих координатах */
        auto_y -= font_height(ff) + 1;
    }
    *y = auto_y;
    if (got < 1) *x = 0;
}

static void mb_line_update(int ln)
{
    uint8_t save = font_id;

    mb_text[ln][TEXT_LEN] = 0;
    mb_params[ln][TEXT_PARAMS_LEN] = 0;

    if (ln_box[ln].w > 0)              /* стереть прежний текст этой строки */
        fb_clear_rect(ln_box[ln].x, ln_box[ln].y, ln_box[ln].w, ln_box[ln].h);
    ln_box[ln].w = 0;

    if (!blank_only(mb_text[ln])) {
        int x, y, fnt;
        line_geom(ln, &x, &y, &fnt);
        if (y >= 0) {
            font_id = (uint8_t)fnt;
            fb_text(x, y, mb_text[ln]);
            font_id = save;
            int w = text_width(fnt, mb_text[ln]);
            if (x + w > 128) w = 128 - x;
            ln_box[ln].x = (int16_t)x;
            ln_box[ln].y = (int16_t)y;
            ln_box[ln].w = (int16_t)(w > 0 ? w : 0);
            ln_box[ln].h = (int16_t)font_height(fnt);
        }
    }
    dirty = 1;
}

/* Полная выкладка всех строк — при входе в режим текста. Кадр не очищаем: графика,
   выложенная до этого, должна остаться. */
static void mb_text_redraw(void)
{
    for (int ln = 0; ln < TEXT_LINES; ln++) {
        ln_box[ln].w = 0;
        mb_line_update(ln);
    }
    if (fb_inverted) fb_invert_now();
    mb_text_new = 0;
    mb_text_dirty = 0;
}

/* Время приходит секундами эпохи, разворачиваем в часы и календарь сами:
   от контроллера идёт UTC, поэтому смещение зоны храним отдельно. */
static int16_t mb_tz_min = 0;

static void mb_set_unixtime(uint32_t t)
{
    int32_t local = (int32_t)t + (int32_t)mb_tz_min * 60;
    uint32_t days = (uint32_t)(local / 86400);
    uint32_t rem = (uint32_t)(local % 86400);
    clk_h = (uint8_t)(rem / 3600);
    clk_m = (uint8_t)((rem % 3600) / 60);
    clk_s = (uint8_t)(rem % 60);
    clk_tick = systick_ms();

    /* от 1970-01-01 идём годами: диапазон нам нужен бытовой, не астрономический */
    uint16_t y = 1970;
    for (;;) {
        uint16_t len = (uint16_t)(((y % 4 == 0 && y % 100 != 0) || y % 400 == 0) ? 366 : 365);
        if (days < len) break;
        days -= len; y++;
    }
    uint8_t mo = 1;
    for (;;) {
        uint8_t len = MDAYS[mo - 1];
        if (mo == 2 && ((y % 4 == 0 && y % 100 != 0) || y % 400 == 0)) len = 29;
        if (days < len) break;
        days -= len; mo++;
    }
    dt_y = y; dt_mo = mo; dt_d = (uint8_t)(days + 1);
}

/* Обратное преобразование: свои часы и календарь -> unix-секунды UTC. Нужно,
   чтобы контроллер видел, какое время у модуля, и замечал расхождение. */
static uint32_t mb_get_unixtime(void)
{
    uint32_t days = 0;
    for (uint16_t y = 1970; y < dt_y; y++)
        days += (((y % 4 == 0 && y % 100 != 0) || y % 400 == 0) ? 366U : 365U);
    for (uint8_t mo = 1; mo < dt_mo; mo++) {
        days += MDAYS[mo - 1];
        if (mo == 2 && ((dt_y % 4 == 0 && dt_y % 100 != 0) || dt_y % 400 == 0)) days++;
    }
    days += (uint32_t)(dt_d - 1);
    int32_t local = (int32_t)(days * 86400U + clk_h * 3600U + clk_m * 60U + clk_s);
    return (uint32_t)(local - (int32_t)mb_tz_min * 60);
}

static uint16_t mb_read_reg(uint16_t reg)
{
    /* стандартные регистры ВБ: по ним инструменты узнают устройство */
    if (reg >= WBFW_REG_FW_VERSION && reg < WBFW_REG_FW_VERSION + 8) {
        static const char v[] = "1.0.0";
        uint16_t i = reg - WBFW_REG_FW_VERSION;
        return (i < sizeof v - 1) ? (uint8_t)v[i] : 0;
    }
    if (reg >= WBFW_REG_SIGNATURE && reg < WBFW_REG_SIGNATURE + WBFW_SIGNATURE_LEN) {
        static const char s[] = WBFW_SIGNATURE;
        uint16_t i = reg - WBFW_REG_SIGNATURE;
        return (i < sizeof s - 1) ? (uint8_t)s[i] : 0;
    }
    if (reg >= WBFW_REG_BOOTLOADER_VERSION && reg < WBFW_REG_BOOTLOADER_VERSION + 8) {
        static const char b[] = "1.0.0";
        uint16_t i = reg - WBFW_REG_BOOTLOADER_VERSION;
        return (i < sizeof b - 1) ? (uint8_t)b[i] : 0;
    }

    switch (reg) {
    case REGMAP_ADDR_HW_INFO + 0: return 0x1283;              /* модель: 128x32 */
    case REGMAP_ADDR_HW_INFO + 2: return 1;                   /* версия прошивки */
    case REGMAP_ADDR_HW_INFO + 3: return 0;
    case REGMAP_ADDR_HW_INFO + 4: return 0;

    case REGMAP_ADDR_CONTROL + 0: return anim_to_mode();
    case REGMAP_ADDR_CONTROL + 1: return font_id;
    case REGMAP_ADDR_CONTROL + 2: return clk_set;
    case REGMAP_ADDR_CONTROL + 3: return 0;
    case REGMAP_ADDR_CONTROL + 4: return cfg.on_us;
    case REGMAP_ADDR_CONTROL + 5: return fb_inverted;

    /* время отдаём тем же порядком, каким принимаем: старшее слово первым */
    case REGMAP_ADDR_TIME + 0:    return (uint16_t)(mb_get_unixtime() >> 16);
    case REGMAP_ADDR_TIME + 1:    return (uint16_t)(mb_get_unixtime() & 0xFFFF);
    case REGMAP_ADDR_TIME + 2:    return (uint16_t)mb_tz_min;

    case REGMAP_ADDR_STATUS + 0:  return (uint16_t)(cfg.slotus * cfg.slots);
    case REGMAP_ADDR_STATUS + 1:  return cfg.slotus;
    /* наработка тоже старшим словом вперёд, иначе u32 у драйвера едет на 65536 */
    case REGMAP_ADDR_STATUS + 2:  return (uint16_t)((systick_ms() / 1000U) >> 16);
    case REGMAP_ADDR_STATUS + 3:  return (uint16_t)((systick_ms() / 1000U) & 0xFFFF);
    case REGMAP_ADDR_STATUS + 4:  return mb_errors;
    default: break;
    }

    /* строки текста: по два символа в регистре, за текстом — параметры строки */
    if (reg >= REGMAP_ADDR_TEXT1 && reg < REGMAP_ADDR_TEXT1 + TEXT_LINES * TEXT_STRIDE) {
        uint16_t off = reg - REGMAP_ADDR_TEXT1;
        uint16_t ln = off / TEXT_STRIDE, r = off % TEXT_STRIDE;
        if (ln >= TEXT_LINES) return 0;
        /* Пустое поле отдаём одним пробелом. Причина не в железе: пустая
           удержанная посылка удаляет топик MQTT, а интерфейс не рисует контрол
           без значения — поля просто исчезали бы из списка. Пробел даёт топику
           существовать, а на экране он ничего не рисует и в разборе параметров
           чисел не даёт, поэтому строка укладывается сама. */
        if (r < TEXT_PARAMS_OFF) {
            uint16_t i = r * 2;
            if (i + 1 < TEXT_LEN) {
                if (i == 0 && !mb_text[ln][0]) return (uint16_t)(' ' << 8);
                return (uint16_t)(((uint8_t)mb_text[ln][i] << 8) | (uint8_t)mb_text[ln][i + 1]);
            }
        } else {
            uint16_t i = (uint16_t)((r - TEXT_PARAMS_OFF) * 2);
            if (i + 1 < TEXT_PARAMS_LEN) {
                if (i == 0 && !mb_params[ln][0]) return (uint16_t)(' ' << 8);
                return (uint16_t)(((uint8_t)mb_params[ln][i] << 8) | (uint8_t)mb_params[ln][i + 1]);
            }
        }
    }

    /* примитивы: три поля по 16 байт, пустое отдаём пробелом, как и строки */
    if (reg >= REGMAP_ADDR_LINE && reg < REGMAP_ADDR_LINE + GRAPH_LINES * 0x10) {
        uint16_t g = (uint16_t)((reg - REGMAP_ADDR_LINE) / 0x10);
        uint16_t i = (uint16_t)(((reg - REGMAP_ADDR_LINE) % 0x10) * 2);
        if (g < GRAPH_LINES && i + 1 < GRAPH_LEN) {
            if (i == 0 && !mb_graph[g][0]) return (uint16_t)(' ' << 8);
            return (uint16_t)(((uint8_t)mb_graph[g][i] << 8) | (uint8_t)mb_graph[g][i + 1]);
        }
    }
    return 0;
}

static int mb_write_reg(uint16_t reg, uint16_t v)
{
    /* уход в загрузчик по шине — то же, что делает ключ -j у флешера ВБ */
    if ((reg == WBFW_REG_JUMP_TO_BOOT_STANDARD_BAUD ||
         reg == WBFW_REG_JUMP_TO_BOOT_CURRENT_BAUD) && v == 1) {
        boot_request = 1;                 /* ответим и уйдём: у нас ~5 мс по протоколу */
        return 1;
    }

    switch (reg) {
    case REGMAP_ADDR_CONTROL + 0:
        if (v > VFD_MODE_MAX_WORKING) return 0;   /* картинка ещё не реализована */
        mode_apply(v); return 1;
    case REGMAP_ADDR_CONTROL + 1: if (v < FONT_COUNT) font_id = (uint8_t)v; return 1;
    case REGMAP_ADDR_CONTROL + 2: if (v < DIGITS_COUNT) clk_set = (uint8_t)v; return 1;
    case REGMAP_ADDR_CONTROL + 3: return 1;                     /* раскладка часов сама */
    case REGMAP_ADDR_CONTROL + 4: cfg.on_us = v; dirty = 1; return 1;   /* яркость */
    case REGMAP_ADDR_CONTROL + 5: fb_invert_to(v & VFD_FLAG_INVERT); return 1;
    case REGMAP_ADDR_CONTROL + 6: if (v) mb_clear_all(); return 1;   /* кнопка «очистить» */

    /* старшее слово приходит первым, применяем по приходу младшего */
    case REGMAP_ADDR_TIME + 0: mb_unix_hi = v; return 1;
    case REGMAP_ADDR_TIME + 1: mb_set_unixtime(((uint32_t)mb_unix_hi << 16) | v); return 1;
    case REGMAP_ADDR_TIME + 2: mb_tz_min = (int16_t)v; return 1;
    default: break;
    }

    if (reg >= REGMAP_ADDR_TEXT1 && reg < REGMAP_ADDR_TEXT1 + TEXT_LINES * TEXT_STRIDE) {
        uint16_t off = reg - REGMAP_ADDR_TEXT1;
        uint16_t ln = off / TEXT_STRIDE, r = off % TEXT_STRIDE;
        if (ln >= TEXT_LINES) return 0;
        /* Запись в первый регистр = новое значение целиком: драйвер передаёт
           только нужные ему регистры, и без очистки остаётся хвост от прошлой
           строки, а на кириллице ещё и рвётся UTF-8. */
        if (r < TEXT_PARAMS_OFF) {
            uint16_t i = r * 2;
            if (i + 1 >= TEXT_LEN) return 0;
            if (i == 0) memset(mb_text[ln], 0, TEXT_LEN + 1);
            mb_text[ln][i] = (char)(v >> 8);
            mb_text[ln][i + 1] = (char)(v & 0xFF);
            mb_text_new |= (uint8_t)(1u << ln);      /* перерисуем только эту строку */
        } else {
            uint16_t i = (uint16_t)((r - TEXT_PARAMS_OFF) * 2);
            if (i + 1 >= TEXT_PARAMS_LEN) return 0;
            if (i == 0) memset(mb_params[ln], 0, TEXT_PARAMS_LEN + 1);
            mb_params[ln][i] = (char)(v >> 8);
            mb_params[ln][i + 1] = (char)(v & 0xFF);
            mb_text_new |= (uint8_t)(1u << ln);      /* сменилось размещение строки */
        }
        return 1;
    }

    if (reg >= REGMAP_ADDR_LINE && reg < REGMAP_ADDR_LINE + GRAPH_LINES * 0x10) {
        uint16_t g = (uint16_t)((reg - REGMAP_ADDR_LINE) / 0x10);
        uint16_t i = (uint16_t)(((reg - REGMAP_ADDR_LINE) % 0x10) * 2);
        if (g >= GRAPH_LINES || i + 1 >= GRAPH_LEN) return 0;
        if (i == 0) memset(mb_graph[g], 0, GRAPH_LEN + 1);   /* новое значение целиком */
        mb_graph[g][i] = (char)(v >> 8);
        mb_graph[g][i + 1] = (char)(v & 0xFF);
        mb_graph_new |= (uint8_t)(1u << g);    /* дорисуем поверх, не стирая прежнее */
        return 1;
    }
    return 0;                                   /* адрес не наш */
}

static void mb_handle(const uint8_t *req, int len)
{
    if (len < 4 || req[0] != MB_ADDR) return;               /* не нам */
    if (mb_crc(req, len - 2) != (uint16_t)(req[len - 2] | (req[len - 1] << 8))) {
        mb_errors++;
        return;                                            /* битый кадр — молчим */
    }

    uint8_t fn = req[1];
    uint16_t reg = (uint16_t)((req[2] << 8) | req[3]);
    uint16_t cnt = (uint16_t)((req[4] << 8) | req[5]);

    if (fn == 0x03 || fn == 0x04) {                         /* чтение регистров */
        if (cnt == 0 || cnt > 125) { mb_exception(fn, 0x03); return; }
        uint8_t r[256];
        r[0] = MB_ADDR; r[1] = fn; r[2] = (uint8_t)(cnt * 2);
        for (uint16_t i = 0; i < cnt; i++) {
            uint16_t v = mb_read_reg((uint16_t)(reg + i));
            r[3 + i * 2] = (uint8_t)(v >> 8);
            r[4 + i * 2] = (uint8_t)(v & 0xFF);
        }
        mb_reply(r, 3 + cnt * 2);
        return;
    }

    if (fn == 0x06) {                                       /* запись одного */
        if (!mb_write_reg(reg, cnt)) { mb_exception(fn, 0x02); return; }
        uint8_t r[6] = { req[0], req[1], req[2], req[3], req[4], req[5] };
        mb_reply(r, 6);
        return;
    }

    if (fn == 0x10) {                                       /* запись нескольких */
        uint8_t nb = req[6];
        if (len < 9 + nb) { mb_errors++; return; }
        for (uint16_t i = 0; i < cnt; i++) {
            uint16_t v = (uint16_t)((req[7 + i * 2] << 8) | req[8 + i * 2]);
            if (!mb_write_reg((uint16_t)(reg + i), v)) { mb_exception(fn, 0x02); return; }
        }
        uint8_t r[6] = { req[0], req[1], req[2], req[3], req[4], req[5] };
        mb_reply(r, 6);
        return;
    }

    mb_exception(fn, 0x01);
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
                clk_tick = systick_ms();
            }
        }
        upf("время %02u:%02u:%02u\r\n", clk_h, clk_m, clk_s);
    }
    else if (!strcmp(c, "clock"))  {   /* clock — часы с датой, clock - выключить */
        if (a && a[0] == '-') { anim = 0; up("часы выключены\r\n"); }
        else {
            if (a && a[0] >= '0' && a[0] <= '9') clk_set = (uint8_t)(atoi(a) % DIGITS_COUNT);
            const digits_t *d = &DIGITS[clk_set];
            demo_on = 0; anim = 3; clk_tick = systick_ms();
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
    else if (!strcmp(c, "font"))   {   /* font 0..6: три текстовых и наборы цифр */
        if (a) font_id = (uint8_t)(atoi(a) % FONT_COUNT);
        int h = font_height(font_id);
        upf("шрифт %d = %s, высота %d%s\r\n", font_id, font_name(font_id), h,
            font_id >= FONT_TEXT_COUNT ? " (цифры и знаки :.,-+/ и градус)" : "");
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
    else if (!strcmp(c, "iv"))     {   /* инверсия кадра, тот же флаг видит шина */
        fb_invert_to(!fb_inverted);
        upf("инверсия %s\r\n", fb_inverted ? "вкл" : "выкл");
    }
    else if (!strcmp(c, "px"))     {   /* px X Y [0|1] — точка в видимых координатах */
        if (a && b) { anim = 0; demo_on = 0;
                      fb_set(atoi(a), 31 - atoi(b), d ? atoi(d) : 1); up("ok\r\n"); }
    }
    else if (!strcmp(c, "str"))    { if (a) { fb_clear(); fb_text(b ? atoi(a) : 2, 12, b ? b : a); up("ok\r\n"); } }
    else if (!strcmp(c, "cfg"))    { if (a && b) cfg_set(a, atoi(b)); else print_st(); }
    else if (!strcmp(c, "ef"))     { filament_enable(!(a && a[0] == '0')); up("ok\r\n"); }
    else if (!strcmp(c, "spd"))    {   /* spd N — предделитель SPI от 100 МГц */
        if (a) { spi_div = (uint32_t)atoi(a); spi_init(spi_div); }
        upf("SPI /%lu = %lu Гц\r\n", (unsigned long)spi_div, (unsigned long)spi_clock_hz());
    }
    else if (!strcmp(c, "hv"))     {   /* HV — вход, плата поднимает его сама */
        up("HV не драйвим: на этой плате это вход\r\n");
    }
    else if (!strcmp(c, "boot"))   {   /* уйти в загрузчик для обновления по UART */
        /* Магия в верхних байтах ОЗУ переживает сброс: загрузчик увидит её и
           останется ждать прошивку вместо прыжка сюда. Тот же путь, каким по
           Modbus работает запись 1 в регистр 129. */
        up("уходим в загрузчик, 9600 8N2\r\n");
        while (uart_tx_busy()) { }             /* дождаться, пока ответ уйдёт */
        *(volatile uint32_t *)0x2001FFF0U = 0xB007574FU;
        NVIC_SystemReset();
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

/* ---------- кооперативный цикл ----------
   Кадр стекла выводится непрерывно, между кадрами обслуживаются анимация,
   разбор шины и консоль. Ни ОСРВ, ни прерываний с логикой — как у ВБ. */
static void app_loop(void)
{
    char line[80]; int li = 0; uint8_t ch;
    uint32_t t_anim = 0;
    while (1) {
        if (demo_on) {                                /* демо: 15 с куб, 15 с лого */
            static uint32_t t_scene = 0;
            if (t_scene == 0) { t_scene = systick_ms(); anim = 1; }
            if (systick_ms() - t_scene >= DEMO_MS) {
                t_scene = systick_ms();
                anim = (anim == 1) ? 2 : 1;
            }
        }
        if (anim && systick_ms() - t_anim >= 40) {   /* 25 кадров в секунду */
            t_anim = systick_ms();
            if (anim == 3) { clock_tick(); clock_draw(); }
            else if (anim == 2) { logo_move(); fb_logo_at((int)logo_x, (int)logo_y); }
            else { cube_ang += 0.10f; cube_move(); fb_cube(cube_ang); }
        }
        if (dirty) rebuild();
        scan_frame();
        if (mb_text_dirty && anim == 0) mb_text_redraw();
        if (mb_text_new && anim == 0) {              /* перерисовать только изменённые строки */
            for (int ln = 0; ln < TEXT_LINES; ln++)
                if (mb_text_new & (1u << ln)) mb_line_update(ln);
            mb_text_new = 0;
        }
        if (mb_graph_new && anim == 0) mb_graph_pending();   /* примитивы копятся */
        if (boot_request) {                       /* ответ уже в очереди — дождёмся и уйдём */
            while (uart_tx_busy()) { }
            delay_ms(2);
            *(volatile uint32_t *)BOOT_MAGIC_ADDR = BOOT_MAGIC_STAY;
            NVIC_SystemReset();
        }
        while (uart_get(&ch)) {
            /* Modbus или консоль? Кадр Modbus начинается с адреса 0x01, такого
               байта в текстовых командах не бывает. Пока кадр собирается, байты
               в консоль не попадают. */
            if (mb_len || ch == MB_ADDR) {
                /* На пустоту буфера консоли НЕ смотрим: один случайный байт в
                   нём (хост дёргает DTR/RTS при открытии порта) навсегда лишал
                   бы модуль Modbus, и запись регистра 129 от флешера уходила бы
                   в консоль эхом вместо прыжка в загрузчик. */
                if (mb_len < MB_BUF) mb_buf[mb_len++] = ch;
                if (mb_len >= mb_expected(mb_buf, mb_len)) {   /* кадр целиком */
                    mb_handle(mb_buf, mb_len);
                    mb_len = 0;
                    li = 0;                     /* мусор из консоли заодно выкидываем */
                }
                continue;
            }
            /* двоичный мусор в консоль не пускаем: ни в строку, ни в эхо */
            if (ch < 0x20 && ch != '\r' && ch != '\n' && ch != 8 && ch != 127 && ch != 27)
                continue;
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
            else if (li < 79) { line[li++] = (char)ch; uart_push((char *)&ch, 1); }
        }
    }
}

/* ---------- вход ----------
   Порядок как в wb-embedded-controller: сначала тактирование и время, потом
   драйверы, потом кооперативный цикл, который дёргает периодические работы. */
int main(void)
{
    SCB->VTOR = BOOT_APP_BASE;      /* приложение линкуется после загрузчика */

    rcc_init();
    dwt_init();
    systick_init();

    pins_init();
    spi_init(spi_div);
    uart_init(115200);

    /* стартовая рабочая точка стенда: кадр 10 мс = 100 Гц */
    cfg.mode = 0; cfg.mir = 0; cfg.miry = 0;
    cfg.on_us = 180; cfg.slotus = 227; font_id = 1;
    fb_logo();

    up("\r\nMN12832L: 44 слота, перекрытие парами, abc/def\r\n");
    print_st();
    up("> ");

    app_loop();
    return 0;
}
