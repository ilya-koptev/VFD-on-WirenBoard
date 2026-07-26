/*
 * Загрузчик VFD-модуля: Modbus RTU по UART, протокол обновления Wiren Board.
 *
 * Живёт в начале флеша, приложение лежит со смещением BOOT_APP_BASE. Работает на
 * HSI 16 МГц без PLL — скорости тут не нужны, зато меньше кода инициализации.
 *
 * Куда попадаем после сброса:
 *   - приложение попросило остаться (магия в ОЗУ) -> обслуживаем обновление;
 *   - приложение битое или его нет -> обслуживаем обновление, ждём вечно;
 *   - иначе -> сразу прыгаем в приложение.
 */
#include "stm32f4xx.h"
#include "wbfw.h"

#define MB_ADDR         1           /* адрес на шине, как у модулей ВБ по умолчанию */
#define MB_BUF_SIZE     256
#define MB_FRAME_GAP_MS 4           /* 3.5 символа на 9600 8N2 ≈ 4 мс */

#define MB_FN_READ_HOLDING      0x03
#define MB_FN_WRITE_SINGLE      0x06
#define MB_FN_WRITE_MULTIPLE    0x10

#define MB_ERR_ILLEGAL_FUNCTION 0x01
#define MB_ERR_ILLEGAL_ADDRESS  0x02    /* EMBXILADD: не в режиме загрузчика */
#define MB_ERR_ILLEGAL_VALUE    0x03    /* EMBXILVAL */
#define MB_ERR_SLAVE_FAILURE    0x04    /* EMBXSFAIL: не та подпись */

/* ---------- часы и микросекундный счётчик ---------- */
static volatile uint32_t ms_ticks;

void SysTick_Handler(void) { ms_ticks++; }

static uint32_t now_ms(void) { return ms_ticks; }

static void clock_init(void)
{
    /* HSI 16 МГц уже работает после сброса, PLL не поднимаем.
       Тик 1 кГц нужен для межкадровой паузы Modbus. */
    SysTick_Config(16000000U / 1000U);
}

/* ---------- UART ---------- */
static void uart_init(uint32_t baud)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    /* PA2 = TX, PA3 = RX, альтернативная функция 7 */
    GPIOA->MODER &= ~(GPIO_MODER_MODER2 | GPIO_MODER_MODER3);
    GPIOA->MODER |= (2U << GPIO_MODER_MODER2_Pos) | (2U << GPIO_MODER_MODER3_Pos);
    GPIOA->AFR[0] &= ~((0xFU << 8) | (0xFU << 12));
    GPIOA->AFR[0] |= (7U << 8) | (7U << 12);

    USART2->CR1 = 0;
    USART2->BRR = (16000000U + baud / 2) / baud;
    USART2->CR2 = (2U << USART_CR2_STOP_Pos);           /* два стоп-бита */
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

static void uart_send(const uint8_t *d, int n)
{
    for (int i = 0; i < n; i++) {
        while (!(USART2->SR & USART_SR_TXE)) { }
        USART2->DR = d[i];
    }
    while (!(USART2->SR & USART_SR_TC)) { }
}

static int uart_get(uint8_t *b)
{
    if (!(USART2->SR & USART_SR_RXNE)) return 0;
    *b = (uint8_t)USART2->DR;
    return 1;
}

/* ---------- CRC ---------- */
static uint16_t crc16_modbus(const uint8_t *d, int n)
{
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < n; i++) {
        crc ^= d[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 1) ? (uint16_t)((crc >> 1) ^ 0xA001) : (uint16_t)(crc >> 1);
    }
    return crc;
}

static uint32_t crc32_of(const uint8_t *d, uint32_t n)
{
    uint32_t crc = 0xFFFFFFFFU;
    for (uint32_t i = 0; i < n; i++) {
        crc ^= d[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 1) ? ((crc >> 1) ^ 0xEDB88320U) : (crc >> 1);
    }
    return ~crc;
}

/* ---------- флеш ----------
   На F411 стирание идёт секторами: 0..3 по 16 КБ, 4 — 64 КБ, 5..7 — по 128 КБ.
   Загрузчик занимает сектор 0, приложению отдаём остальные. */
static void flash_unlock(void)
{
    if (FLASH->CR & FLASH_CR_LOCK) {
        FLASH->KEYR = 0x45670123U;
        FLASH->KEYR = 0xCDEF89ABU;
    }
}

static void flash_wait(void) { while (FLASH->SR & FLASH_SR_BSY) { } }

static uint32_t sector_base(int sector)
{
    if (sector <= 3) return 0x08000000U + (uint32_t)sector * 0x4000U;
    if (sector == 4) return 0x08010000U;
    return 0x08020000U + (uint32_t)(sector - 5) * 0x20000U;
}

static uint32_t sector_size(int sector)
{
    if (sector <= 3) return 0x4000U;
    if (sector == 4) return 0x10000U;
    return 0x20000U;
}

/* Стереть столько секторов приложения, сколько нужно под fw_size */
static void flash_erase_app(uint32_t fw_size)
{
    uint32_t end = BOOT_APP_BASE + fw_size;
    flash_unlock();
    flash_wait();
    for (int s = 1; s <= 7; s++) {
        uint32_t base = sector_base(s);
        if (base >= end) break;
        if (base + sector_size(s) <= BOOT_APP_BASE) continue;
        FLASH->CR &= ~FLASH_CR_SNB;
        FLASH->CR |= FLASH_CR_SER | ((uint32_t)s << FLASH_CR_SNB_Pos);
        FLASH->CR |= FLASH_CR_STRT;
        flash_wait();
        FLASH->CR &= ~FLASH_CR_SER;
    }
}

static void flash_write(uint32_t addr, const uint8_t *d, int n)
{
    flash_unlock();
    flash_wait();
    FLASH->CR &= ~FLASH_CR_PSIZE;
    FLASH->CR |= (2U << FLASH_CR_PSIZE_Pos);        /* пишем словами по 32 бита */
    FLASH->CR |= FLASH_CR_PG;
    for (int i = 0; i < n; i += 4) {
        uint32_t w = 0xFFFFFFFFU;
        for (int k = 0; k < 4 && i + k < n; k++)
            w = (w & ~(0xFFU << (8 * k))) | ((uint32_t)d[i + k] << (8 * k));
        *(volatile uint32_t *)(addr + (uint32_t)i) = w;
        flash_wait();
    }
    FLASH->CR &= ~FLASH_CR_PG;
}

/* ---------- переход в приложение ---------- */
static int app_is_valid(void)
{
    uint32_t sp = *(volatile uint32_t *)BOOT_APP_BASE;
    uint32_t pc = *(volatile uint32_t *)(BOOT_APP_BASE + 4);
    /* вершина стека должна смотреть в ОЗУ, точка входа — во флеш приложения */
    if ((sp & 0xFFF00000U) != 0x20000000U) return 0;
    if (pc < BOOT_APP_BASE || pc >= BOOT_APP_LIMIT) return 0;
    return 1;
}

static void jump_to_app(void)
{
    uint32_t sp = *(volatile uint32_t *)BOOT_APP_BASE;
    uint32_t pc = *(volatile uint32_t *)(BOOT_APP_BASE + 4);

    SysTick->CTRL = 0;
    USART2->CR1 = 0;
    SCB->VTOR = BOOT_APP_BASE;              /* таблица векторов приложения */
    __DSB();
    __set_MSP(sp);
    ((void (*)(void))pc)();
    while (1) { }                           /* не возвращаемся */
}

/* ---------- состояние обновления ---------- */
static struct {
    uint8_t  receiving;         /* инфоблок принят, ждём куски данных */
    uint32_t fw_size;           /* сколько байт должно прийти */
    uint32_t fw_crc32;
    uint32_t written;           /* сколько уже записано */
} up;

static const char signature[WBFW_SIGNATURE_LEN] = WBFW_SIGNATURE;

/* ---------- обработка запросов Modbus ---------- */
static int build_exception(uint8_t *out, uint8_t fn, uint8_t code)
{
    out[0] = MB_ADDR;
    out[1] = (uint8_t)(fn | 0x80);
    out[2] = code;
    uint16_t crc = crc16_modbus(out, 3);
    out[3] = (uint8_t)(crc & 0xFF);
    out[4] = (uint8_t)(crc >> 8);
    return 5;
}

static int build_echo(uint8_t *out, const uint8_t *req)
{
    /* штатный ответ на 0x10: адрес, функция, начальный регистр, количество */
    for (int i = 0; i < 6; i++) out[i] = req[i];
    uint16_t crc = crc16_modbus(out, 6);
    out[6] = (uint8_t)(crc & 0xFF);
    out[7] = (uint8_t)(crc >> 8);
    return 8;
}

static int handle_info_block(const uint8_t *payload, uint8_t *out, const uint8_t *req)
{
    const struct wbfw_info_block *ib = (const struct wbfw_info_block *)payload;

    for (int i = 0; i < WBFW_SIGNATURE_LEN; i++)
        if (ib->signature[i] != signature[i])
            return build_exception(out, MB_FN_WRITE_MULTIPLE, MB_ERR_SLAVE_FAILURE);

    /* поля инфоблока идут старшим байтом вперёд, как их видит Modbus */
    uint32_t size = ((uint32_t)payload[12] << 24) | ((uint32_t)payload[13] << 16) |
                    ((uint32_t)payload[14] << 8)  |  (uint32_t)payload[15];
    uint32_t crc  = ((uint32_t)payload[16] << 24) | ((uint32_t)payload[17] << 16) |
                    ((uint32_t)payload[18] << 8)  |  (uint32_t)payload[19];

    if (size == 0 || BOOT_APP_BASE + size > BOOT_APP_LIMIT)
        return build_exception(out, MB_FN_WRITE_MULTIPLE, MB_ERR_ILLEGAL_VALUE);

    up.fw_size = size;
    up.fw_crc32 = crc;
    up.written = 0;
    up.receiving = 1;

    flash_erase_app(size);
    return build_echo(out, req);
}

static int handle_data_block(const uint8_t *payload, uint8_t *out, const uint8_t *req)
{
    if (!up.receiving)
        return build_exception(out, MB_FN_WRITE_MULTIPLE, MB_ERR_ILLEGAL_ADDRESS);

    flash_write(BOOT_APP_BASE + up.written, payload, WBFW_DATA_BLOCK_SIZE);
    up.written += WBFW_DATA_BLOCK_SIZE;

    if (up.written >= up.fw_size) {
        up.receiving = 0;
        uint32_t crc = crc32_of((const uint8_t *)BOOT_APP_BASE, up.fw_size);
        if (crc != up.fw_crc32)
            return build_exception(out, MB_FN_WRITE_MULTIPLE, MB_ERR_SLAVE_FAILURE);
        /* ответ отдаём до прыжка, иначе флешер не увидит подтверждения */
        int n = build_echo(out, req);
        uart_send(out, n);
        *(volatile uint32_t *)BOOT_MAGIC_ADDR = 0;
        jump_to_app();
    }
    return build_echo(out, req);
}

static int handle_read(const uint8_t *req, uint8_t *out)
{
    uint16_t reg = (uint16_t)((req[2] << 8) | req[3]);
    uint16_t cnt = (uint16_t)((req[4] << 8) | req[5]);
    if (cnt == 0 || cnt > 64)
        return build_exception(out, MB_FN_READ_HOLDING, MB_ERR_ILLEGAL_VALUE);

    out[0] = MB_ADDR;
    out[1] = MB_FN_READ_HOLDING;
    out[2] = (uint8_t)(cnt * 2);
    for (int i = 0; i < cnt * 2; i++) out[3 + i] = 0;

    if (reg == WBFW_REG_SIGNATURE) {
        /* подпись: по одному символу на регистр, как читает флешер */
        for (int i = 0; i < cnt && i < WBFW_SIGNATURE_LEN; i++)
            out[3 + i * 2 + 1] = (uint8_t)signature[i];
    } else if (reg == WBFW_REG_BOOTLOADER_VERSION) {
        const char *v = "1.0.0";
        for (int i = 0; i < cnt && v[i]; i++)
            out[3 + i * 2 + 1] = (uint8_t)v[i];
    } else {
        return build_exception(out, MB_FN_READ_HOLDING, MB_ERR_ILLEGAL_ADDRESS);
    }

    int n = 3 + cnt * 2;
    uint16_t crc = crc16_modbus(out, n);
    out[n] = (uint8_t)(crc & 0xFF);
    out[n + 1] = (uint8_t)(crc >> 8);
    return n + 2;
}

static int handle_frame(const uint8_t *req, int len, uint8_t *out)
{
    if (len < 4) return 0;
    if (req[0] != MB_ADDR) return 0;                    /* не нам */

    uint16_t crc = crc16_modbus(req, len - 2);
    if ((uint8_t)(crc & 0xFF) != req[len - 2] ||
        (uint8_t)(crc >> 8) != req[len - 1]) return 0;   /* битый кадр — молчим */

    uint8_t fn = req[1];
    if (fn == MB_FN_READ_HOLDING) return handle_read(req, out);

    if (fn == MB_FN_WRITE_MULTIPLE) {
        uint16_t reg = (uint16_t)((req[2] << 8) | req[3]);
        uint16_t cnt = (uint16_t)((req[4] << 8) | req[5]);
        uint8_t  nb  = req[6];
        if (len < 9 + nb) return 0;
        const uint8_t *payload = &req[7];

        if (reg == WBFW_REG_INFO_BLOCK && nb == WBFW_INFO_BLOCK_SIZE)
            return handle_info_block(payload, out, req);
        if (reg == WBFW_REG_DATA_BLOCK && nb == WBFW_DATA_BLOCK_SIZE)
            return handle_data_block(payload, out, req);

        (void)cnt;
        return build_exception(out, fn, MB_ERR_ILLEGAL_ADDRESS);
    }

    return build_exception(out, fn, MB_ERR_ILLEGAL_FUNCTION);
}

/* ---------- главный цикл ---------- */
int main(void)
{
    volatile uint32_t *magic = (volatile uint32_t *)BOOT_MAGIC_ADDR;
    uint32_t req_magic = *magic;
    *magic = 0;

    int stay = (req_magic == BOOT_MAGIC_STAY) || (req_magic == BOOT_MAGIC_KEEP_BAUD);
    if (!stay && app_is_valid()) jump_to_app();

    clock_init();
    uart_init(WBFW_BOOT_BAUD);

    static uint8_t rx[MB_BUF_SIZE], tx[MB_BUF_SIZE];
    int rlen = 0;
    uint32_t last_byte = now_ms();

    while (1) {
        uint8_t b;
        if (uart_get(&b)) {
            if (rlen < MB_BUF_SIZE) rx[rlen++] = b;
            last_byte = now_ms();
            continue;
        }
        /* кадр Modbus закончен, когда линия молчит 3.5 символа */
        if (rlen && (now_ms() - last_byte) >= MB_FRAME_GAP_MS) {
            int n = handle_frame(rx, rlen, tx);
            if (n) uart_send(tx, n);
            rlen = 0;
        }
    }
}
