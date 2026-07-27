// Автояркость VFD-модуля по датчику освещённости WB-MSW.
//
// Кладётся в /etc/wb-rules/. Логика на контроллере, модулю ничего менять не надо:
// правило читает люксы и пишет их в контрол «Яркость», то есть в регистр окна
// показа 0x14.
//
// ПЕРЕД ЗАПУСКОМ: вписать имя своего датчика в SENSOR_DEV. Посмотреть можно так:
//     mosquitto_sub -h localhost -t "/devices/+/controls/Illuminance" -v
// Пока датчика нет, правило просто ничего не делает.

var SENSOR_DEV  = "wb-msw-v3_21";     // имя устройства MSW на шине
var SENSOR_CTRL = "Illuminance";      // канал освещённости, люксы
var TARGET      = "vfd/Brightness";

var B_MIN = 10;      // ночь: еле светится, но читается в темноте
var B_MAX = 190;     // день: предел окна показа, дальше слот не пускает
var LUX_FULL = 2000; // с какой освещённости даём полную яркость

var STEP = 4;        // порог записи: мелкие колебания не гоняем по шине
var SMOOTH = 0.3;    // сглаживание, доля нового значения

var filtered = null;
var applied = null;

// Глаз воспринимает яркость логарифмически, поэтому и шкалу берём логарифмическую:
// иначе на сумерках всё скачет, а днём разницы не видно.
function luxToBrightness(lux) {
    if (lux < 0) lux = 0;
    var k = Math.log(1 + lux) / Math.log(1 + LUX_FULL);
    if (k > 1) k = 1;
    return Math.round(B_MIN + (B_MAX - B_MIN) * k);
}

defineRule("vfd_autobright", {
    when: cron("@every 5s"),
    then: function () {
        var lux = dev[SENSOR_DEV] && dev[SENSOR_DEV][SENSOR_CTRL];
        if (lux === undefined || lux === null) return;   // датчика ещё нет

        // сглаживаем: датчик шумит, а дёргать яркость на каждый шорох незачем
        filtered = (filtered === null) ? lux : filtered + SMOOTH * (lux - filtered);

        var b = luxToBrightness(filtered);
        if (applied !== null && Math.abs(b - applied) < STEP) return;

        applied = b;
        dev[TARGET] = b;
        log("VFD: {} лк -> яркость {}", Math.round(filtered), b);
    }
});
