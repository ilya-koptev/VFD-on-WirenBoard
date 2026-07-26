// Кнопка «Демо» для дашборда: виртуальное устройство с кнопкой, по нажатию
// запускается сценарий на контроллере.
//
// Класть в /etc/wb-rules/, сам сценарий — /root/demo-graph.sh.
// Запуск через setsid и в фоне: правило не должно ждать окончания демки, она
// идёт около минуты.

defineVirtualDevice("vfd_demo", {
    title: "VFD демо",
    cells: {
        run: {
            title: "Запустить график",
            type: "pushbutton",
            value: false
        },
        stop: {
            title: "Остановить",
            type: "pushbutton",
            value: false
        }
    }
});

defineRule("vfd_demo_run", {
    whenChanged: "vfd_demo/run",
    then: function () {
        // прежний прогон снимет сам сценарий по pid-файлу
        runShellCommand("setsid sh /root/demo-graph.sh 3 >/tmp/vfd-demo.log 2>&1 &");
        log("VFD: демо запущено");
    }
});

defineRule("vfd_demo_stop", {
    whenChanged: "vfd_demo/stop",
    then: function () {
        runShellCommand("pkill -F /run/vfd-demo.pid; mosquitto_pub -h localhost -t /devices/vfd/controls/Mode/on -m 0");
        log("VFD: демо остановлено");
    }
});
