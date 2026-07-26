"""Локальная сборка прошивки теми же флагами, что в Makefile.

    python build.py boot    — только загрузчик
    python build.py app     — только приложение
    python build.py         — оба, плюс .wbfw для обновления по шине

На этом стенде (Windows) make не установлен, а тулчейн лежит в пакетах
PlatformIO; на контроллере и на Linux собирается обычным make.
"""
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.abspath(__file__))
PIO = os.path.join(os.path.expanduser("~"), ".platformio", "packages")
BIN = os.path.join(PIO, "toolchain-gccarmnoneeabi", "bin")
CMSIS = os.path.join(PIO, "framework-stm32cubef4", "Drivers", "CMSIS")

CC = os.path.join(BIN, "arm-none-eabi-gcc.exe")
OBJCOPY = os.path.join(BIN, "arm-none-eabi-objcopy.exe")
SIZE = os.path.join(BIN, "arm-none-eabi-size.exe")

MCU, CPU = "STM32F411xE", "cortex-m4"
BUILD = os.path.join(ROOT, "build")

TARGETS = {
    "boot": {
        "src": ["src/boot/main.c", "system/startup.c"],
        "ld": "ldscripts/stm32f411_boot.ld",
        "extra": [],
    },
    "app": {
        "src": ["src/vfd-app.c", "src/rcc.c", "src/gpio.c", "src/spi.c", "src/uart.c",
                "system/startup.c", "system/syscalls.c"],
        "ld": "ldscripts/stm32f411_app.ld",
        # аппаратный FPU: вращение куба считается во float
        "extra": ["-mfpu=fpv4-sp-d16", "-mfloat-abi=hard"],
    },
}

BASE_CFLAGS = ["-mthumb", "-mcpu=" + CPU, "-Os", "-std=gnu11",
               "-ffunction-sections", "-fdata-sections", "-fno-common",
               "-Wall", "-Wextra", "-D" + MCU,
               "-I" + os.path.join(ROOT, "include"),
               "-I" + os.path.join(CMSIS, "Include"),
               "-I" + os.path.join(CMSIS, "Device", "ST", "STM32F4xx", "Include")]


def run(cmd):
    r = subprocess.run(cmd, capture_output=True, text=True)
    out = (r.stdout + r.stderr).strip()
    if out:
        print(out)
    if r.returncode:
        sys.exit(r.returncode)


def build(name):
    t = TARGETS[name]
    outdir = os.path.join(BUILD, name)
    os.makedirs(outdir, exist_ok=True)
    cflags = BASE_CFLAGS + t["extra"]
    objs = []
    for s in t["src"]:
        o = os.path.join(outdir, os.path.basename(s).replace(".c", ".o"))
        run([CC] + cflags + ["-c", os.path.join(ROOT, s), "-o", o])
        objs.append(o)
    elf = os.path.join(BUILD, name + ".elf")
    ldflags = ["-mthumb", "-mcpu=" + CPU] + t["extra"] + [
        "-T" + os.path.join(ROOT, t["ld"]), "-nostartfiles",
        "-specs=nano.specs",                  # компактная libc: нужен snprintf
        "-Wl,--gc-sections", "-Wl,--nmagic",  # без выравнивания сегмента на страницу
        "-Wl,-Map=" + os.path.join(BUILD, name + ".map")]
    run([CC] + ldflags + objs + ["-lc", "-lm", "-lgcc", "-o", elf])
    bin_ = os.path.join(BUILD, name + ".bin")
    run([OBJCOPY, "-O", "binary", elf, bin_])
    run([SIZE, elf])
    print("%s: %d байт" % (bin_, os.path.getsize(bin_)))
    return bin_


which = sys.argv[1:] or ["boot", "app"]
built = {n: build(n) for n in which}

if "app" in built:
    run([sys.executable, os.path.join(ROOT, "tools", "mkwbfw.py"),
         built["app"], os.path.join(BUILD, "app.wbfw")])

