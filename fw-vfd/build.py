"""Локальная сборка загрузчика теми же флагами, что в Makefile.

На этом стенде (Windows) make не установлен, а тулчейн лежит в пакетах
PlatformIO. Скрипт повторяет цели Makefile один в один, чтобы бинарь получался
тот же самый; на контроллере собирается обычным make.
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

MCU, CPU, TARGET = "STM32F411xE", "cortex-m4", "boot"
SRC = ["src/boot/main.c", "system/startup.c"]
LDSCRIPT = "ldscripts/stm32f411_boot.ld"
BUILD = os.path.join(ROOT, "build")

CFLAGS = ["-mcpu=" + CPU, "-mthumb", "-Os", "-std=gnu11",
          "-ffunction-sections", "-fdata-sections", "-fno-common",
          "-Wall", "-Wextra", "-D" + MCU,
          "-I" + os.path.join(ROOT, "include"),
          "-I" + os.path.join(CMSIS, "Include"),
          "-I" + os.path.join(CMSIS, "Device", "ST", "STM32F4xx", "Include")]

LDFLAGS = ["-mcpu=" + CPU, "-mthumb", "-T" + os.path.join(ROOT, LDSCRIPT),
           "-nostartfiles", "-Wl,--gc-sections",
           "-Wl,-Map=" + os.path.join(BUILD, TARGET + ".map")]


def run(cmd):
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.stdout.strip():
        print(r.stdout.strip())
    if r.returncode:
        print(r.stderr.strip())
        sys.exit(r.returncode)
    elif r.stderr.strip():
        print(r.stderr.strip())


os.makedirs(BUILD, exist_ok=True)
objs = []
for s in SRC:
    o = os.path.join(BUILD, os.path.basename(s).replace(".c", ".o"))
    run([CC] + CFLAGS + ["-c", os.path.join(ROOT, s), "-o", o])
    objs.append(o)

elf = os.path.join(BUILD, TARGET + ".elf")
run([CC] + LDFLAGS + objs + ["-o", elf])
run([OBJCOPY, "-O", "binary", elf, os.path.join(BUILD, TARGET + ".bin")])
run([SIZE, elf])
print("загрузчик собран:", os.path.join(BUILD, TARGET + ".bin"),
      os.path.getsize(os.path.join(BUILD, TARGET + ".bin")), "байт")
