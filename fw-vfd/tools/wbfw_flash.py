"""Прошить модуль по протоколу обновления Wiren Board — стендовый аналог
wb-mcu-fw-flasher для Windows, где libmodbus нет.

Делает ровно то же, что штатный флешер: при -j пишет 1 в регистр 129, потом
BEGIN (функция 0x10, 16 регистров по 0x1000) и куски DATA (0x10, 68 регистров
по 0x2000). На контроллере то же самое делается штатной командой:

    wb-mcu-fw-flasher -j -d /dev/ttyMOD1 -a 1 -f vfd.wbfw

Здесь:
    python wbfw_flash.py COM4 app.wbfw [-j] [--addr 1] [--baud 9600]
"""
import struct
import sys
import time

import serial

INFO_SIZE = 32
CHUNK = 136
REG_JUMP_STANDARD = 129
REG_INFO = 0x1000
REG_DATA = 0x2000


def crc16(d):
    crc = 0xFFFF
    for b in d:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def frame(body):
    c = crc16(body)
    return body + bytes([c & 0xFF, c >> 8])


def request(port, body, expect, timeout=10.0):
    port.reset_input_buffer()
    port.write(frame(body))
    port.flush()
    deadline = time.time() + timeout
    got = b""
    while time.time() < deadline:
        got += port.read(max(1, port.in_waiting))
        if len(got) >= expect:
            break
        time.sleep(0.005)
    if len(got) < 3:
        raise SystemExit("нет ответа: %r" % got)
    if got[1] & 0x80:
        codes = {1: "неизвестная функция", 2: "не в режиме загрузчика",
                 3: "недопустимое значение", 4: "подпись не совпала"}
        raise SystemExit("ошибка Modbus %d: %s" % (got[2], codes.get(got[2], "?")))
    return got


def write_regs(port, addr, reg, payload, timeout=10.0):
    nregs = len(payload) // 2
    body = bytes([addr, 0x10]) + struct.pack(">HHB", reg, nregs, len(payload)) + payload
    return request(port, body, 8, timeout)


args = sys.argv[1:]
if len(args) < 2:
    raise SystemExit(__doc__)
com, path = args[0], args[1]
jump = "-j" in args
addr = int(args[args.index("--addr") + 1]) if "--addr" in args else 1
baud = int(args[args.index("--baud") + 1]) if "--baud" in args else 9600

data = open(path, "rb").read()
info, body = data[:INFO_SIZE], data[INFO_SIZE:]
if len(body) % CHUNK:
    raise SystemExit("тело не кратно %d байт — пересобери mkwbfw.py" % CHUNK)

if jump:
    # прыжок в загрузчик отправляем на текущей скорости приложения
    app = serial.Serial(com, 115200, timeout=1)
    app.write(frame(bytes([addr, 0x06]) + struct.pack(">HH", REG_JUMP_STANDARD, 1)))
    app.flush()
    time.sleep(0.3)
    app.close()
    print("отправлен прыжок в загрузчик (регистр %d)" % REG_JUMP_STANDARD)
    time.sleep(0.5)

p = serial.Serial(com, baud, stopbits=serial.STOPBITS_TWO, timeout=1)
time.sleep(0.2)

print("инфоблок: подпись %s, %d байт тела" %
      (info[:12].rstrip(b"\0").decode(errors="replace"), struct.unpack(">I", info[12:16])[0]))
write_regs(p, addr, REG_INFO, info, timeout=15.0)   # тут же идёт стирание флеша
print("инфоблок принят, флеш стёрт")

total = len(body) // CHUNK
t0 = time.time()
for i in range(total):
    write_regs(p, addr, REG_DATA, body[i * CHUNK:(i + 1) * CHUNK])
    if (i + 1) % 25 == 0 or i + 1 == total:
        print("  кусок %d/%d" % (i + 1, total))
p.close()
print("готово за %.1f с" % (time.time() - t0))
