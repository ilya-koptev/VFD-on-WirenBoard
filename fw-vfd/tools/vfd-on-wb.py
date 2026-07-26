"""Помощник для VFD-модуля на контроллере: консоль приложения и опрос загрузчика.

    python3 vfd.py con "st" "clock 0"    команды консоли приложения, 115200
    python3 vfd.py boot                  перевести в загрузчик
    python3 vfd.py probe                 прочитать подпись и версию загрузчика
"""
import struct
import sys
import time

import serial

PORT = "/dev/ttyACM0"


def crc16(d):
    c = 0xFFFF
    for b in d:
        c ^= b
        for _ in range(8):
            c = (c >> 1) ^ 0xA001 if c & 1 else c >> 1
    return c


def console(cmds):
    p = serial.Serial(PORT, 115200, timeout=2)
    time.sleep(0.5)
    p.reset_input_buffer()
    for c in cmds:
        p.write((c + "\r\n").encode())
        time.sleep(0.6)
        print(p.read(p.in_waiting or 1).decode("utf-8", "replace").strip())
    p.close()


def probe():
    p = serial.Serial(PORT, 9600, stopbits=serial.STOPBITS_TWO, timeout=1)
    time.sleep(0.3)
    for reg, n in ((290, 12), (330, 8)):
        body = bytes([1, 3]) + struct.pack(">HH", reg, n)
        c = crc16(body)
        p.reset_input_buffer()
        p.write(body + bytes([c & 0xFF, c >> 8]))
        time.sleep(0.5)
        r = p.read(p.in_waiting or 1)
        txt = bytes(r[4:4 + n * 2:2]).decode("ascii", "replace").rstrip("\0") if len(r) >= 4 + n * 2 else ""
        print("регистр %-4d %s -> %r" % (reg, r.hex() if r else "молчит", txt))
    p.close()


cmd = sys.argv[1] if len(sys.argv) > 1 else "probe"
if cmd == "con":
    console(sys.argv[2:])
elif cmd == "boot":
    console(["boot"])
    print("ушёл в загрузчик")
else:
    probe()
