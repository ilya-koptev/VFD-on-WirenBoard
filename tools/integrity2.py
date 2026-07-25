"""Отделяем смаз ЗАПИСИ от медленного спада линии SO1 при ЧТЕНИИ.

Пишем одиночный бит, читаем на разных скоростях чтения (rddly).
Если прогон единиц — артефакт чтения (постоянная времени спада), то при
замедлении чтения он должен занимать МЕНЬШЕ отсчётов. Если это смаз записи,
длина в отсчётах от скорости чтения не зависит.
"""
import re
import time

import serial

PORT, BAUD = "COM4", 115200
POS = 100


def runs(hexstr, nbits):
    bits = bin(int(hexstr, 16))[2:].zfill(len(hexstr) * 4)[:nbits]
    out, i = [], 0
    while i < len(bits):
        if bits[i] == "1":
            j = i
            while j < len(bits) and bits[j] == "1":
                j += 1
            out.append((i, j - i))
            i = j
        else:
            i += 1
    return out


with serial.Serial(PORT, BAUD, timeout=0.4) as s:
    time.sleep(0.3)
    s.reset_input_buffer()

    def cmd(c, wait=0.25):
        s.write((c + "\r\n").encode())
        time.sleep(wait)
        return s.read(30000).decode("utf-8", "replace")

    cmd("bb 1")
    cmd("cfg rtz 1")
    print("запись   чтение | прогон (позиция:длина)")
    print("-" * 52)
    for bbdly in (0, 8):
        for rddly in (2, 10, 40):
            cmd("rz 240")
            cmd("rb %d 1" % POS)
            cmd("cfg bbdly %d" % bbdly)
            cmd("cfg rddly %d" % rddly)
            out = cmd("sord 160", wait=3.5 + rddly * 0.05)
            m = re.search(r"recv:\s*([0-9A-F]+)", out)
            r = runs(m.group(1), 160) if m else []
            desc = ", ".join("%d:%d" % (a, b) for a, b in r) if r else "ПУСТО"
            print("bbdly=%-3d rddly=%-3d | %s" % (bbdly, rddly, desc))
