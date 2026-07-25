"""Целостность записи в регистр: пишем одиночный бит, читаем через SO1.

Для каждой комбинации (bbdly, rtz) печатаем найденные прогоны единиц:
идеал — один прогон, начинающийся в заданной позиции, длиной 1.
Растяжение = сколько лишних разрядов «протянула» единица.
"""
import re
import sys
import time

import serial

PORT, BAUD = "COM4", 115200
POS = 100          # куда кладём одиночный бит в 240-битном потоке


def runs_of_ones(hexstr, nbits):
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


def main():
    nbits = 200
    with serial.Serial(PORT, BAUD, timeout=0.4) as s:
        time.sleep(0.3)
        s.reset_input_buffer()

        def cmd(c, wait=0.25):
            s.write((c + "\r\n").encode())
            time.sleep(wait)
            return s.read(20000).decode("utf-8", "replace")

        cmd("bb 1")
        print("bbdly rtz | прогоны единиц (позиция:длина)")
        print("-" * 60)
        for bbdly in (0, 1, 2, 4, 8):
            for rtz in (0, 1):
                cmd("rz 240")
                cmd("rb %d 1" % POS)
                cmd("cfg bbdly %d" % bbdly)
                cmd("cfg rtz %d" % rtz)
                out = cmd("sord %d" % nbits, wait=2.5)
                m = re.search(r"recv:\s*([0-9A-F]+)", out)
                if not m:
                    print("%5d %3d | нет ответа" % (bbdly, rtz))
                    continue
                r = runs_of_ones(m.group(1), nbits)
                desc = ", ".join("%d:%d" % (a, b) for a, b in r) if r else "ПУСТО (бит не захвачен)"
                print("%5d %3d | %s" % (bbdly, rtz, desc))


if __name__ == "__main__":
    main()
