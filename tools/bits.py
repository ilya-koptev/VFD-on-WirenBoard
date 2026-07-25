"""Восстановить слово, которое реально защёлкивается в регистре, по захвату
логического анализатора: сэмплируем SIN на каждом нарастающем фронте CLK.

Не нужна обратка SO1: смотрим ровно то, что видит вход модуля.
Оговорка: порог анализатора ~1.4 В, а VIL модуля 0.7 В, поэтому модуль может
увидеть БОЛЬШЕ единиц, чем видим мы — оценка снизу.

  python bits.py --expect 100        # ждём одну единицу в позиции 100
"""
import argparse
import os
import subprocess

import numpy as np

CLI = r"C:\Program Files\sigrok\sigrok-cli\sigrok-cli.exe"
HERE = os.path.dirname(os.path.abspath(__file__))
BIN = os.path.join(HERE, "cap3.bin")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rate", type=int, default=24000000)
    ap.add_argument("--samples", type=int, default=1200000)
    ap.add_argument("--expect", type=int, default=None, help="позиция единственной ожидаемой единицы")
    ap.add_argument("--bursts", type=int, default=3, help="сколько посылок показать")
    a = ap.parse_args()

    subprocess.run([CLI, "--driver", "fx2lafw", "--config", "samplerate=%d" % a.rate,
                    "--samples", str(a.samples), "-O", "binary", "-o", BIN],
                   capture_output=True, timeout=180)
    d = np.fromfile(BIN, dtype=np.uint8)
    if d.size == 0:
        raise SystemExit("захват пуст")
    us = 1e6 / a.rate
    sin = (d >> 0) & 1
    clk = (d >> 1) & 1

    rise = np.where(np.diff(clk.astype(np.int8)) > 0)[0] + 1
    if rise.size < 10:
        raise SystemExit("на CLK нет фронтов")
    per = np.diff(rise) * us
    med = np.median(per)
    print("тактов %d, период медиана %.3f мкс (%.2f МГц), отсчётов на такт %.1f"
          % (rise.size, med, 1 / med, med / us))
    if med / us < 3:
        print("ВНИМАНИЕ: меньше 3 отсчётов на такт — сэмплирование ненадёжно, замедли SPI")

    gaps = np.where(per > max(8 * med, 15))[0]
    bursts = np.split(rise, gaps + 1)
    bursts = [b for b in bursts if b.size > 100]
    print("посылок %d, тактов в посылке: %s" % (len(bursts), [b.size for b in bursts[:6]]))

    for k, b in enumerate(bursts[:a.bursts]):
        word = sin[b]                       # уровень SIN на каждом фронте = защёлкнутый бит
        ones = np.where(word)[0]
        runs, i = [], 0
        while i < ones.size:
            j = i
            while j + 1 < ones.size and ones[j + 1] == ones[j] + 1:
                j += 1
            runs.append((int(ones[i]), int(ones[j] - ones[i] + 1)))
            i = j + 1
        txt = ", ".join("%d:%d" % r for r in runs) if runs else "нет единиц"
        print("посылка %d (%d тактов): %s" % (k, b.size, txt))
        if a.expect is not None:
            ok = len(runs) == 1 and runs[0] == (a.expect, 1)
            print("   ожидали %d:1 -> %s" % (a.expect, "ЧИСТО" if ok else "СМАЗ/ЛИШНИЕ БИТЫ"))


if __name__ == "__main__":
    main()
