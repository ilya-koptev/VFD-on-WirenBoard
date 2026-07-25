"""Захват CLK/SIN логическим анализатором (Hantek 6022BL в режиме fx2lafw) и разбор.

Подключение: SIN -> D0, CLK -> D1, земля анализатора -> земля стенда.

Главные метрики:
  * сколько импульсов CLK в одной посылке (должно быть ровно 240 на слот);
  * сколько фронтов CLK попадает на один высокий уровень SIN (должно быть 1);
  * период такта и его разброс.

  python la.py [--samples 800000] [--rate 24000000]
"""
import argparse
import os
import subprocess

import numpy as np

CLI = r"C:\Program Files\sigrok\sigrok-cli\sigrok-cli.exe"
HERE = os.path.dirname(os.path.abspath(__file__))
BIN = os.path.join(HERE, "cap.bin")
SIN_CH, CLK_CH = 0, 1


def capture(rate, samples):
    cmd = [CLI, "--driver", "fx2lafw", "--config", "samplerate=%d" % rate,
           "--samples", str(samples), "-O", "binary", "-o", BIN]
    subprocess.run(cmd, capture_output=True, timeout=120)
    return np.fromfile(BIN, dtype=np.uint8)


def edges(bits):
    d = np.diff(bits.astype(np.int8))
    return np.where(d > 0)[0] + 1, np.where(d < 0)[0] + 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rate", type=int, default=24000000)
    ap.add_argument("--samples", type=int, default=800000)
    a = ap.parse_args()

    d = capture(a.rate, a.samples)
    if d.size == 0:
        raise SystemExit("захват пуст — проверь подключение анализатора")
    us = 1e6 / a.rate
    sin = (d >> SIN_CH) & 1
    clk = (d >> CLK_CH) & 1
    print("сэмплов %d (%.2f мс), шаг %.3f мкс" % (d.size, d.size * us / 1000, us))
    print("SIN: доля высокого %.3f | CLK: доля высокого %.3f" % (sin.mean(), clk.mean()))

    cr, cf = edges(clk)
    if cr.size < 2:
        raise SystemExit("на CLK нет фронтов — не тот канал или нет сигнала")
    per = np.diff(cr) * us
    print("CLK: нарастающих фронтов %d, период мкс: мин %.2f сред %.2f макс %.2f"
          % (cr.size, per.min(), per.mean(), per.max()))

    # посылки = группы тактов, разделённые паузой больше 10 периодов
    gap = np.where(per > max(10 * np.median(per), 20))[0]
    bursts = np.split(cr, gap + 1)
    lens = [b.size for b in bursts if b.size > 5]
    if lens:
        print("посылок %d, тактов в посылке: мин %d медиана %d макс %d  (ожидаем 240)"
              % (len(lens), min(lens), int(np.median(lens)), max(lens)))

    # сколько фронтов CLK попадает на каждый высокий уровень SIN
    sr, sf = edges(sin)
    n = min(sr.size, sf.size)
    if n:
        cnt = []
        for i in range(min(n, 200)):
            lo, hi = sr[i], sf[i] if sf[i] > sr[i] else sr[i] + 1
            cnt.append(int(np.count_nonzero((cr >= lo) & (cr <= hi))))
        cnt = np.array(cnt)
        print("высоких уровней SIN: %d, фронтов CLK на каждый: мин %d сред %.2f макс %d  (ожидаем 1)"
              % (sr.size, cnt.min(), cnt.mean(), cnt.max()))
        w = (sf[:n] - sr[:n]) * us
        print("длительность высокого SIN, мкс: мин %.2f сред %.2f макс %.2f" % (w.min(), w.mean(), w.max()))


if __name__ == "__main__":
    main()
