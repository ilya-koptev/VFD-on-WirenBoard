"""Измерение карты сдвигового регистра MN12832L по камере.

Для каждой позиции блока единиц в потоке снимаем кадр и считаем профили:
  ROW  — 32 бина по вертикали (какие строки горят)
  COL  — 32 бина по горизонтали (какие столбцы горят)
Вывод компактный, чтобы карту можно было читать глазами как таблицу.

  python map.py --start 528 --stop 600 --step 6 --n 6
"""
import argparse
import json
import os
import time

import cv2
import numpy as np
import serial

PORT, BAUD = "COM4", 115200
HERE = os.path.dirname(os.path.abspath(__file__))
BOX = json.load(open(os.path.join(HERE, "box.json")))
LEV = " .:-=+*#%@"


def bar(v):
    """массив 0..1 -> строка символов"""
    return "".join(LEV[min(9, int(x * 9.99))] for x in v)


def profiles(fr, nrow=32, ncol=32):
    x0, y0, x1, y1 = BOX
    g = cv2.cvtColor(fr[y0:y1, x0:x1], cv2.COLOR_BGR2GRAY).astype(np.float32)
    g = cv2.resize(g, (ncol, nrow), interpolation=cv2.INTER_AREA)
    row = g.mean(axis=1)
    col = g.mean(axis=0)
    return row, col, g.mean()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--start", type=int, default=528)
    ap.add_argument("--stop", type=int, default=600)
    ap.add_argument("--step", type=int, default=6)
    ap.add_argument("--n", type=int, default=6)
    ap.add_argument("--total", type=int, default=600)
    ap.add_argument("--exp", type=float, default=-6)
    ap.add_argument("--out", default=os.path.join(HERE, "map.txt"))
    a = ap.parse_args()

    cap = cv2.VideoCapture(1, cv2.CAP_MSMF)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
    cap.set(cv2.CAP_PROP_EXPOSURE, a.exp)

    lines = []
    with serial.Serial(PORT, BAUD, timeout=0.3) as s:
        time.sleep(0.3)
        s.reset_input_buffer()
        for pos in range(a.start, a.stop, a.step):
            s.write(("raw %d %d %d\r\n" % (pos, a.n, a.total)).encode())
            time.sleep(0.2)
            s.read(2000)
            for _ in range(6):
                ok, fr = cap.read()
            row, col, m = profiles(fr)
            mx = max(row.max(), 1.0)
            rn = row / mx
            cn = col / max(col.max(), 1.0)
            if m < 6:                      # темно — мерить нечего
                txt = "s=%3d m=%5.1f DARK" % (pos, m)
            else:
                ri = np.where(rn > 0.5)[0]
                ci = np.where(cn > 0.5)[0]
                txt = ("s=%3d m=%5.1f rows %2d-%2d (%2d) cols %2d-%2d (%2d) w=%3d%% %s"
                       % (pos, m, ri.min(), ri.max(), len(ri), ci.min(), ci.max(), len(ci),
                          100 * len(ci) // 32, "<<< NARROW" if len(ci) < 20 else ""))
            print(txt)
            lines.append(txt)
    cap.release()
    open(a.out, "w").write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
