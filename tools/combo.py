"""Полный перебор упаковки сеточных/анодных бит в РАБОЧЕМ режиме (44-слотовый скан).

Тест: vline 20 — должна получиться УЗКАЯ вертикальная полоса (мало столбцов, вся высота).
Контроль: hline 8 — одна горизонтальная линия (много столбцов, мало строк).
Победитель: cols мало И rows много.
"""
import itertools
import json
import os
import time

import cv2
import numpy as np
import serial

PORT, BAUD = "COM4", 115200
HERE = os.path.dirname(os.path.abspath(__file__))
BOX = json.load(open(os.path.join(HERE, "box.json")))
EXP = -7


def measure(cap, s, cmds):
    for c in cmds:
        s.write((c + "\r\n").encode())
        time.sleep(0.07)
    s.read(4000)
    time.sleep(0.2)
    for _ in range(6):
        ok, fr = cap.read()
    x0, y0, x1, y1 = BOX
    g = cv2.cvtColor(fr[y0:y1, x0:x1], cv2.COLOR_BGR2GRAY).astype(np.float32)
    g = cv2.resize(g, (32, 32), interpolation=cv2.INTER_AREA)
    m = g.mean()
    if m < 4:
        return m, 0, 0
    row, col = g.mean(axis=1), g.mean(axis=0)
    nr = int((row / row.max() > 0.5).sum())
    nc = int((col / col.max() > 0.5).sum())
    return m, nr, nc


def main():
    cap = cv2.VideoCapture(1, cv2.CAP_MSMF)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
    cap.set(cv2.CAP_PROP_EXPOSURE, EXP)

    rows = []
    with serial.Serial(PORT, BAUD, timeout=0.3) as s:
        time.sleep(0.3)
        s.reset_input_buffer()
        for c in ["bb 1", "noraw", "gall", "cfg mode 0", "cfg on 200", "cfg latpol 0",
                  "cfg miry 1", "cfg hold 0"]:
            s.write((c + "\r\n").encode())
            time.sleep(0.1)
        s.read(4000)

        for triad, par, mir, dbl, gstep in itertools.product((0, 1), (0, 1), (0, 1), (0, 1), (1, 2)):
            pre = ["cfg triad %d" % triad, "cfg par %d" % par, "cfg mir %d" % mir,
                   "cfg dbl %d" % dbl, "cfg gstep %d" % gstep,
                   "cfg slots %d" % (44 if gstep == 1 else 22)]
            mv, nrv, ncv = measure(cap, s, pre + ["vline 20"])
            mh, nrh, nch = measure(cap, s, pre + ["hline 8"])
            tag = "t%d p%d m%d d%d g%d" % (triad, par, mir, dbl, gstep)
            score = (nrv >= 18 and ncv <= 8 and nch >= 18 and nrh <= 8)
            line = ("%s | vline m=%5.1f rows=%2d cols=%2d | hline m=%5.1f rows=%2d cols=%2d %s"
                    % (tag, mv, nrv, ncv, mh, nrh, nch, "  <<< WIN" if score else ""))
            print(line)
            rows.append(line)
    cap.release()
    open(os.path.join(HERE, "combo.txt"), "w").write("\n".join(rows) + "\n")


if __name__ == "__main__":
    main()
