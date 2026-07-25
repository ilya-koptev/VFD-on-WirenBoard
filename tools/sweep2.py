"""Развёртка по шаблону команды + численные профили (строки/столбцы).

  python sweep2.py --cmd "rz 240;rb {} 6;rb 192 2" --vals 0:192:6
  python sweep2.py --cmd "rz 240;rb 0 6;rb {} 2" --vals 192:236:2
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
    return "".join(LEV[min(9, int(x * 9.99))] for x in v)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cmd", required=True, help="шаблон команд, {} = подставляемое значение")
    ap.add_argument("--vals", required=True, help="start:stop:step")
    ap.add_argument("--exp", type=float, default=-7)
    ap.add_argument("--bars", action="store_true", help="печатать профили-полоски")
    ap.add_argument("--out", default=os.path.join(HERE, "sweep2.txt"))
    a = ap.parse_args()

    st, sp, step = (int(x) for x in a.vals.split(":"))
    x0, y0, x1, y1 = BOX

    cap = cv2.VideoCapture(1, cv2.CAP_MSMF)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
    cap.set(cv2.CAP_PROP_EXPOSURE, a.exp)

    out = []
    with serial.Serial(PORT, BAUD, timeout=0.3) as s:
        time.sleep(0.3)
        s.reset_input_buffer()
        for v in range(st, sp, step):
            for part in a.cmd.format(v).split(";"):
                s.write((part.strip() + "\r\n").encode())
                time.sleep(0.08)
            s.read(4000)
            time.sleep(0.15)
            for _ in range(6):
                ok, fr = cap.read()
            g = cv2.cvtColor(fr[y0:y1, x0:x1], cv2.COLOR_BGR2GRAY).astype(np.float32)
            g = cv2.resize(g, (32, 32), interpolation=cv2.INTER_AREA)
            m = g.mean()
            row, col = g.mean(axis=1), g.mean(axis=0)
            if m < 5:
                line = "v=%3d m=%5.1f DARK" % (v, m)
            else:
                rn, cn = row / row.max(), col / col.max()
                ri, ci = np.where(rn > 0.6)[0], np.where(cn > 0.6)[0]
                cb = np.clip(col - col.min(), 0, None)
                cen = float((cb * np.arange(32)).sum() / max(cb.sum(), 1e-6))
                line = "v=%3d m=%5.1f rows %2d-%2d cols %2d-%2d peak=%2d cen=%4.1f" % (
                    v, m, ri.min(), ri.max(), ci.min(), ci.max(), int(np.argmax(col)), cen)
                if a.bars:
                    line += " R|%s| C|%s|" % (bar(rn), bar(cn))
            print(line)
            out.append(line)
    cap.release()
    open(a.out, "w").write("\n".join(out) + "\n")


if __name__ == "__main__":
    main()
