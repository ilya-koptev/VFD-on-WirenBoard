"""Замер в родном разрешении 128x32 через DSHOW (MSMF врёт про экспозицию).

  python dot.py --cmd "probe {}" --vals 97:103:1 --exp -7
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


def open_cam(exp):
    cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
    if not cap.isOpened():
        raise SystemExit("камера не открылась")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_EXPOSURE, exp)
    got = cap.get(cv2.CAP_PROP_EXPOSURE)
    for _ in range(10):
        cap.read()
    print("# exp запрошена %s, применена %s" % (exp, got))
    return cap


def grab(cap, x0, y0, x1, y1, navg):
    acc = None
    for _ in range(navg + 2):
        ok, fr = cap.read()
        if not ok:
            continue
        g = cv2.cvtColor(fr[y0:y1, x0:x1], cv2.COLOR_BGR2GRAY).astype(np.float32)
        g = cv2.resize(g, (128, 32), interpolation=cv2.INTER_AREA)
        acc = g if acc is None else acc + g
    return acc / (navg + 2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cmd", required=True)
    ap.add_argument("--vals", required=True)
    ap.add_argument("--exp", type=float, default=-7)
    ap.add_argument("--avg", type=int, default=4)
    ap.add_argument("--pad", type=int, default=12)
    ap.add_argument("--thr", type=float, default=0.5)
    a = ap.parse_args()

    st, sp, step = (int(x) for x in a.vals.split(":"))
    x0, y0, x1, y1 = BOX
    x0, y0, x1, y1 = x0 + a.pad, y0 + a.pad, x1 - a.pad, y1 - a.pad
    cap = open_cam(a.exp)

    with serial.Serial(PORT, BAUD, timeout=0.3) as s:
        time.sleep(0.3)
        s.reset_input_buffer()
        for v in range(st, sp, step):
            for part in a.cmd.format(v).split(";"):
                s.write((part.strip() + "\r\n").encode())
                time.sleep(0.08)
            s.read(4000)
            time.sleep(0.2)
            g = grab(cap, x0, y0, x1, y1, a.avg)
            mx = float(g.max())
            if mx < 10:
                print("v=%3d DARK (max=%.0f)" % (v, mx))
                continue
            ry, rx = np.unravel_index(int(np.argmax(g)), g.shape)
            colp, rowp = g.max(axis=0), g.max(axis=1)
            hc = np.where(colp > a.thr * mx)[0]
            hr = np.where(rowp > a.thr * mx)[0]
            print("v=%3d max=%3.0f peak=(x=%3d,y=%2d)  cols %3d-%3d (n=%2d)  rows %2d-%2d (n=%2d)"
                  % (v, mx, rx, ry, hc.min(), hc.max(), len(hc), hr.min(), hr.max(), len(hr)))
    cap.release()


if __name__ == "__main__":
    main()
