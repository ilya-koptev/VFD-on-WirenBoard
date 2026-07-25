"""Стендовая тула MN12832L: слать команды в прошивку по COM4 и снимать дисплей камерой.

  python vfd.py send "st" "fill"
  python vfd.py shot --exp -6 --out fill.png [--find]
  python vfd.py seq  --exp -6 --out montage.png "clr" "fill" "border"

Камера: index 1 = внешняя USB, направлена на дисплей. index 0 (встроенная) НЕ используется.
"""
import argparse
import json
import os
import sys
import time

import cv2
import numpy as np
import serial

PORT = "COM4"
BAUD = 115200
HERE = os.path.dirname(os.path.abspath(__file__))
BOXFILE = os.path.join(HERE, "box.json")


def send(cmds, wait=0.25):
    outs = []
    with serial.Serial(PORT, BAUD, timeout=0.3) as s:
        time.sleep(0.25)
        s.reset_input_buffer()
        for c in cmds:
            s.write((c + "\r\n").encode())
            time.sleep(wait)
            outs.append(s.read(8000).decode("utf-8", "replace"))
    return outs


def grab(exp=-6, index=1, warm=10):
    # DSHOW, а не MSMF: MSMF врёт про экспозицию (get всегда -6, кадр отстаёт на шаг)
    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    if not cap.isOpened():
        raise SystemExit("камера index=%d не открылась" % index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_EXPOSURE, exp)
    fr = None
    for _ in range(warm):
        ok, f = cap.read()
        if ok:
            fr = f
    cap.release()
    if fr is None:
        raise SystemExit("кадр не получен")
    return fr


def find_box(fr, pad=10):
    """Ищем яркую сине-зелёную зону = поле VFD."""
    b, g, r = cv2.split(fr.astype(np.int16))
    score = np.clip(b + g - 2 * r, 0, 255).astype(np.uint8)
    score = cv2.GaussianBlur(score, (9, 9), 0)
    thr = max(30, int(score.max() * 0.45))
    mask = (score >= thr).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 25), np.uint8))
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    x, y, w, h = cv2.boundingRect(max(cnts, key=cv2.contourArea))
    H, W = fr.shape[:2]
    return [max(0, x - pad), max(0, y - pad), min(W, x + w + pad), min(H, y + h + pad)]


def load_box():
    if os.path.exists(BOXFILE):
        return json.load(open(BOXFILE))
    return None


def save_box(box):
    json.dump(box, open(BOXFILE, "w"))


def crop(fr, box, scale=3):
    if box:
        x0, y0, x1, y1 = box
        fr = fr[y0:y1, x0:x1]
    if scale != 1 and fr.size:
        fr = cv2.resize(fr, (fr.shape[1] * scale, fr.shape[0] * scale), interpolation=cv2.INTER_NEAREST)
    return fr


def stats(fr):
    g = cv2.cvtColor(fr, cv2.COLOR_BGR2GRAY)
    return "mean=%.1f max=%d p99=%.0f" % (g.mean(), g.max(), np.percentile(g, 99))


def label(img, text):
    bar = np.zeros((26, img.shape[1], 3), np.uint8)
    cv2.putText(bar, text[:70], (4, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
    return np.vstack([bar, img])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=["send", "shot", "seq"])
    ap.add_argument("cmds", nargs="*")
    ap.add_argument("--exp", type=float, default=-6)
    ap.add_argument("--out", default=os.path.join(HERE, "shot.png"))
    ap.add_argument("--find", action="store_true", help="перелокализовать поле дисплея")
    ap.add_argument("--full", action="store_true", help="без кропа")
    ap.add_argument("--scale", type=int, default=3)
    ap.add_argument("--wait", type=float, default=0.25)
    a = ap.parse_args()

    if a.mode == "send":
        for c, o in zip(a.cmds, send(a.cmds, a.wait)):
            print("$ %s\n%s" % (c, o.strip()))
        return

    box = None if a.full else load_box()

    if a.mode == "shot":
        if a.cmds:
            send(a.cmds, a.wait)
            time.sleep(0.3)
        fr = grab(a.exp)
        if a.find or (box is None and not a.full):
            nb = find_box(fr)
            if nb:
                box = nb
                save_box(nb)
                print("box =", nb)
        img = crop(fr, box, a.scale)
        cv2.imwrite(a.out, img)
        print("%s  %s  size=%s" % (a.out, stats(img), img.shape[:2]))
        return

    # seq: команда -> снимок -> монтаж
    tiles = []
    for c in a.cmds:
        send([p.strip() for p in c.split(";") if p.strip()], a.wait)
        time.sleep(0.3)
        fr = grab(a.exp)
        img = crop(fr, box, a.scale)
        tiles.append(label(img, "%s   %s" % (c, stats(img))))
        print("%-28s %s" % (c, stats(img)))
    w = max(t.shape[1] for t in tiles)
    tiles = [cv2.copyMakeBorder(t, 0, 4, 0, w - t.shape[1], cv2.BORDER_CONSTANT, value=(40, 40, 40)) for t in tiles]
    cv2.imwrite(a.out, np.vstack(tiles))
    print("montage:", a.out)


if __name__ == "__main__":
    main()
