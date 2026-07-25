"""Разбор захвата по-битно: на каких номерах тактов SIN был высоким.

Отвечает на вопрос: единица, положенная в позицию N, действительно приходит
одним битом, или линия держится высокой много тактов подряд (тот самый смаз).
"""
import os
import subprocess

import numpy as np

CLI = r"C:\Program Files\sigrok\sigrok-cli\sigrok-cli.exe"
HERE = os.path.dirname(os.path.abspath(__file__))
BIN = os.path.join(HERE, "cap.bin")
RATE = 24000000
SIN_CH, CLK_CH = 0, 1


def main():
    subprocess.run([CLI, "--driver", "fx2lafw", "--config", "samplerate=%d" % RATE,
                    "--samples", "800000", "-O", "binary", "-o", BIN],
                   capture_output=True, timeout=120)
    d = np.fromfile(BIN, dtype=np.uint8)
    sin = (d >> SIN_CH) & 1
    clk = (d >> CLK_CH) & 1
    us = 1e6 / RATE

    rise = np.where(np.diff(clk.astype(np.int8)) > 0)[0] + 1
    fall = np.where(np.diff(clk.astype(np.int8)) < 0)[0] + 1
    per = np.diff(rise) * us
    gaps = np.where(per > max(10 * np.median(per), 20))[0]
    bursts = np.split(rise, gaps + 1)

    print("шаг %.3f мкс, период такта медиана %.2f мкс" % (us, np.median(per)))
    shown = 0
    for b in bursts:
        if b.size < 200:
            continue
        # уровень SIN на каждом нарастающем и на каждом спадающем фронте
        hi_r = [i for i, s in enumerate(b) if sin[s]]
        # спады внутри этой посылки
        f = fall[(fall >= b[0]) & (fall <= b[-1] + 5)]
        hi_f = [i for i, s in enumerate(f) if sin[min(s, sin.size - 1)]]
        print("\nпосылка: тактов %d" % b.size)
        print("  SIN высокий на нарастающих фронтах: %s" % (hi_r if len(hi_r) < 40 else
              "%s ... всего %d" % (hi_r[:12], len(hi_r))))
        print("  SIN высокий на спадающих фронтах:   %s" % (hi_f if len(hi_f) < 40 else
              "%s ... всего %d" % (hi_f[:12], len(hi_f))))
        # временная развёртка вокруг первого высокого уровня
        if hi_r:
            c = b[hi_r[0]]
            w0, w1 = max(0, c - 400), min(d.size, c + 1200)
            step = max(1, (w1 - w0) // 120)
            s_line = "".join("#" if sin[i] else "." for i in range(w0, w1, step))
            c_line = "".join("#" if clk[i] else "." for i in range(w0, w1, step))
            print("  окно %.1f мкс (символ = %.2f мкс):" % ((w1 - w0) * us, step * us))
            print("    SIN |%s|" % s_line)
            print("    CLK |%s|" % c_line)
        shown += 1
        if shown >= 2:
            break


if __name__ == "__main__":
    main()
