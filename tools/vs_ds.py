"""Сверка измеренного обмена с таблицей INTERFACE TIMING даташита MN12832L.

Каналы: SIN -> D0, CLK -> D1 (по желанию LAT -> D2, BLK -> D3).
Разрешение при 24 МГц = 41.7 нс, поэтому setup/hold в 40-50 нс мы можем
подтвердить лишь с точностью до одного отсчёта — это отмечено в выводе.
"""
import os
import subprocess

import numpy as np

CLI = r"C:\Program Files\sigrok\sigrok-cli\sigrok-cli.exe"
HERE = os.path.dirname(os.path.abspath(__file__))
BIN = os.path.join(HERE, "cap2.bin")
RATE = 24000000
US = 1e6 / RATE

# параметр -> (минимум по даташиту в мкс, комментарий)
DS = {
    "CLK Cycle":    (0.400, "период такта"),
    "CLK High":     (0.200, "высокий уровень такта"),
    "CLK Low":      (0.200, "низкий уровень такта"),
    "SIN Setup":    (0.040, "данные до фронта CLK"),
    "SIN Hold":     (0.050, "данные после фронта CLK"),
    "LAT High":     (0.300, "длительность LATCH"),
    "CLK then LAT": (0.250, "от последнего такта до LATCH"),
    "BLK Hold":     (10.00, "длительность гашения"),
}


def verdict(name, measured):
    lo = DS[name][0]
    if measured is None:
        return "%-13s | %-9s | норма >= %-7s | канал не подключён" % (name, "-", "%g мкс" % lo)
    ok = "OK" if measured >= lo else "НАРУШЕНО"
    return "%-13s | %8.3f | норма >= %-8s | %s" % (name, measured, "%g мкс" % lo, ok)


def widths(bits, level):
    d = np.diff(bits.astype(np.int8))
    up = np.where(d > 0)[0] + 1
    dn = np.where(d < 0)[0] + 1
    if up.size < 2 or dn.size < 2:
        return None, None, None
    if level:
        a, b = up, dn
    else:
        a, b = dn, up
    if b[0] < a[0]:
        b = b[1:]
    n = min(a.size, b.size)
    w = (b[:n] - a[:n]) * US
    return w.min(), w.mean(), up


def main():
    subprocess.run([CLI, "--driver", "fx2lafw", "--config", "samplerate=%d" % RATE,
                    "--samples", "1200000", "-O", "binary", "-o", BIN],
                   capture_output=True, timeout=180)
    d = np.fromfile(BIN, dtype=np.uint8)
    if d.size == 0:
        raise SystemExit("захват пуст")
    sin = (d >> 0) & 1
    clk = (d >> 1) & 1
    lat = (d >> 2) & 1
    blk = (d >> 3) & 1
    print("захват %.1f мс, разрешение %.1f нс\n" % (d.size * US / 1000, US * 1000))

    rise = np.where(np.diff(clk.astype(np.int8)) > 0)[0] + 1
    per = np.diff(rise) * US
    inner = per[per < 5 * np.median(per)]          # без межслотовых пауз
    hi_min, hi_mean, _ = widths(clk, 1)
    lo_min, lo_mean, _ = widths(clk, 0)

    # setup/hold: расстояние от фронта CLK до ближайшего изменения SIN
    strans = np.where(np.diff(sin.astype(np.int8)) != 0)[0] + 1
    setup = hold = None
    if strans.size:
        idx = np.searchsorted(strans, rise)
        prev = np.where(idx > 0, strans[np.clip(idx - 1, 0, None)], -1)
        nxt = np.where(idx < strans.size, strans[np.clip(idx, None, strans.size - 1)], -1)
        s = (rise - prev)[prev >= 0] * US
        h = (nxt - rise)[nxt >= 0] * US
        setup, hold = float(s.min()), float(h.min())

    print("=== такт и данные ===")
    print(verdict("CLK Cycle", float(inner.min())))
    print(verdict("CLK High", float(hi_min)))
    print(verdict("CLK Low", float(lo_min)))
    print(verdict("SIN Setup", setup))
    print(verdict("SIN Hold", hold))

    # Минимумы выше загрязнены выбросами наводки, которые сидят прямо на фронтах CLK.
    # Считаем их отдельно и смотрим setup/hold по «чистым» переходам данных.
    if strans.size:
        near = np.zeros(strans.size, dtype=bool)
        for r in rise:
            near |= np.abs(strans - r) * US < 0.10          # ближе 100 нс к фронту такта
        gl = int(near.sum())
        clean = strans[~near]
        print("\n=== разделение: наводка отдельно от формирования ===")
        print("переходов SIN всего %d, из них в пределах 100 нс от фронта CLK: %d (выбросы)"
              % (strans.size, gl))
        if clean.size:
            idx = np.searchsorted(clean, rise)
            prev = clean[np.clip(idx - 1, 0, clean.size - 1)]
            nxt = clean[np.clip(idx, 0, clean.size - 1)]
            s = (rise - prev)[(rise - prev) > 0] * US
            h = (nxt - rise)[(nxt - rise) > 0] * US
            if s.size and h.size:
                print("по чистым переходам: setup мин %.3f мкс, hold мин %.3f мкс"
                      % (s.min(), h.min()))

    lat_min = widths(lat, 1)[0] if lat.any() else None
    blk_min = widths(blk, 1)[0] if blk.any() else None

    # CLK then LAT: от последнего спада такта в посылке до фронта LATCH
    clk_lat = None
    if lat.any():
        lat_up = np.where(np.diff(lat.astype(np.int8)) > 0)[0] + 1
        clk_dn = np.where(np.diff(clk.astype(np.int8)) < 0)[0] + 1
        if lat_up.size and clk_dn.size:
            j = np.searchsorted(clk_dn, lat_up) - 1
            ok = j >= 0
            if ok.any():
                dt = (lat_up[ok] - clk_dn[j[ok]]) * US
                clk_lat = float(dt.min())

    print("\n=== защёлка и гашение ===")
    print(verdict("LAT High", lat_min))
    print(verdict("CLK then LAT", clk_lat))
    print(verdict("BLK Hold", blk_min))

    print("\n=== структура кадра (даташит: слот ~100 мкс, кадр < 10 мс) ===")
    gaps = np.where(per > max(10 * np.median(inner), 20))[0]
    bursts = np.split(rise, gaps + 1)
    lens = [b.size for b in bursts if b.size > 5]
    if lens:
        starts = [b[0] for b in bursts if b.size > 5]
        slot = np.diff(starts) * US if len(starts) > 1 else np.array([0.0])
        print("тактов в посылке: медиана %d (даташит 240)" % int(np.median(lens)))
        print("период слота: %.0f мкс (даташит ~100 мкс)" % slot.mean())
        print("кадр из 44 слотов: %.1f мс -> %.1f Гц (даташит < 10 мс, > 100 Гц)"
              % (slot.mean() * 44 / 1000, 1e6 / (slot.mean() * 44)))


if __name__ == "__main__":
    main()
