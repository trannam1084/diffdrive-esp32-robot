#!/usr/bin/env python3
"""Đánh giá một lần chạy (CSV tải từ http://192.168.4.1/log) để tune PID.

Cột: ms, state, mục tiêu R, đo R, mục tiêu L, đo L, PWM R, PWM L, yaw(độ)
Cách dùng: python tools/analyze_run.py run.csv

Với từng bánh in ra: mục tiêu cuối, vận tốc trung bình ở 30% cuối, sai số xác lập,
độ dao động, vọt lố và thời gian tăng. Thời gian tăng tính từ lúc mục tiêu bắt đầu
khác 0 nên đã gồm cả giai đoạn ramp tăng tốc.
Các gợi ý chỉ là quy tắc thô để bạn quyết định hướng chỉnh, không thay cho việc xem đồ thị.
"""
import csv
import statistics as st
import sys


def load(path):
    rows = []
    with open(path, encoding="utf-8", errors="ignore") as f:
        for r in csv.reader(f):
            if len(r) != 9:
                continue
            try:
                rows.append([float(x) for x in r])
            except ValueError:
                continue          # dòng tiêu đề hoặc dòng log khác
    return rows


def analyze(name, t, tgt, v):
    active = [i for i, x in enumerate(tgt) if abs(x) > 1e-3]
    if len(active) < 10:
        print(f"{name}: không có đoạn chạy (mục tiêu luôn bằng 0), bỏ qua.")
        return
    i0, i1 = active[0], active[-1]
    n = i1 - i0
    final = st.median(tgt[i0 + n * 6 // 10: i1 + 1])
    if abs(final) < 0.02:
        print(f"{name}: mục tiêu cuối quá nhỏ ({final:.3f} m/s), khó đánh giá.")
        return
    tail = v[i0 + n * 7 // 10: i1 + 1]
    mean, sd = st.mean(tail), st.pstdev(tail)
    err = mean - final
    seg = v[i0: i1 + 1]
    peak = max(seg) if final > 0 else min(seg)
    overshoot = max(0.0, (peak - final) / final * 100.0)

    def reach(frac):
        for i in range(i0, i1 + 1):
            if v[i] / final >= frac:
                return t[i] - t[i0]
        return None

    t10, t90 = reach(0.1), reach(0.9)
    print(f"\n== {name} ==")
    print(f"mục tiêu cuối      : {final:+.3f} m/s")
    print(f"trung bình 30% cuối: {mean:+.3f} m/s   sai số xác lập: {err:+.3f} m/s ({err / final * 100:+.1f}%)")
    print(f"độ dao động (độ lệch chuẩn): {sd:.3f} m/s ({sd / abs(final) * 100:.1f}% mục tiêu)")
    print(f"vọt lố             : {overshoot:.1f}%")
    print("thời gian tới 90%   : " + (f"{t90:.2f} s kể từ lúc bắt đầu" if t90 is not None else "không đạt 90% mục tiêu"))

    hints = []
    if t90 is None:
        hints.append("không đạt mục tiêu: kiểm tra PWM có chạm 255 không (cột pwm), hoặc tăng Ki/chỉnh feedforward")
    elif abs(err / final) > 0.05:
        hints.append("sai số xác lập lớn: tăng Ki hoặc đo lại feedforward (tools/fit_feedforward.py)")
    if overshoot > 15:
        hints.append("vọt lố cao: giảm Kp hoặc Ki")
    if sd / abs(final) > 0.08:
        hints.append("dao động nhiều: giảm Kp (hoặc kiểm tra dead-zone, xem docs/tuning.md)")
    for h in hints:
        print("  -> " + h)
    if not hints:
        print("  -> không thấy vấn đề rõ ràng theo các tiêu chí thô")


def main():
    if len(sys.argv) != 2:
        sys.exit("Cách dùng: python tools/analyze_run.py <file csv>")
    rows = load(sys.argv[1])
    if not rows:
        sys.exit("Không tìm thấy dòng CSV hợp lệ trong file.")
    t0 = rows[0][0]
    t = [(r[0] - t0) / 1000.0 for r in rows]
    print(f"{len(rows)} mẫu, {t[-1]:.1f} s")
    analyze("Bánh phải", t, [r[2] for r in rows], [r[3] for r in rows])
    analyze("Bánh trái", t, [r[4] for r in rows], [r[5] for r in rows])


if __name__ == "__main__":
    main()
