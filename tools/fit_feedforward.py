#!/usr/bin/env python3
"""Tìm WHEEL_FF_OFFSET và WHEEL_FF_GAIN từ các lần chạy /pwm.

Mỗi file CSV là một lần chạy /pwm ở một mức PWM cố định (ví dụ 120, 160, 200, 255,
mỗi mức khoảng 2 đến 3 giây, đặt xe trên sàn như khi chạy thật). Script lấy nửa sau
của mỗi lần chạy (đã ổn định), tính PWM trung bình và vận tốc trung bình của từng bánh,
rồi khớp đường thẳng  PWM = OFFSET + GAIN * v.

Cách dùng: python tools/fit_feedforward.py pwm120.csv pwm160.csv pwm200.csv pwm255.csv
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
                continue
    return rows


def fit(points):
    """Bình phương tối thiểu: pwm = offset + gain * v. points = [(v, pwm), ...]"""
    n = len(points)
    mv = sum(p[0] for p in points) / n
    mp = sum(p[1] for p in points) / n
    sxx = sum((p[0] - mv) ** 2 for p in points)
    if sxx < 1e-9:
        return None
    gain = sum((p[0] - mv) * (p[1] - mp) for p in points) / sxx
    return mp - gain * mv, gain


def main():
    files = sys.argv[1:]
    if len(files) < 2:
        sys.exit("Cần ít nhất 2 file (3 đến 4 mức PWM cho kết quả tin cậy).")

    pts = {"phải": [], "trái": []}
    for path in files:
        rows = load(path)
        if len(rows) < 20:
            print(f"{path}: quá ít mẫu, bỏ qua.")
            continue
        half = rows[len(rows) // 2:]
        for side, ip, iv in (("phải", 6, 3), ("trái", 7, 5)):
            pwm = st.mean(r[ip] for r in half)
            v = st.mean(r[iv] for r in half)
            if abs(pwm) < 1 or abs(v) < 0.02:
                print(f"{path}: bánh {side} gần như không quay (PWM {pwm:.0f}, v {v:.3f}), bỏ điểm này.")
                continue
            pts[side].append((abs(v), abs(pwm)))
            print(f"{path}: bánh {side}  PWM {abs(pwm):5.0f}  v {abs(v):.3f} m/s")

    res = {}
    for side, p in pts.items():
        if len(p) < 2:
            print(f"Bánh {side}: không đủ điểm để khớp.")
            continue
        r = fit(p)
        if r is None:
            print(f"Bánh {side}: các điểm có cùng vận tốc, không khớp được.")
            continue
        res[side] = r
        print(f"\nBánh {side} ({len(p)} điểm): OFFSET = {r[0]:.1f}, GAIN = {r[1]:.1f}")
        if len(p) < 3:
            print("  (chỉ 2 điểm: nên đo thêm mức PWM nữa)")

    if len(res) == 2:
        o = (res["phải"][0] + res["trái"][0]) / 2
        g = (res["phải"][1] + res["trái"][1]) / 2
        print(f"\nTrung bình hai bánh: WHEEL_FF_OFFSET = {o:.1f}, WHEEL_FF_GAIN = {g:.1f}")
        gap = abs(res["phải"][1] - res["trái"][1]) / g
        if gap > 0.10:
            print(f"Chú ý: GAIN hai bánh lệch {gap * 100:.0f}% (hơn 10%). Nên tách hệ số theo từng bánh.")
        print("Ghi hai giá trị trên vào include/Config.h.")


if __name__ == "__main__":
    main()
