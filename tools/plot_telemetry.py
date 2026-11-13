#!/usr/bin/env python3
"""Vẽ log CSV từ bản build `telemetry`.

Cột: ms, state, mục tiêu R, đo R, mục tiêu L, đo L, PWM R, PWM L, yaw(độ)
Cách dùng: python tools/plot_telemetry.py logs/run1.log
Các dòng không phải CSV (log khởi động, thông báo) được bỏ qua tự động.
"""
import sys

import matplotlib.pyplot as plt


def read_rows(path):
    rows = []
    with open(path, encoding="utf-8", errors="ignore") as f:
        for line in f:
            parts = line.strip().split(",")
            if len(parts) != 9:
                continue
            try:
                rows.append([float(x) for x in parts])
            except ValueError:
                continue
    return rows


def main():
    if len(sys.argv) != 2:
        sys.exit("Cách dùng: python tools/plot_telemetry.py <file log>")
    rows = read_rows(sys.argv[1])
    if not rows:
        sys.exit("Không tìm thấy dòng CSV hợp lệ trong file.")

    t0 = rows[0][0]
    t = [(r[0] - t0) / 1000.0 for r in rows]
    col = lambda i: [r[i] for r in rows]

    fig, ax = plt.subplots(3, 1, sharex=True, figsize=(9, 8))
    ax[0].plot(t, col(2), "--", label="mục tiêu R")
    ax[0].plot(t, col(3), label="đo R")
    ax[0].plot(t, col(4), "--", label="mục tiêu L")
    ax[0].plot(t, col(5), label="đo L")
    ax[0].set_ylabel("vận tốc bánh (m/s)")
    ax[0].legend(ncol=4)

    ax[1].plot(t, col(6), label="PWM R")
    ax[1].plot(t, col(7), label="PWM L")
    ax[1].set_ylabel("PWM")
    ax[1].legend(ncol=2)

    ax[2].plot(t, col(8))
    ax[2].set_ylabel("yaw (độ)")
    ax[2].set_xlabel("thời gian (s)")

    for a in ax:
        a.grid(True, alpha=0.3)
    fig.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
