#!/usr/bin/env python3
"""
generate_test_csv.py
生成測試軌跡 CSV 檔案，供 Motor-CAN-Bus-monitor 測試使用。
"""
import csv
import math

def generate_trajectory_csv(filename="test_trajectory.csv", duration=10.0, dt=0.01):
    """生成 100Hz 採樣率的馬達測試軌跡 (含時間、位置、速度)"""
    num_points = int(duration / dt)
    data = []

    for i in range(num_points):
        t = i * dt
        
        # 多階段運動 Profile
        if t < 2.0:
            # 階段 0~2s: 5 次多項式 (Quintic) 平滑從 0 rad 升至 0.8 rad
            s = t / 2.0
            pos = 0.8 * (10 * (s**3) - 15 * (s**4) + 6 * (s**5))
            vel = 0.8 * (30 * (s**2) - 60 * (s**3) + 30 * (s**4)) / 2.0
        elif t < 7.0:
            # 階段 2~7s: 0.8 rad 基準點加上 1 Hz 複合正弦擺動 (振幅 0.4 rad)
            tau = t - 2.0
            pos = 0.8 + 0.4 * math.sin(2 * math.pi * 1.0 * tau)
            vel = 0.4 * (2 * math.pi * 1.0) * math.cos(2 * math.pi * 1.0 * tau)
        else:
            # 階段 7~10s: 從當前位置平滑減速歸零 (0 rad)
            s = (t - 7.0) / 3.0
            # 歸零起點 (t=7.0 時正弦波正好在 0.8 rad，速度為 +2.51 rad/s)
            p_start = 0.8
            pos = p_start * (1 - (10 * (s**3) - 15 * (s**4) + 6 * (s**5)))
            vel = -p_start * (30 * (s**2) - 60 * (s**3) + 30 * (s**4)) / 3.0

        data.append((round(t, 3), round(pos, 4), round(vel, 4)))

    with open(filename, mode="w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["time", "pos", "vel"])
        writer.writerows(data)

    print(f"[CSV Generator] 成功生成測試軌跡檔: {filename} (共 {num_points} 點)")

if __name__ == "__main__":
    generate_trajectory_csv()