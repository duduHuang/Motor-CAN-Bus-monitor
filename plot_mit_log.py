import sys
import os
import pandas as pd
import matplotlib.pyplot as plt

def plot_single_mit_log(csv_file: str):
    """讀取單一 CSV 檔案並繪製 MIT 控制跟隨圖表"""
    if not os.path.exists(csv_file):
        print(f"[ERROR] 找不到檔案: {csv_file}")
        return

    df = pd.read_csv(csv_file)

    # 相容新舊 CSV 欄位名稱 (time_s / timestamp, torque_act / torque)
    time_col = 'time_s' if 'time_s' in df.columns else 'timestamp'
    torque_col = 'torque_act' if 'torque_act' in df.columns else 'torque'

    df['time'] = df[time_col] - df[time_col].iloc[0]

    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

    # 取得檔名主體 (做為圖表標題與輸出的 PNG 檔名)
    base_name = os.path.splitext(os.path.basename(csv_file))[0]

    # 1. 位置跟隨曲線
    axs[0].plot(df['time'], df['p_des'], 'r--', label='p_des (Desired)')
    axs[0].plot(df['time'], df['p_act'], 'b-', label='p_act (Actual)')
    axs[0].set_ylabel('Position (rad)')
    axs[0].legend(loc='upper right')
    axs[0].grid(True)
    axs[0].set_title(f'MIT Control Tracking Performance ({base_name})')

    # 2. 速度跟隨曲線
    axs[1].plot(df['time'], df['v_des'], 'r--', label='v_des (Desired)')
    axs[1].plot(df['time'], df['v_act'], 'g-', label='v_act (Actual)')
    axs[1].set_ylabel('Velocity (rad/s)')
    axs[1].legend(loc='upper right')
    axs[1].grid(True)

    # 3. 輸出力矩曲線
    axs[2].plot(df['time'], df[torque_col], 'm-', label='Torque (Actual)')
    axs[2].set_ylabel('Torque (Nm)')
    axs[2].set_xlabel('Time (s)')
    axs[2].legend(loc='upper right')
    axs[2].grid(True)

    plt.tight_layout()

    # 自動動態命名 PNG 檔案 (例如 m1_hardware_tracking_log_tracking_plot.png)
    out_png = f"{base_name}_tracking_plot.png"
    plt.savefig(out_png, dpi=300)
    print(f"[Plotter] 繪製完成：{csv_file} -> {out_png}")
    plt.close(fig)

def main():
    csv_files = sys.argv[1:]

    # 驗證輸入參數個數 (支援 1 ~ 3 個檔案)
    if not (1 <= len(csv_files) <= 3):
        print("\n[使用錯誤] 請提供 1 ~ 3 個 CSV 數據檔案！")
        print("指令範例:")
        print("  1 個檔案: python plot_mit_log.py m1.csv")
        print("  2 個檔案: python plot_mit_log.py m1.csv m2.csv")
        print("  3 個檔案: python plot_mit_log.py m1.csv m2.csv m3.csv\n")
        return

    # 依序繪製每個 CSV 檔的分析圖表
    for csv_file in csv_files:
        plot_single_mit_log(csv_file)

if __name__ == "__main__":
    main()