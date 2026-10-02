#!/usr/bin/env python3
"""
plot_load_test_log.py
12 軸四足負重測試 CSV 數據繪圖分析腳本
輸入：run_12motor_load_test.py 產生的 CSV 檔案
輸出：共 5 張 PNG 分析圖表 (4 張 CAN 通道詳細圖 + 1 張 4 通道綜合總覽圖)
"""

import sys
import os
import pandas as pd
import matplotlib.pyplot as plt

# CAN 通道與肢體名稱對應表 (英文)
LIMB_MAP = {
    'can1': 'Right Hind',
    'can2': 'Left Hind',
    'can3': 'Right Fore',
    'can4': 'Left Fore'
}

def plot_channel_detail(df: pd.DataFrame, channel: str, base_name: str):
    """繪製單一 CAN 通道 (ID 1, 2, 3 三顆馬達) 的 4 項物理量數據曲線圖"""
    ch_df = df[df['Channel'] == channel]
    if ch_df.empty:
        print(f"[WARN] 找不到通道 {channel} 的數據，跳過繪製單通道圖。")
        return

    limb_name = LIMB_MAP.get(channel.lower(), '')
    ch_label = f"{channel.upper()} ({limb_name})" if limb_name else channel.upper()

    # 建立 4x1 的子圖結構 (角度、扭矩、電流、溫度)
    fig, axs = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
    fig.suptitle(f'{ch_label} Motors Load Test Detail ({base_name})', fontsize=14, fontweight='bold')

    motors = [1, 2, 3]
    colors = {1: '#1f77b4', 2: '#ff7f0e', 3: '#2ca02c'}  # ID1: 藍, ID2: 橘, ID3: 綠

    # 1. 位置角度曲線 (Pos_deg)
    for m_id in motors:
        m_df = ch_df[ch_df['Motor_ID'] == m_id]
        axs[0].plot(m_df['Time_s'], m_df['Pos_deg'], label=f'Motor ID {m_id}', color=colors[m_id], linewidth=1.5)
    axs[0].set_ylabel('Position (deg)')
    axs[0].grid(True, linestyle='--', alpha=0.6)
    axs[0].legend(loc='upper right')

    # 2. 輸出力矩曲線 (Torque_Nm)
    for m_id in motors:
        m_df = ch_df[ch_df['Motor_ID'] == m_id]
        axs[1].plot(m_df['Time_s'], m_df['Torque_Nm'], label=f'Motor ID {m_id}', color=colors[m_id], linewidth=1.5)
    axs[1].set_ylabel('Torque (Nm)')
    axs[1].grid(True, linestyle='--', alpha=0.6)
    axs[1].legend(loc='upper right')

    # 3. 轉矩電流曲線 (Current_A)
    for m_id in motors:
        m_df = ch_df[ch_df['Motor_ID'] == m_id]
        axs[2].plot(m_df['Time_s'], m_df['Current_A'], label=f'Motor ID {m_id}', color=colors[m_id], linewidth=1.5)
    axs[2].set_ylabel('Current (A)')
    axs[2].grid(True, linestyle='--', alpha=0.6)
    axs[2].legend(loc='upper right')

    # 4. 馬達溫度曲線 (Temp_C)
    for m_id in motors:
        m_df = ch_df[ch_df['Motor_ID'] == m_id]
        axs[3].plot(m_df['Time_s'], m_df['Temp_C'], label=f'Motor ID {m_id}', color=colors[m_id], linewidth=1.5)
    axs[3].set_ylabel('Temp (°C)')
    axs[3].set_xlabel('Time (s)')
    axs[3].grid(True, linestyle='--', alpha=0.6)
    axs[3].legend(loc='upper right')

    plt.tight_layout()
    out_png = f"{base_name}_{channel}.png"
    plt.savefig(out_png, dpi=300)
    plt.close(fig)
    print(f"[Plotter] 通道詳細圖表已匯出：{out_png}")

def plot_summary_overview(df: pd.DataFrame, base_name: str):
    """繪製 4 個 CAN 通道 (2x2 矩陣) 的總覽對比圖"""
    channels = ['can1', 'can2', 'can3', 'can4']
    fig, axs = plt.subplots(2, 2, figsize=(14, 10), sharex=True)
    fig.suptitle(f'12-Axis Quadruped Load Test Overall Summary ({base_name})', fontsize=16, fontweight='bold')

    motors = [1, 2, 3]
    colors = {1: '#1f77b4', 2: '#ff7f0e', 3: '#2ca02c'}

    for idx, ch in enumerate(channels):
        row, col = divmod(idx, 2)
        ax = axs[row, col]
        ch_df = df[df['Channel'] == ch]

        limb_name = LIMB_MAP.get(ch.lower(), '')
        ch_label = f"{ch.upper()} ({limb_name})" if limb_name else ch.upper()

        if ch_df.empty:
            ax.set_title(f'{ch_label} (No Data)')
            continue

        # 每個通道繪製該腿 3 顆馬達的扭矩分佈
        for m_id in motors:
            m_df = ch_df[ch_df['Motor_ID'] == m_id]
            ax.plot(m_df['Time_s'], m_df['Torque_Nm'], label=f'M{m_id} Torque', color=colors[m_id], linewidth=1.5)

        ax.set_title(f'Channel: {ch_label}', fontsize=12, fontweight='bold')
        ax.set_ylabel('Torque (Nm)')
        ax.grid(True, linestyle='--', alpha=0.6)
        ax.legend(loc='upper right', fontsize='small')

        if row == 1:
            ax.set_xlabel('Time (s)')

    plt.tight_layout()
    out_png = f"{base_name}_summary_overview.png"
    plt.savefig(out_png, dpi=300)
    plt.close(fig)
    print(f"[Plotter] 4 通道總覽圖表已匯出：{out_png}")

def plot_load_test_log(csv_file: str):
    """主執行流程：驗證檔案並產生 5 張分析圖表"""
    if not os.path.exists(csv_file):
        print(f"[ERROR] 找不到指定的 CSV 檔案: {csv_file}")
        return

    df = pd.read_csv(csv_file)

    required_cols = {'Time_s', 'Channel', 'Motor_ID', 'Pos_deg', 'Torque_Nm', 'Temp_C', 'Current_A'}
    if not required_cols.issubset(df.columns):
        print(f"[ERROR] CSV 格式不符！缺少必要欄位: {required_cols - set(df.columns)}")
        return

    base_name = os.path.splitext(os.path.basename(csv_file))[0]

    # 1~4. 繪製 can1 ~ can4 各自的詳細圖表
    for ch in ['can1', 'can2', 'can3', 'can4']:
        plot_channel_detail(df, ch, base_name)

    # 5. 繪製 4 個 CAN 通道 2x2 對比總覽圖
    plot_summary_overview(df, base_name)

def main():
    if len(sys.argv) < 2:
        print("\n[使用錯誤] 請提供 CSV 數據檔案路徑！")
        print("使用指令範例:")
        print("  python plot_load_test_log.py rmd_x4_load_3kg_5min_20261002_150000.csv\n")
        return

    for csv_file in sys.argv[1:]:
        plot_load_test_log(csv_file)

if __name__ == "__main__":
    main()