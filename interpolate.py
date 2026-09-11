import numpy as np
import pandas as pd
from scipy.interpolate import PchipInterpolator
from scipy.signal import butter, filtfilt

def interpolate_trajectory(input_csv="FL_Knee_test_trajectory.csv", output_csv="FL_Knee_test_trajectory_1kHz_smooth.csv"):
    # 1. 讀取原始數據
    df = pd.read_csv(input_csv)
    t_orig = df['time'].values
    pos_orig = df['pos'].values
    vel_orig = df['vel'].values
    torque_orig = df['torque'].values

    # 2. 建立 1kHz 控制時間軸 (dt = 0.001s)
    t_new = np.arange(t_orig[0], t_orig[-1], 0.001)

    # 3. 使用 PCHIP 保持單調性並進行平滑插值
    pos_interp = PchipInterpolator(t_orig, pos_orig)(t_new)
    vel_interp = PchipInterpolator(t_orig, vel_orig)(t_new)
    torque_interp = PchipInterpolator(t_orig, torque_orig)(t_new)

    # 4. 施加 50Hz 低通濾波器消除高頻雜訊
    fs, cutoff = 1000.0, 50.0
    b, a = butter(2, cutoff / (0.5 * fs), btype='low')
    
    pos_smooth = filtfilt(b, a, pos_interp)
    vel_smooth = filtfilt(b, a, vel_interp)
    torque_smooth = filtfilt(b, a, torque_interp)

    # 5. 匯出至 CSV
    df_out = pd.DataFrame({
        'time': np.round(t_new, 3),
        'pos': np.round(pos_smooth, 5),
        'vel': np.round(vel_smooth, 5),
        'torque': np.round(torque_smooth, 5)
    })
    df_out.to_csv(output_csv, index=False)
    print(f"[Done] 已生成 1kHz 平滑軌跡檔: {output_csv} (共 {len(df_out)} 筆數據)")

if __name__ == "__main__":
    interpolate_trajectory()