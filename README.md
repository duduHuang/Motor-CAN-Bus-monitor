# Motor-CAN-Bus-monitor

# Quadruped Motor CAN Bus Monitor & Control System

![Python Version](https://img.shields.io/badge/Python-3.10%2B-blue.svg)
![Architecture](https://img.shields.io/badge/Architecture-MVVM-green.svg)
![Protocol](https://img.shields.io/badge/Bus-SocketCAN-orange.svg)
![UI Framework](https://img.shields.io/badge/UI-DearPyGui-red.svg)

本專案為針對四足機器人（Quadruped Robot）開發的多軸 CAN Bus 馬達驅動控制、姿態動態驗證與實時遙測監控系統。系統採用 **MVVM (Model-View-ViewModel)** 架構，具備高精度的時間補償控制迴圈、動態濾波與多重連鎖安全保護機制。

---

## 核心功能 (Key Features)
* **12 軸雙向姿態狀態機 (FSM Stance Control)**
  * 支援趴姿（Prone）、低蹲（Crouch）與高剛度站立（Stand）姿態切換。
  * 內建餘弦 S 曲線（S-Curve）軌跡插值，確保關節無衝擊運作與站立防腿軟前饋補償。
* **對角步態踏步測試 (Trot Gait Test)**
  * 支援四足對角動態抬腳與原地踏步驗證。
  * 提供獨立關節抬腿比例（Lift Ratio）與擺動相/支撐相剛度切換。
* **即時高幀率遙測 GUI (MVVM Real-time Monitor)**
  * 基於 **DearPyGui** 打造的多軸視覺化控制面板。
  * 提供角度、轉速、電流、溫度、電壓五大狀態看板，並整合三合一跟隨波形圖（Position, Velocity, Torque）與 Raw CAN Console Log。
* **工業級安全防護與 Cascade E-STOP**
  * 支援全聲道跨軸連鎖急停（Cascade E-STOP）：任一關節發生過速、過載、跟隨誤差過大或通訊超時，即刻同步切斷全車馬達使能。
  * 具備 CAN 電磁波離群值雜訊過濾（Outlier Spike Filter）與多重 Debounce 防誤觸機制。
* **解耦式硬體校正配置 (Decoupling Configuration)**
  * 關節極限角度與零點校正參數獨立於程式碼之外，採用外置配置文件動態載入。

---

## 專案現況與進度 (Current Status)
目前專案已完成**單機/多軸控制驗證、核心效能重構與 MVVM 架構定型**：

| 測試驗證功能 | 腳本 / 模組 | 發展狀態 | 說明 |
| :--- | :--- | :---: | :--- |
| **姿態 FSM 驗證** | `run_12motor_fsm.py` | ✅ 已完成 | 驗證趴下、蹲低、站立三大姿態切換與關節支撐剛度。 |
| **對角步態驗證** | `run_12motor_gait.py` | ✅ 已完成 | 驗證動態 Trot Gait 踏步跟隨與擺動相抬腿軌跡。 |
| **GUI 即時監控面板** | `main_gui.py` | ✅ 已完成 | 提供多軸參數微調、波形觀察與全域 Master 急停控制。 |
| **效能與架構優化** | `core/`, `trajectory/` | ✅ 已完成 | 實現高頻 CAN Logging 延遲格式化、自適應時間補償與參數解耦。 |

### 最近完成的重構與優化 (Recent Refactoring)
1. **高頻 CAN Logging 效能優化**：控制迴圈改儲存 Raw Data，改為 GUI 讀取時才進行「延遲格式化 (Lazy Formatting)」，消除高頻字串處理帶來的 CPU 負擔。
2. **ViewModel 封裝性提升**：消除 View 直接對 ViewModel 私有成員（`_current_provider`）的存取，規範統一注入介面。
3. **校正參數解耦**：將硬編碼之 `MOTOR_CALIBRATION_DEG` 抽離至外置 Config 檔案，提升硬體維護彈性。
4. **控制迴圈時間補償**：採用 `time.monotonic()` 自適應休眠補償，顯著降低高頻控制時的時間抖動 (Jitter)。

---

## 系統架構 (Architecture)
本專案遵循嚴格的 **MVVM** 設計模式：

  [ View ]                           [ ViewModel ]                         [ Model ]
+-------------------+              +-----------------------+             +-------------------+
|   MotorControlView|              | MotorControlViewModel |             |  MotorController  |
|  (DearPyGui Main) | <----------> | (Thread-Safe Snapshot)| <---------> |  (SocketCAN Bus)  |
|                   |  DataBind /  |                       |  Control /  |                   |
|   CLI Scripts     |  Observer    | MultiMotorManager     |  Telemetry  |   MotorRxWorker   |
| (FSM/Gait/Stand)  |              |                       |             | (Background Thread|
+-------------------+              +-----------------------+             +-------------------+
|
+-------------------+
|TrajectoryProvider |
|(Sine/CSV/FSM/Gait)|
+-------------------+

---

## 快速開始 (Quick Start)
### 1. 環境需求
* Linux (Ubuntu 20.04 / 22.04 推廣)
* Python 3.10+
* 已配置之 SocketCAN 介面 (`can0`, `can1` ~ `can4`)

### 2. 安裝套件
pip install -r requirements.txt

### 3. 執行指令範例
* **啟動多軸 GUI 視覺監控面板：**
# 實體硬體模式 (指定 CAN Channel 與 馬達 ID)
python main_gui.py --config "can1:1,can1:2,can1:3,can2:1" --bitrate 1000000

# 免硬體模擬測試 (Mock Mode)
python main_gui.py --mock --config "can1:1,can1:2,can1:3"

* **執行 12 軸姿態 FSM 控制驗證：**
python run_12motor_fsm.py

* **執行 12 軸對角步態踏步測試：**
python run_12motor_gait.py

---

## 未來開發規劃 (Roadmap)
目前的營運環境為**筆電 (Windows) --SSH--> 板子 A (Linux) --SSH--> 板子 B (Linux / SocketCAN)**。未來預計推進至 **Phase 4：分散式遠端控制架構**，將 GUI 控制介面與底層 CAN 控制實體進行分離。

[ 板子 A / 筆電 (上位機) ]                       [ 板子 B (CAN 控制節點 / 下位機) ]
+--------------------------+                   +----------------------------------+
|  Motor Control GUI       |                   |  Robot Motion Daemon             |
|  Remote ViewModel        | <== Network ====> |  (FSM / Gait / Motor Controller) |
|  (Telemetry Display)     |   (gRPC / ROS2)   |  SocketCAN Interface (12 Motors) |
+--------------------------+                   +----------------------------------+

### 關鍵開發階段計畫：
* [ ] **板子 B 下位機服務化 (Headless Daemon)**
* 將板子 B 的控制邏輯封裝為 Systemd 背景服務，開機自動執行底層安全監控與控制迴圈。

* [ ] **分散式通訊協定整合 (Transmission Protocol)**
* 評估並導入 **ROS 2 (DDS)** / **gRPC** / **ZeroMQ** 技術，實現跨節點雙向串流通訊。

* [ ] **網路超時與 Heartbeat 安全守護 (Watchdog System)**
* 新增網路連線 Watchdog，當筆電/板子 A 與板子 B 斷線超過 300ms 時，板子 B 本地自動觸發安全平滑降落或 E-STOP。

* [ ] **遙測數據動態降頻 (Telemetry Downsampling)**
* 板子 B 本地保持 100Hz~500Hz 高頻控制，將網路遙測傳輸壓縮至 30Hz~60Hz，優化無線頻寬利用率。

---

## 專案目錄結構 (Project Structure)
├── core/                        # MVVM 核心控制層 (Controller, RxWorker, ViewModel, Manager)
├── gui/                         # DearPyGui 視覺化元件與主視圖
│   └── components/              # Dashboard, Panels, Waveform Plots, Raw CAN Console
├── protocol/                    # CAN Bus 協定壓碼與解碼器 (MIT Mode & Servocmd)
│   └── decoders/                # 各類 Sensor, Motion, Query 封包解碼器
├── trajectory/                  # 軌跡產生器 (Sine, CSV, Rosbag DB3, FSM, Gait)
├── main_gui.py                  # GUI 主程式進入點
├── run_12motor_fsm.py           # 12 軸雙向姿態 FSM 測試腳本
├── run_12motor_gait.py          # 12 軸對角步態測試腳本
└── test/                        # Headless 自動化單元測試集與 Mock Engine
