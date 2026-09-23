# config_loader.py
import json
import os
from typing import Dict, Any

# 安全降級預設值
DEFAULT_CALIBRATION_DEG: Dict[str, Dict[int, Dict[str, Any]]] = {
    "can1": {
        1: {"min": -62.0,  "max": -10.0,  "prone": -36.0,  "crouch": -36.0,  "stand": -36.0},
        2: {"min": -380.0, "max": 160.0,  "prone": 10.0,   "crouch": 40.0,   "stand": 67.0},
        3: {"min": -107.0, "max": 25.0,   "prone": 21.0,  "crouch": -39.0,  "stand": -103.0},
    },
    "can2": {
        1: {"min": -130.0, "max": -77.0,  "prone": -103.5, "crouch": -103.5, "stand": -103.5},
        2: {"min": 29.0,   "max": 545.0,  "prone": 170.0,  "crouch": 145.0,  "stand": 117.0},
        3: {"min": -46.0,  "max": 85.5,   "prone": -42.0,  "crouch": 18.0,   "stand": 81.5},
    },
    "can3": {
        1: {"min": 72.0,   "max": 123.0,  "prone": 97.5,   "crouch": 97.5,   "stand": 97.5},
        2: {"min": -210.0, "max": 325.0,  "prone": 120.0,  "crouch": 152.0,  "stand": 182.0},
        3: {"min": -104.0, "max": 24.0,   "prone": 20.0,   "crouch": -40.0,  "stand": -100.0},
    },
    "can4": {
        1: {"min": -1.0,   "max": 48.0,   "prone": 23.5,   "crouch": 23.5,   "stand": 23.5},
        2: {"min": -140.0, "max": 370.0,  "prone": 50.0,   "crouch": 21.0,   "stand": -9.0},
        3: {"min": 16.0,   "max": 148.0,  "prone": 20.0,   "crouch": 80.0,   "stand": 144.0},
    }
}

class ConfigManager:
    """單例模式硬體校正配置載入器"""
    _instance = None
    _config: Dict[str, Dict[int, Dict[str, Any]]] = {}

    def __new__(cls, filepath: str = "robot_calibration.json"):
        if cls._instance is None:
            cls._instance = super(ConfigManager, cls).__new__(cls)

            # 若傳入相對路徑，自動計算相對於 config_loader.py 所在目錄（根目錄）的絕對路徑
            if not os.path.isabs(filepath):
                base_dir = os.path.dirname(os.path.abspath(__file__))
                filepath = os.path.join(base_dir, filepath)

            cls._instance._load_config(filepath)
        return cls._instance

    def _load_config(self, filepath: str) -> None:
        if os.path.exists(filepath):
            try:
                with open(filepath, "r", encoding="utf-8") as f:
                    raw_data = json.load(f)
                    parsed = {}
                    for ch, motors in raw_data.items():
                        parsed[ch.lower()] = {}
                        for m_id_str, cfg in motors.items():
                            parsed[ch.lower()][int(m_id_str)] = cfg
                    self._config = parsed
                    return
            except Exception as e:
                print(f"[ConfigManager] 載入配置文件 '{filepath}' 失敗: {e}，降級使用安全預設值。")
        else:
            print(f"[ConfigManager] 未找到配置文件 '{filepath}'，降級使用安全預設值。")

        self._config = DEFAULT_CALIBRATION_DEG

    def get_calibration(self) -> Dict[str, Dict[int, Dict[str, Any]]]:
        return self._config