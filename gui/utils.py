# gui/utils.py
"""
Utility functions for Motor CAN Bus Monitor GUI
"""
import subprocess
import re
import glob
import ctypes
from ctypes import c_void_p, c_int, c_char_p
import dearpygui.dearpygui as dpg

def get_screen_resolution() -> tuple[int, int]:
    """取得螢幕解析度 (x86_64 / ARM64) (X11 / Wayland)"""
    # 1. Linux DRM (ARM64 Wayland/X11)
    try:
        for mode_file in glob.glob('/sys/class/drm/card*-*/modes'):
            with open(mode_file, 'r') as f:
                line = f.readline().strip()
                if 'x' in line:
                    w, h = map(int, line.split('x')[:2])
                    if w > 0 and h > 0:
                        return w, h
    except Exception:
        pass

    # 2. subprocess xrandr
    try:
        output = subprocess.check_output(['xrandr'], stderr = subprocess.DEVNULL).decode('utf-8')
        match = re.search(r'current\s+(\d+)\s+x\s+(\d+)', output)
        if match:
            return int(match.group(1)), int(match.group(2))
    except Exception:
        pass

    # 3. X11
    try:
        x11 = ctypes.cdll.LoadLibrary('libX11.so.6')
        x11.XOpenDisplay.restype = c_void_p
        x11.XOpenDisplay.argtypes = [c_char_p]
        display = x11.XOpenDisplay(None)
        if display:
            x11.XDefaultScreenOfDisplay.restype = c_void_p
            x11.XDefaultScreenOfDisplay.argtypes = [c_void_p]
            screen = x11.XDefaultScreenOfDisplay(display)
            if screen:
                x11.XWidthOfScreen.restype = c_int
                x11.XWidthOfScreen.argtypes = [c_void_p]
                x11.XHeightOfScreen.restype = c_int
                x11.XHeightOfScreen.argtypes = [c_void_p]
                w = x11.XWidthOfScreen(screen)
                h = x11.XHeightOfScreen(screen)
                x11.XCloseDisplay.argtypes = [c_void_p]
                x11.XCloseDisplay(display)
                if w > 0 and h > 0:
                    return w, h
    except Exception:
        pass

    # 4. Fallback 預設值
    return 1280, 800


def center_modal(modal_tag: int, modal_w: int, modal_h: int) -> None:
    """計算主視窗 Client 區域並將指定 Modal 彈窗水平垂直置中"""
    vp_w = dpg.get_viewport_client_width()
    vp_h = dpg.get_viewport_client_height()
    pos_x = max(0, (vp_w - modal_w) // 2)
    pos_y = max(0, (vp_h - modal_h) // 2)
    dpg.set_item_pos(modal_tag, [pos_x, pos_y])