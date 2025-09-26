#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
servo_control.py — 控制 ESP32 舵机（通过你暴露的HTTP接口）

依赖:
  pip install requests

用法示例:
  # 设定角度到 120°
  python servo_control.py --host 192.168.1.88 set 120

  # 扫角（30~150，步进2°，延时22ms）
  python servo_control.py --host 192.168.1.88 sweep --min 30 --max 150 --step 2 --delay 22

  # 点动（每次+/-5度）
  python servo_control.py --host 192.168.1.88 jog +5
  python servo_control.py --host 192.168.1.88 jog -10

  # 执行一串角度（文件或命令行列表）
  python servo_control.py --host 192.168.1.88 runpath 30 60 90 120 150 120 90 60 30 --delay 40
  python servo_control.py --host 192.168.1.88 runpath --file angles.txt --delay 30

  # 启动简易 GUI 滑块（需要 tkinter）
  python servo_control.py --host 192.168.1.88 gui
"""

import argparse
import time
import sys
from typing import List, Optional
import requests

# 默认设备地址（可用 --host 覆盖）
DEVICE_HOST = "10.201.171.123"
# 舵机控制服务端口（与固件一致）
SERVO_PORT = 8080
# 角度安全范围
ANGLE_MIN = 0
ANGLE_MAX = 180
# HTTP 超时
HTTP_TIMEOUT = 3.0


def clamp(angle: int) -> int:
    return max(ANGLE_MIN, min(ANGLE_MAX, int(angle)))


def endpoint(host: str, path: str) -> str:
    return f"http://{host}:{SERVO_PORT}{path}"


def set_angle(host: str, angle: int) -> dict:
    angle = clamp(angle)
    url = endpoint(host, f"/servo?angle={angle}")
    r = requests.get(url, timeout=HTTP_TIMEOUT)
    r.raise_for_status()
    return r.json()


def sweep(host: str, a_min: int, a_max: int, step: int, delay_ms: int) -> dict:
    a_min = clamp(a_min)
    a_max = clamp(a_max)
    if a_min > a_max:
        a_min, a_max = a_max, a_min
    step = max(1, int(step))
    delay_ms = max(1, int(delay_ms))
    url = endpoint(host, f"/servo/sweep?min={a_min}&max={a_max}&step={step}&delay={delay_ms}")
    r = requests.get(url, timeout=max(HTTP_TIMEOUT, delay_ms/1000.0*2))
    r.raise_for_status()
    return r.json()

def sweep_forever(host: str, a_min: int, a_max: int, step: int, delay_ms: int):
    """连续扫描角度范围"""
    while True:  # 永久循环，直到程序被手动停止
        res = sweep(host, a_min, a_max, step, delay_ms)
        print(res)
        time.sleep(1)  # 每次扫描完成后停留1秒

def run_path(host: str, angles: List[int], delay_ms: int):
    delay_ms = max(1, int(delay_ms))
    for a in angles:
        a = clamp(a)
        try:
            res = set_angle(host, a)
        except Exception as e:
            print(f"[ERR] set_angle({a}) failed: {e}", file=sys.stderr)
            break
        else:
            print(f"[OK] -> {res}")
        time.sleep(delay_ms / 1000.0)


def parse_angles_file(path: str) -> List[int]:
    seq = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            # 允许以逗号/空格分隔
            parts = [p for chunk in line.split(",") for p in chunk.split()]
            for p in parts:
                try:
                    seq.append(int(p))
                except ValueError:
                    raise ValueError(f"Invalid angle value in file: {p}")
    return seq


def main():
    parser = argparse.ArgumentParser(description="ESP32 Servo HTTP Controller")
    parser.add_argument("--host", default=DEVICE_HOST, help=f"设备IP（默认 {DEVICE_HOST}）")
    sub = parser.add_subparsers(dest="cmd", required=True)

    # set
    p_set = sub.add_parser("set", help="设定角度")
    p_set.add_argument("angle", type=int, help="角度（0~180）")

    # sweep
    p_sweep = sub.add_parser("sweep", help="扫角一次往返")
    p_sweep.add_argument("--min", type=int, default=30, dest="amin", help="最小角度")
    p_sweep.add_argument("--max", type=int, default=150, dest="amax", help="最大角度")
    p_sweep.add_argument("--step", type=int, default=2, help="步进角度(°)")
    p_sweep.add_argument("--delay", type=int, default=22, help="每步延时(ms)")

    # sweep_forever
    p_sweep = sub.add_parser("sweep_forever", help="扫角循环")
    p_sweep.add_argument("--min", type=int, default=30, dest="amin", help="最小角度")
    p_sweep.add_argument("--max", type=int, default=150, dest="amax", help="最大角度")
    p_sweep.add_argument("--step", type=int, default=2, help="步进角度(°)")
    p_sweep.add_argument("--delay", type=int, default=22, help="每步延时(ms)")

    # jog
    p_jog = sub.add_parser("jog", help="点动（相对当前角度的增量；脚本不记忆当前角度，按目标角度自行管理）")
    p_jog.add_argument("delta", type=int, help="增量角度（如 +5 或 -5）")
    p_jog.add_argument("--base", type=int, default=None, help="基准角度（若提供则=基准+delta）")

    # runpath
    p_path = sub.add_parser("runpath", help="依次执行一串角度")
    p_path.add_argument("angles", nargs="*", type=int, help="角度序列（空格分隔）")
    p_path.add_argument("--file", type=str, help="从文件读取角度序列（可逗号或空格分隔，支持注释/空行）")
    p_path.add_argument("--delay", type=int, default=30, help="两角度间延时(ms)")

    # gui
    p_gui = sub.add_parser("gui", help="简易 GUI 滑块控制（需要 tkinter）")

    args = parser.parse_args()
    host = args.host

    try:
        if args.cmd == "set":
            res = set_angle(host, args.angle)
            print(res)

        elif args.cmd == "sweep":
            res = sweep(host, args.amin, args.amax, args.step, args.delay)
            print(res)

        elif args.cmd == "jog":
            # 简化处理：若未提供 base，则提示使用者先用 set 设定一个基准
            if args.base is None:
                print("[TIP] 未提供 --base，默认以 90° 为基准。可用 --base 指定。")
                base = 90
            else:
                base = args.base
            target = clamp(base + args.delta)
            res = set_angle(host, target)
            print(res)

        elif args.cmd == "sweep_forever":
            res = sweep_forever(host, args.amin, args.amax, args.step, args.delay)
            print(res)

        elif args.cmd == "runpath":
            seq: List[int] = []
            if args.file:
                seq = parse_angles_file(args.file)
            if args.angles:
                seq.extend(args.angles)
            if not seq:
                print("角度序列为空。请提供参数，如：runpath 30 60 90 或 --file angles.txt")
                sys.exit(1)
            run_path(host, seq, args.delay)

        elif args.cmd == "gui":
            try:
                import tkinter as tk
            except Exception as e:
                print("需要 tkinter：请在桌面环境下运行，或安装相应依赖。", file=sys.stderr)
                sys.exit(1)

            root = tk.Tk()
            root.title(f"Servo Controller @ {host}:{SERVO_PORT}")

            current_var = tk.IntVar(value=90)

            def on_set(val=None):
                a = clamp(current_var.get())
                try:
                    set_angle(host, a)
                except Exception as e:
                    status_var.set(f"错误: {e}")
                else:
                    status_var.set(f"已设定角度: {a}°")

            def on_jog(delta):
                a = clamp(current_var.get() + delta)
                current_var.set(a)
                on_set()

            frm = tk.Frame(root, padx=12, pady=12)
            frm.pack(fill="both", expand=True)

            slider = tk.Scale(frm, from_=ANGLE_MIN, to=ANGLE_MAX, orient="horizontal",
                              variable=current_var, label="角度", length=360, command=lambda v: None)
            slider.pack(fill="x")

            btns = tk.Frame(frm)
            btns.pack(pady=8)
            tk.Button(btns, text="-10°", width=8, command=lambda: on_jog(-10)).pack(side="left", padx=4)
            tk.Button(btns, text="-1°",  width=8, command=lambda: on_jog(-1)).pack(side="left", padx=4)
            tk.Button(btns, text="设定",  width=8, command=on_set).pack(side="left", padx=4)
            tk.Button(btns, text="+1°",  width=8, command=lambda: on_jog(+1)).pack(side="left", padx=4)
            tk.Button(btns, text="+10°", width=8, command=lambda: on_jog(+10)).pack(side="left", padx=4)

            status_var = tk.StringVar(value="准备就绪")
            tk.Label(frm, textvariable=status_var, anchor="w").pack(fill="x")

            # 初始设置一次
            on_set()
            root.mainloop()

    except requests.exceptions.RequestException as e:
        print(f"[HTTP ERROR] {e}\n"
              f"请确认设备已联网、IP/端口正确（默认端口 {SERVO_PORT}），以及固件已启用 /servo 接口。",
              file=sys.stderr)
        sys.exit(2)


if __name__ == "__main__":
    main()