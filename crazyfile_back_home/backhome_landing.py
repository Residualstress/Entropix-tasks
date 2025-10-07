import logging
import sys
import time
from threading import Event
import traceback

import numpy as np

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper

from uwb_localization.uwb_receiver import UWBReceiver
from uwb_localization.ukf_filter import PositionUKF
from pid_controller.PIDController import PIDController2D
from apriltag_beacon.apriltag_beacon import HttpAprilResolver

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

deck_attached_event = Event()

logging.basicConfig(level=logging.ERROR)

position_estimate   = [0.0, 0.0, 0.0]  # stateEstimate.{x,y,z}

DEFAULT_HEIGHT = 0.7

# 相机内参（像素焦距，建议替换为标定值）
FX = 320.1
FY = 320.1

# 图像中心（像素）
IMG_CX = 160
IMG_CY = 120

# 对齐阈值（等效米）+ 保持时间
BACKHOME_EPS_M_ENTER = 0.2   # 进入 BACKHOME 的误差阈值（米）
ALIGN_EPS_M_ENTER = 0.08   # 进入 TRACK_DESCEND 的误差阈值（米）
ALIGN_HOLD_SEC    = 0.80   # 进入前需连续满足的时间（秒）

# 丢失目标判定
MISS_TIMEOUT      = 1.5    # 超过该秒数未见目标，则视为丢失（悬停）

# EMA 平滑像素误差（抗抖）
PIX_EMA_ALPHA     = 0.8

# 同步下降阶段
DESCENT_RATE      = 0.03   # TRACK_DESCEND 阶段的目标竖直速度（m/s，向下）
XY_SPEED_LIMIT_LANDING    = 0.1   # 同步下降阶段的水平限速（m/s）
XY_SPEED_LIMIT_BACKHOME   = 0.2   # backhome 时的水平限速（m/s）

# 仅Z收尾阈值
Z_ONLY_THRESHOLD  = 0.25   # 进入 FINAL_DROP 的高度阈值（m）
FINAL_DROP_VZ_FAR = 0.15   # 0.15 m ~ 0.08 m 区间的竖直速度（m/s）
FINAL_DROP_VZ_NEAR= 0.10   # 0.08 m ~ 地面的竖直速度（m/s）
TOUCHDOWN_Z       = 0.03   # 触地判定/收尾触发（m）

# -----------------------------
# 全局状态
# -----------------------------
deck_attached_event = Event()
position_estimate   = [0.0, 0.0, 0.0]  # stateEstimate.{x,y,z}

last_seen_ts = None
pix_err_raw  = None        # (u, v) 像素误差（右正、上正）
pix_err_ema  = None

state = 'BACKHOME'         # 'BACKHOME' -> 'ALIGNING' -> 'TRACK_DESCEND' -> 'FINAL_DROP' -> 'LANDED'

# 计时器
_align_ok_since = None

def log_pos_callback(timestamp, data, logconf):
    position_estimate[0] = data['stateEstimate.x']
    position_estimate[1] = data['stateEstimate.y']
    position_estimate[2] = data['stateEstimate.z']


def param_deck_flow(_, value_str):
    value = int(value_str)
    print(value)
    if value:
        deck_attached_event.set()
        print('Deck is attached!')
    else:
        print('Deck is NOT attached!')
        

def pix_to_meters(du, dv, z):
    """像素误差 -> 等效米误差"""
    z = max(0.1, float(z))
    ex = z * (du / FX)
    ey = z * (dv / FY)
    return ex, ey


def back_home(mc: MotionCommander, pid_xy: PIDController2D):
    global state, _align_ok_since

    if not uwb.has_data():
        return
    
    while uwb.has_data():
        _,_, x, y = uwb.pop_data()

    vx, vy = pid_xy.compute_velocity(x, y)

    print(f'x: {x}, y: {y}, vel_x: {vx}, vel_y: {vy}')

    mc.start_linear_motion(vx, vy, 0)

    ex, ey = pid_xy.get_err(x, y)
    e_xy   = (ex**2 + ey**2)**0.5

    # 进入判据：e_xy < 阈值并保持一段时间
    now = time.time()
    if e_xy < BACKHOME_EPS_M_ENTER:
        _align_ok_since = _align_ok_since or now
        if now - _align_ok_since >= ALIGN_HOLD_SEC:
            print(f"[BACKHOME] OK -> ALIGNING | e_xy={e_xy:.3f} "
                  f"vx={vx:.2f} vy={vy:.2f}", flush=True)
            state = 'ALIGNING'
    else:
        _align_ok_since = None

        

# -----------------------------
# 阶段：ALIGNING（仅XY对齐）
# -----------------------------
def aligning(mc: MotionCommander, pid_xy: PIDController2D):
    global state, _align_ok_since

    if pix_err_ema is None:
        return

    now = time.time()
    if (last_seen_ts is None) or (now - last_seen_ts > MISS_TIMEOUT):
        mc.start_linear_motion(0.0, 0.0, 0.0)
        _align_ok_since = None
        print("[ALIGNING] target lost -> hold", flush=True)
        return

    z  = max(0.1, position_estimate[2])
    du, dv = pix_err_ema
    x, y = pix_to_meters(du, dv, z)      # 等效米误差
    vx, vy = pid_xy.compute_velocity(x, y)

    mc.start_linear_motion(vx, vy, 0.0)

    ex, ey = pid_xy.get_err(x, y)
    e_xy   = (ex**2 + ey**2)**0.5

    # 进入判据：e_xy < 阈值并保持一段时间
    if e_xy < ALIGN_EPS_M_ENTER:
        _align_ok_since = _align_ok_since or now
        if now - _align_ok_since >= ALIGN_HOLD_SEC:
            print(f"[ALIGNING] OK -> TRACK_DESCEND | Z={z:.2f} e_xy={e_xy:.3f} "
                  f"vx={vx:.2f} vy={vy:.2f}", flush=True)
            state = 'TRACK_DESCEND'
    else:
        _align_ok_since = None

    print(f"[ALIGNING] Z={z:.2f} e_xy={e_xy:.3f} (<{ALIGN_EPS_M_ENTER:.2f}) "
          f"vx={vx:.2f} vy={vy:.2f}", flush=True)

# -----------------------------
# 阶段：TRACK_DESCEND（下降 + 持续XY矫正）
# -----------------------------
def track_descend(mc: MotionCommander, pid_xy: PIDController2D):
    global state

    if pix_err_ema is None:
        return

    # 丢失目标则悬停（或只做慢速竖直下沉，你可按需改）
    if (last_seen_ts is None) or (time.time() - last_seen_ts > MISS_TIMEOUT):
        mc.start_linear_motion(0.0, 0.0, 0.0)
        print("[TRACK_DESCEND] target lost -> hold", flush=True)
        return

    z  = max(0.05, position_estimate[2])

    # XY 误差 & 速度
    x, y = pix_to_meters(pix_err_ema[0], pix_err_ema[1], z)
    vx, vy = pid_xy.compute_velocity(x, y)

    # Z 速度：恒定下沉（向下为负）
    vz = -DESCENT_RATE

    mc.start_linear_motion(vx, vy, vz)

    ex, ey = pid_xy.get_err(x, y)
    e_xy   = (ex**2 + ey**2)**0.5
    print(f"[TRACK_DESCEND] Z={z:.2f} e_xy={e_xy:.3f} vx={vx:.2f} vy={vy:.2f} vz={vz:.2f}", flush=True)

    # 进入仅Z收尾
    if z <= Z_ONLY_THRESHOLD:
        print(f"[TRACK_DESCEND] -> FINAL_DROP (z <= {Z_ONLY_THRESHOLD*100:.0f} cm)", flush=True)
        state = 'FINAL_DROP'

# -----------------------------
# 阶段：FINAL_DROP（仅Z收尾）
# -----------------------------
def final_drop(mc: MotionCommander):
    global state

    z = max(0.0, position_estimate[2])

    # 根据高度分段控制竖直速度，vx=vy=0
    if z > 0.08:
        vz = -FINAL_DROP_VZ_FAR
    elif z > TOUCHDOWN_Z:
        vz = -FINAL_DROP_VZ_NEAR
    else:
        # 认为非常接近地面，使用 land() 收尾
        print(f"[FINAL_DROP] touchdown zone (z={z:.3f} m) -> mc.land()", flush=True)
        mc.land()
        state = 'LANDED'
        return

    mc.start_linear_motion(0.0, 0.0, vz)
    print(f"[FINAL_DROP] Z={z:.3f} -> vz={vz:.2f} (vx=vy=0)", flush=True)


# -----------------------------
# 主状态机
# -----------------------------
def backhome_landing(scf):
    """
    BACKHOME          : 回家
    ALIGNING          : 仅XY对齐
    TRACK_DESCEND     : 在下降的同时持续XY矫正（单次 start_linear_motion）
    FINAL_DROP        : z <= 0.15m 后，仅Z收尾
    LANDED            : 收尾完成
    """

    pid_backhome = PIDController2D(
        target_point=(0.0, 0.0), 
        kp=1.0, ki=0.3, kd=0.0, 
        output_limit=0.2
    )
    pid_landing = PIDController2D(
        target_point=(0.02, 0.0),
        kp=1.0, ki=0.3, kd=0.0,
        output_limit=XY_SPEED_LIMIT_LANDING  # 二重限幅，保险
    )

    global state
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(1.5)  # 起飞后稳定一下
        while True:
            try:
                if state == 'BACKHOME':
                    back_home(mc, pid_backhome)
                elif state == 'ALIGNING':
                    aligning(mc, pid_landing)
                elif state == 'TRACK_DESCEND':
                    track_descend(mc, pid_landing)
                elif state == 'FINAL_DROP':
                    final_drop(mc)
                elif state == 'LANDED':
                    print("[STATE] Landed. Exit loop.", flush=True)
                    break
            except Exception:
                print("[ERROR] control loop crashed:\n" + traceback.format_exc(), flush=True)
                # 安全：悬停，让 with 退出去自动降落
                mc.start_linear_motion(0.0, 0.0, 0.0)
                break
            time.sleep(0.05)  # 20 Hz

def beacon_resolver_callback(center):
    """
    兼容多种 center 结构；统一输出 (u,v)，v 向上为正
    """
    global pix_err_raw, pix_err_ema, last_seen_ts

    if center is None:
        return

    # 解析 (x, y)
    if isinstance(center, dict):
        if 'center' in center and center['center'] is not None:
            c = center['center']
            if not (isinstance(c, (list, tuple, np.ndarray)) and len(c) >= 2):
                return
            x, y = float(c[0]), float(c[1])
        elif 'x' in center and 'y' in center:
            x, y = float(center['x']), float(center['y'])
        else:
            return
    elif isinstance(center, (list, tuple, np.ndarray)) and len(center) >= 2:
        x, y = float(center[0]), float(center[1])
    else:
        return

    # 像素误差（图像中心为原点；v 取上为正）
    u = y - IMG_CY
    v = x - IMG_CX

    pix_err_raw = (u, v)

    # EMA 平滑
    if pix_err_ema is None:
        pix_err_ema = (u, v)
    else:
        a = PIX_EMA_ALPHA
        pix_err_ema = (
            (1 - a) * pix_err_ema[0] + a * u,
            (1 - a) * pix_err_ema[1] + a * v
        )

    last_seen_ts = time.time()


if __name__ == '__main__':
    # UWB 解析
    ukf_filter = PositionUKF(dt=0.02, win_size=6) # 50Hz data rate
    uwb = UWBReceiver("COM4", 115200, ukf_filter)
    uwb.start()

    # AprilTag 解析
    ip = "10.201.171.4"  # TODO: 改成你的服务器 IP
    april_beacon = HttpAprilResolver(ip, callback=beacon_resolver_callback)
    if hasattr(april_beacon, "start"):
        try:
            april_beacon.start()
        except Exception as e:
            print(f'[AprilResolver] start() failed: {e}', flush=True)


    cflib.crtp.init_drivers()

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:

        scf.cf.param.add_update_callback(group='deck', name='bcFlow2',
                                         cb=param_deck_flow)
        time.sleep(1)

        logconf = LogConfig(name='Position', period_in_ms=10)
        logconf.add_variable('stateEstimate.x', 'float')
        logconf.add_variable('stateEstimate.y', 'float')
        logconf.add_variable('stateEstimate.z', 'float')
        scf.cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(log_pos_callback)


        if not deck_attached_event.wait(timeout=5):
            print('No flow deck detected!')
            sys.exit(1)

        # Arm the Crazyflie
        scf.cf.platform.send_arming_request(True)
        time.sleep(2.0)

        logconf.start()
        backhome_landing(scf)
        logconf.stop()