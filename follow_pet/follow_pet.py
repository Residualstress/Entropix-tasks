import logging
import sys
import time
from threading import Event
import traceback

import numpy as np
import math

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper

from pid_controller.PIDController import PIDController2D
from yolo_detection.yolo_det_module import HttpPetDetection

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

deck_attached_event = Event()

logging.basicConfig(level=logging.ERROR)

DEFAULT_HEIGHT = 0.7

# 相机内参（像素焦距，建议替换为标定值）
FX = 320.1
FY = 320.1

# 图像中心（像素）
IMG_CX = 160
IMG_CY = 120


# 丢失目标判定
MISS_TIMEOUT = 1.5    # 超过该秒数未见目标，则视为丢失（悬停）

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
uwb_position = [0.0, 0.0, 0.0]
yaw_estimate = 0

last_seen_ts = None
pix_err_raw  = None        # (u, v) 像素误差（右正、上正）
pix_err_ema  = None
target_area_sqrt = None

state = 'FOLLOW_PET'         # 'BACKHOME' -> 'ALIGNING' -> 'TRACK_DESCEND' -> 'FINAL_DROP' -> 'LANDED'

# 计时器
_align_ok_since = None


def log_pos_callback(timestamp, data, logconf):
    global position_estimate, yaw_estimate
    position_estimate[0] = data['stateEstimate.x']
    position_estimate[1] = data['stateEstimate.y']
    position_estimate[2] = data['stateEstimate.z']
    yaw_estimate = data['stateEstimate.yaw']


def param_deck_flow(_, value_str):
    value = int(value_str)
    print(value)
    if value:
        deck_attached_event.set()
        print('Deck is attached!')
    else:
        print('Deck is NOT attached!')
        

def pix_to_meters(dv, z):
    """像素误差 -> 等效米误差"""
    z = max(0.1, float(z))
    ex = z * (dv / FX)
    ex = ex *2
    return ex

def pix_to_deg(du):
    return du * 0.1875


# -----------------------------
# 阶段：ALIGNING（仅XY对齐）
# -----------------------------
def pet(mc: MotionCommander, pid_area: PIDController2D, pid_deg: PIDController2D):
    global state, _align_ok_since

    if pix_err_ema is None:
        return

    now = time.time()
    if (last_seen_ts is None) or (now - last_seen_ts > MISS_TIMEOUT):
        mc.start_linear_motion(0.0, 0.0, 0.0, 0.0)
        _align_ok_since = None
        print("[PET] target lost -> hold", flush=True)
        return

    z  = max(0.1, position_estimate[2])
    du, dv = pix_err_ema
    # y = pix_to_meters(dv, z)
    dyaw = pix_to_deg(du)
    vx, _ = pid_area.compute_velocity(target_area_sqrt, 0)
    v_yaw, _ = pid_deg.compute_velocity(dyaw, 0)

    print(f"[PET] area_sqrt={target_area_sqrt:.2f} du={du:.2f} dv={dv:.2f} dyaw={dyaw:.2f} "
          f"vx={vx:.2f} v_yaw={v_yaw:.2f}", flush=True)

    mc.start_linear_motion(vx, 0.0, 0.0, v_yaw)

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
def pet_follow(scf):
    """
    """
    pid_area = PIDController2D(
        target_point=(100.0, 0.0), 
        kp=0.01, ki=0.003, kd=0.0, 
        output_limit=0.2
    )
    pid_yaw = PIDController2D(
        target_point=(0.0, 0.0),
        kp=4.0, ki=1.2, kd=0.0,
        output_limit=15.0  # 二重限幅，保险
    )

    global state
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(1.5)  # 起飞后稳定一下
        while True:
            try:
                if state == 'FOLLOW_PET':
                    pet(mc, pid_area, pid_yaw)
            except Exception:
                print("[ERROR] control loop crashed:\n" + traceback.format_exc(), flush=True)
                # 安全：悬停，让 with 退出去自动降落
                mc.start_linear_motion(0.0, 0.0, 0.0)
                break
            time.sleep(0.05)  # 20 Hz

def pet_detector_callback(data):
    """
    兼容多种 center 结构；统一输出 (u,v)，v 向上为正
    """
    global pix_err_raw, pix_err_ema, last_seen_ts, target_area_sqrt

    if data is None:
        return

    x,y = data[0]
    target_area_sqrt = math.sqrt(data[1])

    # 像素误差（图像中心为原点；v 取上为正）
    u = x - IMG_CX
    v = y - IMG_CY

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

def uwb_callback(data):
    global uwb_position
    x,y,z = data
    uwb_position = [x,y,z]


if __name__ == '__main__':


    # AprilTag 解析
    ip = "172.20.10.11"  # TODO: 改成你的服务器 IP
    april_beacon = HttpPetDetection(ip, callback=pet_detector_callback)
    if hasattr(april_beacon, "start"):
        try:
            april_beacon.start()
        except Exception as e:
            print(f'[PetResolver] start() failed: {e}', flush=True)


    cflib.crtp.init_drivers()

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:

        scf.cf.param.add_update_callback(group='deck', name='bcFlow2',
                                         cb=param_deck_flow)
        time.sleep(1)

        logconf = LogConfig(name='Position', period_in_ms=10)
        logconf.add_variable('stateEstimate.x', 'float')
        logconf.add_variable('stateEstimate.y', 'float')
        logconf.add_variable('stateEstimate.z', 'float')
        logconf.add_variable('stateEstimate.yaw', 'float')
        scf.cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(log_pos_callback)


        if not deck_attached_event.wait(timeout=5):
            print('No flow deck detected!')
            sys.exit(1)

        # Arm the Crazyflie
        scf.cf.platform.send_arming_request(True)
        time.sleep(2.0)

        logconf.start()
        pet_follow(scf)
        logconf.stop()