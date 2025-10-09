import logging
import sys
import time
from threading import Event
import traceback
import requests

import numpy as np
import math

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper

from uwb360_localization.ukf_filter import PositionUKF
from uwb360_localization.uwb360_receiver import UWB360Receiver
from pid_controller.PIDController import PIDController2D, PIDController1D
from apriltag_beacon.apriltag_beacon import HttpAprilResolver
from yolo_detection.yolo_det_module import HttpPetDetection

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

deck_attached_event = Event()

logging.basicConfig(level=logging.ERROR)

ip = "172.20.10.14"  # TODO: 改成你的服务器 IP

DEFAULT_HEIGHT = 0.7

pet_beacon = None
april_beacon = None

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
uwb_position = [0.0, 0.0, 0.0]
yaw_estimate = 0

last_seen_ts = None
pix_err_raw  = None        # (u, v) 像素误差（右正、上正）
pix_err_ema  = None
target_area_sqrt = None

start_time_stamp = None
FIND_PET_TIMEOUT = 3

state = 'FIND_PET'         # 'BACKHOME' -> 'ALIGNING' -> 'TRACK_DESCEND' -> 'FINAL_DROP' -> 'LANDED'

SERVO_DOWN_ANGLE = 180
SERVO_FORWARD_ANGLE = 90
SERVO_45_ANGLE = 135

# 计时器
_align_ok_since = None

def servo_set_angle(angle):
    print(f'http://{ip}:8080/servo?angle={angle}')
    requests.get(f'http://{ip}:8080/servo?angle={angle}')

def yaw_to_rot2d(yaw_deg=None, yaw_rad=None):
    """
    根据 yaw 角度生成二维旋转矩阵。
    :param yaw_deg: 偏航角（度）
    :param yaw_rad: 偏航角（弧度）
    :return: 2x2 numpy 旋转矩阵
    """
    if yaw_rad is None:
        if yaw_deg is None:
            raise ValueError("必须提供 yaw_deg 或 yaw_rad")
        yaw_rad = np.deg2rad(-yaw_deg)
    
    c, s = np.cos(yaw_rad), np.sin(yaw_rad)
    R = np.array([[c, -s],
                  [s,  c]])
    return R

def log_pos_callback(timestamp, data, logconf):
    global position_estimate, yaw_estimate
    position_estimate[0] = data['stateEstimate.x']
    position_estimate[1] = data['stateEstimate.y']
    position_estimate[2] = data['stateEstimate.z']
    # position_estimate[2] = data['range.zrange'] / 1000.0
    yaw_estimate = data['stateEstimate.yaw']


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

def find_pet(mc: MotionCommander, pid_area: PIDController1D, pid_yaw: PIDController1D):
    def pix_to_deg(du):
        return du * 0.1875

    global state, _align_ok_since

    if pix_err_ema is None:
        return
    
    print(f'FINDPET {time.time() - start_time_stamp:.2f}', flush=True)
    if time.time() - start_time_stamp > FIND_PET_TIMEOUT:
        state = 'BACKHOME'
        print("1[PET] FIND_PET_TIMEOUT -> BACKHOME", flush=True)
        pet_beacon.stop() # 停止解析
        print("[PET] FIND_PET_TIMEOUT -> BACKHOME", flush=True)
        servo_set_angle(SERVO_DOWN_ANGLE)
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
    vx = pid_area.compute_velocity(target_area_sqrt)
    v_yaw = pid_yaw.compute_velocity(dyaw)

    print(f"[PET] area_sqrt={target_area_sqrt:.2f} du={du:.2f} dv={dv:.2f} dyaw={dyaw:.2f} "
          f"vx={vx:.2f} v_yaw={v_yaw:.2f}", flush=True)

    mc.start_linear_motion(vx, 0.0, 0.0, v_yaw)

    

def back_home(mc: MotionCommander, pid_xy: PIDController2D):
    global state, _align_ok_since

    x,y = uwb_position[0], uwb_position[1]

    vx, vy = pid_xy.compute_velocity(x, y)

    

    R = yaw_to_rot2d(yaw_deg=yaw_estimate)
    v = R @ np.array([vx, vy])
    vx_yaw = v[0]
    vy_yaw = v[1]

    print(f'x: {x}, y: {y}, yaw: {yaw_estimate},vel_x: {vx}, vel_y: {vy},vel_x: {vx_yaw}, vel_y: {vy_yaw}')

    mc.start_linear_motion(vx_yaw, vy_yaw, 0)

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
            uwb.stop()
            april_beacon.start() #  启动 AprilTag 解析
            servo_set_angle(SERVO_DOWN_ANGLE)
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
def findpet_backhome_landing(scf):
    """
    BACKHOME          : 回家
    ALIGNING          : 仅XY对齐
    TRACK_DESCEND     : 在下降的同时持续XY矫正（单次 start_linear_motion）
    FINAL_DROP        : z <= 0.15m 后，仅Z收尾
    LANDED            : 收尾完成
    """
    pid_area = PIDController1D(
        target_point=100.0,
        kp=0.01, ki=0.003, kd=0.0, 
        output_limit=0.2
    )
    pid_yaw = PIDController1D(
        target_point=0.0,
        kp=8.0, ki=2.4, kd=0.0,
        output_limit=15.0  # 二重限幅，保险
    )
    pid_backhome = PIDController2D(
        target_point=(0.3, -1.5), 
        kp=1.0, ki=0.3, kd=0.0, 
        output_limit=0.2
    )
    pid_landing = PIDController2D(
        target_point=(0.04, 0.0),
        kp=0.8, ki=0.2, kd=0.0,
        output_limit=XY_SPEED_LIMIT_LANDING  # 二重限幅，保险
    )

    global state
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(1.5)  # 起飞后稳定一下
        global start_time_stamp
        start_time_stamp = time.time()
        while True:
            try:
                if state == 'FIND_PET':
                    find_pet(mc, pid_area, pid_yaw)
                elif state == 'BACKHOME':
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

def uwb_callback(data):
    global uwb_position
    x,y,z = data
    uwb_position = [x,y,z]


if __name__ == '__main__':
    # UWB 解析
    servo_set_angle(SERVO_DOWN_ANGLE)

    ukf_filter = PositionUKF(dt=0.025, win_size=3) # 50Hz data rate
    uwb = UWB360Receiver("COM5", uwb_callback, None)
    uwb.start()

    # AprilTag 解析
    

    servo_set_angle(SERVO_45_ANGLE)

    pet_beacon = HttpPetDetection(ip, callback=pet_detector_callback)
    pet_beacon.start()

    april_beacon = HttpAprilResolver(ip, callback=beacon_resolver_callback)

    cflib.crtp.init_drivers()

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:

        scf.cf.param.add_update_callback(group='deck', name='bcFlow2',
                                         cb=param_deck_flow)
        time.sleep(1)

        logconf = LogConfig(name='Position', period_in_ms=10)
        logconf.add_variable('stateEstimate.x', 'float')
        logconf.add_variable('stateEstimate.y', 'float')
        logconf.add_variable('stateEstimate.z', 'float')
        # logconf.add_variable('range.zrange', 'uint16_t')
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
        findpet_backhome_landing(scf)
        logconf.stop()