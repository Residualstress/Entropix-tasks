import logging
import math
import time
import threading

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Wedge
from matplotlib.patches import Arc

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.positioning.motion_commander import MotionCommander


# ====== 你自己的 URI ======
URI = 'radio://0/80/2M/E7E7E7E7E7'

# ====== 飞行与绘图参数 ======
TARGET_Z = 1.5            # 目标高度 (m)
YAW_RATE_DEG = 40        # 旋转角速度 (deg/s)
N_TURNS = 1.3               # 旋转圈数
LOG_PERIOD_MS = 20          # 50 Hz 日志
ANGLE_BIN_DEG = 5        # 角度栅格, 用于同向覆盖
MAX_RANGE_M = 5.0           # 超过量程丢弃
MIN_RANGE_M = 0.05          # 太近丢弃
PLOT_LIM_M = 2.0            # 画布范围
PLOT_HZ = 15                # 实时刷新频率
DIST_AVAILABLE_M = 3.0      # 发现新方向的阈值距离

estimated_yaw = None
estimated_ranges = None

first_jump_angle_range = None

# ====== 日志等级 ======
logging.basicConfig(level=logging.ERROR)


def spiral(scf):
    with MotionCommander(scf, default_height=TARGET_Z) as mc:
        print('起飞中...')

        # v_yaw_init = 80.0
        # k_v_yaw = 5.0
        # t0 = time.time()
        # while time.time() - t0 < 20.0:
        #     v_yaw = v_yaw_init - k_v_yaw * (time.time() - t0)
        #     if v_yaw < 10.0:
        #         v_yaw = 10.0
        #     mc.start_linear_motion(0.4, 0.0, 0.0, v_yaw)
        #     time.sleep(0.1)
        mc.circle_left(1.0, 0.4, 360)
        mc.circle_left(2.0, 0.4, 360)
        print('旋转完成')
        mc.start_linear_motion(0.0, 0.0, 0.0, 0.0)
        mc.stop()

        mc.land()

def log_cb(ts, data, _logconf):
    global estimated_yaw, estimated_ranges
    estimated_yaw = float(data.get('stateEstimate.yaw', 0.0))
    estimated_ranges = {
        0: data.get('range.front'),
        180:  data.get('range.back'),
        90:  data.get('range.left'),
        270: data.get('range.right'),
    }

def main():

    cflib.crtp.init_drivers()

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf

        # ---- 设置日志 ----
        lg = LogConfig(name='spinlog', period_in_ms=LOG_PERIOD_MS)
        lg.add_variable('stateEstimate.yaw', 'float')
        lg.add_variable('range.front', 'uint16_t')
        lg.add_variable('range.back',  'uint16_t')
        lg.add_variable('range.left',  'uint16_t')
        lg.add_variable('range.right', 'uint16_t')
        cf.log.add_config(lg)
        lg.data_received_cb.add_callback(log_cb)
        lg.start()
        spiral(scf)
        lg.stop()
    

if __name__ == '__main__':
    main()
