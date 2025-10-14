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

from spin_mapper.spin_mapper import SpinMapper
from slam_plotter.live_plotter import LivePlotter
from utils.longest_true_angle_interval import longest_true_angle_interval
from utils.calc_turn_angle import calc_turn_angle

# ====== 你自己的 URI ======
URI = 'radio://0/80/2M/E7E7E7E7E7'

# ====== 飞行与绘图参数 ======
TARGET_Z = 0.3            # 目标高度 (m)

N_TURNS = 1.3               # 旋转圈数
LOG_PERIOD_MS = 20          # 50 Hz 日志
ANGLE_BIN_DEG = 5        # 角度栅格, 用于同向覆盖
MAX_RANGE_M = 5.0           # 超过量程丢弃
MIN_RANGE_M = 0.05          # 太近丢弃
PLOT_LIM_M = 2.0            # 画布范围
PLOT_HZ = 15                # 实时刷新频率

estimated_yaw = None
estimated_ranges = None

first_jump_angle_range = None

state = "TwoJump"
pixel_ema = None

# ====== 日志等级 ======
logging.basicConfig(level=logging.ERROR)

mapper = SpinMapper(bin_deg=ANGLE_BIN_DEG, max_range_m=MAX_RANGE_M, min_range_m=MIN_RANGE_M)
plotter = LivePlotter(mapper, alpha=0.5, plt_lim_m=PLOT_LIM_M)

class TwoJump:
    def __init__(self):
        self.two_jump_state = "SCAN"
        self.t0 = 0
        self.last_jump_angle_range = None
        self.direction = 0
        self.turn_angle = 0
        self.jump_counter = 0
        self.DIST_AVAILABLE_M = 3.0      # 发现新方向的阈值距离
        self.YAW_RATE_DEG = 40        # 旋转角速度 (deg/s)
    def two_jump(self, mc: MotionCommander):
        global state
        if pixel_ema is not None or self.jump_counter >= 2:
            state = "FIND_PET"
            '''
            GET READY FOR FIND PET
            '''
            return
        
        if self.two_jump_state == "SCAN":
            mc.start_turn_left(rate=self.YAW_RATE_DEG)
            plotter.update()
            spin_time = (N_TURNS * 360.0) / self.YAW_RATE_DEG
            if time.time() - self.t0 > spin_time:
                mc.stop()
                angs, dists = mapper.to_polar()
                is_dist_avail = [dist > self.DIST_AVAILABLE_M for dist in dists]
                if self.last_jump_angle_range is None:
                    _ang_start, _ang_end, ang_center = longest_true_angle_interval(angs, is_dist_avail)
                    # print(f"最长连续True区间：{_ang_start:.1f}° → {_ang_end:.1f}°，中心角：{ang_center:.1f}°")
                else:
                    _ang_start, _ang_end, ang_center = longest_true_angle_interval(angs, is_dist_avail, self.last_jump_angle_range)
                    # print(f"最长连续True区间：{_ang_start:.1f}° → {_ang_end:.1f}°，中心角：{ang_center:.1f}°")
                self.direction, self.turn_angle = calc_turn_angle(estimated_yaw, ang_center)
                self.last_jump_angle_range = (_ang_start, _ang_end)
                self.spin_time = self.turn_angle / self.YAW_RATE_DEG
                self.t0 = time.time()
                self.two_jump_state = "TURN"
                print('from scan to turn')
                
        elif self.two_jump_state == "TURN":
            if self.direction == 'left':
                mc.start_turn_left(rate=self.YAW_RATE_DEG)
            else:
                mc.start_circle_right(rate=self.YAW_RATE_DEG)
            if time.time() - self.t0 > self.spin_time:
                mc.stop()
                forward_m = 1.5
                self.spin_time = forward_m / 0.2
                self.t0 = time.time()
                self.two_jump_state = "FORWARD"
                print('from turn to forward')
        
        elif self.two_jump_state == "FORWARD":
            mc.start_linear_motion(0.2, 0.0, 0.0, 0.0)
            if time.time() - self.t0 > self.spin_time or estimated_ranges[0]< 1.0 or estimated_ranges[90]< 0.5 or estimated_ranges[180]< 1.0 or estimated_ranges[270]< 0.5:
                mc.stop()
                self.two_jump_state = "SCAN"
                self.jump_counter += 1
                print('from forward to scan')
                print(f'jump counter: {self.jump_counter}')

        

def ctrl_loop(scf: Crazyflie):
    global state
    twojump = TwoJump()
    print('起飞中...')
    with MotionCommander(scf, default_height=TARGET_Z) as mc:
        time.sleep(1.5)  # 起飞后稳定一下
        state = "TwoJump"
        while True:
            if state == "TwoJump":
                twojump.two_jump(mc)
            elif state == "FIND_PET":
                mc.land()
                state = "LANDED"
                break
            time.sleep(0.1)

def log_cb(ts, data, _logconf):
    global estimated_yaw, estimated_ranges
    estimated_yaw = float(data.get('stateEstimate.yaw', 0.0))
    estimated_ranges = {
        0:   data.get('range.front'),
        180: data.get('range.back'),
        90:  data.get('range.left'),
        270: data.get('range.right')
    }
    mapper.add_sample(estimated_yaw, estimated_ranges, ts)

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
        ctrl_loop(scf)
        lg.stop()
    # 旋转结束后保留最终图像
    mapper.fill_gaps_inplace()  # 保证每个方向都有点
    plt.ioff()
    plotter.update()
    plt.show(block=True)  # 阻塞到窗口被关闭

if __name__ == '__main__':
    main()
