import logging
import math
import time
import threading

import numpy as np
import matplotlib.pyplot as plt

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig

# ====== 你自己的 URI ======
URI = 'radio://0/80/2M/E7E7E7E7E7'

# ====== 飞行与绘图参数 ======
TARGET_Z = 0.5             # 目标高度 (m)
YAW_RATE_DEG = 36.0        # 旋转角速度 (deg/s)
N_TURNS = 3                 # 旋转圈数
LOG_PERIOD_MS = 20          # 50 Hz 日志
ANGLE_BIN_DEG = 1.0         # 角度栅格, 用于同向覆盖
MAX_RANGE_M = 4.0           # 超过量程丢弃
MIN_RANGE_M = 0.05          # 太近丢弃
PLOT_LIM_M = 5.0            # 画布范围
PLOT_HZ = 15                # 实时刷新频率

# ====== 日志等级 ======
logging.basicConfig(level=logging.ERROR)

# ====== 传感器相对机体朝向 (deg) ======
SENSOR_OFFSETS_DEG = {
    'range.front': 0.0,
    'range.left':  90.0,
    'range.back':  180.0,
    'range.right': -90.0,
}

def wrap_deg(a):
    a = a % 360.0
    if a < 0:
        a += 360.0
    return a

# --------- 数据聚合：角度栅格, 同向覆盖最新 ----------
class SpinMapper:
    """按角度栅格聚合，‘同向覆盖最新’，结束后可填补缺测"""
    def __init__(self, bin_deg=1.0):
        self.bin_deg = float(bin_deg)
        self.nbins = int(round(360.0 / self.bin_deg))
        # 距离与时间戳数组：NaN / -1 表示“还没有值”
        self._dists = np.full(self.nbins, np.nan, dtype=float)
        self._tss   = np.full(self.nbins, -1, dtype=np.int64)
        self._lock = threading.Lock()

    def _ang_to_bin(self, ang_deg: float) -> int:
        # 用 round 对齐到就近角度桶，避免系统性偏置
        idx = int(round(ang_deg / self.bin_deg)) % self.nbins
        return idx

    def add_sample(self, yaw_deg, ranges_mm: dict, timestamp):
        with self._lock:
            for key, dmm in ranges_mm.items():
                if dmm is None or dmm <= 0 or dmm >= 65000:
                    continue
                d = dmm / 1000.0
                if d < MIN_RANGE_M or d > MAX_RANGE_M:
                    continue
                offset = SENSOR_OFFSETS_DEG.get(key)
                if offset is None:
                    continue
                ang = wrap_deg(yaw_deg + offset)
                bin_idx = self._ang_to_bin(ang)
                # —— 同向覆盖最新值 ——（仅当有有效读数时覆盖）
                self._dists[bin_idx] = d
                self._tss[bin_idx]   = int(timestamp)

    def to_xy(self):
        """导出当前有效点云（忽略 NaN 桶）"""
        with self._lock:
            valid = ~np.isnan(self._dists)
            if not np.any(valid):
                return np.empty((0,)), np.empty((0,))
            angs = (np.arange(self.nbins, dtype=float) * self.bin_deg)[valid]
            ds   = self._dists[valid]
        rads = np.deg2rad(angs)
        xs = ds * np.cos(rads)
        ys = ds * np.sin(rads)
        return xs, ys

    def fill_gaps_inplace(self):
        """对 NaN 桶做环形最近邻填补，确保每个方向都有值"""
        with self._lock:
            d = self._dists
            if np.all(np.isnan(d)):
                return  # 没有任何有效值，跳过
            n = self.nbins
            valid = ~np.isnan(d)
            # 预先列出所有有效索引，便于最近邻查询
            valid_idx = np.flatnonzero(valid)
            if valid_idx.size == 0:
                return

            def nearest_valid(i):
                # 在环上找最近的有效桶
                # 逐步向两侧扩展，步长 step 从 1..n
                for step in range(1, n):
                    il = (i - step) % n
                    ir = (i + step) % n
                    if valid[il] and valid[ir]:
                        # 等距时任选其一，或取二者平均（这里取更近的任一）
                        return d[il] if step % 2 == 0 else d[ir]
                    if valid[il]: return d[il]
                    if valid[ir]: return d[ir]
                return np.nan  # 理论到不了

            for i in range(n):
                if np.isnan(d[i]):
                    d[i] = nearest_valid(i)

# --------- 实时绘图器 ----------
class LivePlotter:
    def __init__(self, mapper: SpinMapper):
        self.mapper = mapper
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(6, 6))
        self.scat = self.ax.scatter([], [], s=6, alpha=0.85)
        self.ax.scatter([0], [0], s=40, marker='x')  # 机体原点
        self.ax.set_aspect('equal', adjustable='box')
        self.ax.set_xlim(-PLOT_LIM_M, PLOT_LIM_M)
        self.ax.set_ylim(-PLOT_LIM_M, PLOT_LIM_M)
        self.ax.grid(True)
        self.ax.set_title('Room Outline (Live)')
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def update(self):
        xs, ys = self.mapper.to_xy()
        if xs.size:
            self.scat.set_offsets(np.c_[xs, ys])
        else:
            self.scat.set_offsets(np.empty((0, 2)))
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

def main():
    # 可能你项目里已内置自定义 MotionCommander；否则使用 cflib 自带
    try:
        from cflib.positioning.motion_commander import MotionCommander
    except Exception:
        # 如果你把自定义 MotionCommander 放在同目录，可改成:
        # from motion_commander import MotionCommander
        from cflib.positioning.motion_commander import MotionCommander

    cflib.crtp.init_drivers()

    mapper = SpinMapper(bin_deg=ANGLE_BIN_DEG)
    plotter = LivePlotter(mapper)

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf

        # ---- 设置日志 ----
        lg = LogConfig(name='spinlog', period_in_ms=LOG_PERIOD_MS)
        lg.add_variable('stabilizer.yaw', 'float')
        lg.add_variable('range.front', 'uint16_t')
        lg.add_variable('range.back',  'uint16_t')
        lg.add_variable('range.left',  'uint16_t')
        lg.add_variable('range.right', 'uint16_t')

        def log_cb(ts, data, _logconf):
            yaw = float(data.get('stabilizer.yaw', 0.0))
            yaw = wrap_deg(yaw)  # -180..180 -> 0..360
            ranges = {
                'range.front': data.get('range.front'),
                'range.back':  data.get('range.back'),
                'range.left':  data.get('range.left'),
                'range.right': data.get('range.right'),
            }
            mapper.add_sample(yaw, ranges, ts)

        cf.log.add_config(lg)
        lg.data_received_cb.add_callback(log_cb)
        lg.start()

        try:
            # 用上层 MotionCommander 控制
            with MotionCommander(scf, default_height=TARGET_Z) as mc:
                # 进入 with 会：重置定位->起飞到 default_height（阻塞至到位）
                time.sleep(0.3)  # 小稳态

                # 开始原地左转
                mc.start_turn_left(rate=YAW_RATE_DEG)
                spin_time = (N_TURNS * 360.0) / YAW_RATE_DEG

                # 旋转期间实时刷新
                t0 = time.time()
                refresh_dt = 1.0 / PLOT_HZ
                while time.time() - t0 < spin_time:
                    plotter.update()
                    time.sleep(refresh_dt)

                # 停止旋转，悬停半秒稳定
                mc.stop()
                t1 = time.time()
                while time.time() - t1 < 0.5:
                    plotter.update()
                    time.sleep(refresh_dt)

            # 退出 with 会自动降落关电机
        finally:
            lg.stop()

    # 旋转结束后保留最终图像
    mapper.fill_gaps_inplace()  # 保证每个方向都有点
    plt.ioff()
    plotter.update()
    plt.show(block=True)  # 阻塞到窗口被关闭

if __name__ == '__main__':
    main()
