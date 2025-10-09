import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Wedge
from matplotlib.patches import Arc
import threading

# 这些常量请按你的项目中已有的定义保持一致
MIN_RANGE_M = 0.05
MAX_RANGE_M = 6.5
PLOT_LIM_M  = 7.0

# --------- 数据聚合类（保持原样） ----------
class SpinMapper:
    """按角度栅格聚合，‘同向覆盖最新’，结束后可填补缺测"""
    def __init__(self, bin_deg=1.0):
        self.bin_deg = float(bin_deg)
        self.nbins = int(round(360.0 / self.bin_deg))
        self._dists = np.full(self.nbins, np.nan, dtype=float)
        self._tss   = np.full(self.nbins, -1, dtype=np.int64)
        self._lock = threading.Lock()

    def _ang_to_bin(self, ang_deg: float) -> int:
        idx = int(round(ang_deg / self.bin_deg)) % self.nbins
        return idx

    def add_sample(self, yaw_deg, ranges_mm: dict, timestamp):
        with self._lock:
            for key, dmm in ranges_mm.items():
                if dmm is None or dmm <= 0 or dmm >= 65000:
                    continue
                d = dmm / 1000.0
                if d < MIN_RANGE_M:
                    continue
                if d > MAX_RANGE_M:
                    d = MAX_RANGE_M + 1
                # 简化：假设 offset 已在外部处理
                ang = (yaw_deg + key) % 360
                bin_idx = self._ang_to_bin(ang)
                self._dists[bin_idx] = d
                self._tss[bin_idx]   = int(timestamp)

    def to_polar(self):
        """导出角度与距离（含 NaN）"""
        with self._lock:
            angs = np.arange(self.nbins, dtype=float) * self.bin_deg
            ds   = self._dists.copy()
        return angs, ds

    def fill_gaps_inplace(self):
        with self._lock:
            d = self._dists
            if np.all(np.isnan(d)):
                return
            n = self.nbins
            valid = ~np.isnan(d)
            if not np.any(valid):
                return
            valid_idx = np.flatnonzero(valid)

            def nearest_valid(i):
                for step in range(1, n):
                    il = (i - step) % n
                    ir = (i + step) % n
                    if valid[il]: return d[il]
                    if valid[ir]: return d[ir]
                return np.nan
            for i in range(n):
                if np.isnan(d[i]):
                    d[i] = nearest_valid(i)

# --------- 实时绘图器（新版） ----------
class LivePlotter:
    def __init__(self, mapper, alpha=0.5):
        self.mapper = mapper
        self.alpha = alpha
        self.bin_deg = mapper.bin_deg
        self.nbins = mapper.nbins
        self.wedges = []
        self.arcs = []

        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(6, 6))
        self.ax.set_aspect('equal', adjustable='box')
        self.ax.set_xlim(-6, 6)
        self.ax.set_ylim(-6, 6)
        self.ax.grid(True, linestyle='--', alpha=0.3)
        self.ax.scatter([0], [0], s=40, c='red', marker='x', label='Robot')
        self.ax.set_title('2D SLAM Mapper (Live)')
        self.ax.legend()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def _clear_patches(self):
        for w in self.wedges:
            w.remove()
        for a in self.arcs:
            a.remove()
        self.wedges.clear()
        self.arcs.clear()

    def update(self):
        dists = self.mapper._dists.copy()
        bin_deg = self.bin_deg
        nbins = self.nbins
        if np.all(np.isnan(dists)):
            return

        self._clear_patches()

        for i, r in enumerate(dists):
            if np.isnan(r):
                continue
            theta1 = i * bin_deg
            theta2 = theta1 + bin_deg

            # 填充扇区：无边框
            wedge = Wedge(center=(0, 0),
                          r=r,
                          theta1=theta1,
                          theta2=theta2,
                          facecolor='skyblue',
                          alpha=self.alpha,
                          edgecolor='none')
            self.ax.add_patch(wedge)
            self.wedges.append(wedge)

            # 只画外圆弧线
            arc = Arc((0, 0),
                      width=2*r, height=2*r,
                      theta1=theta1,
                      theta2=theta2,
                      edgecolor='steelblue',
                      lw=0.8)
            self.ax.add_patch(arc)
            self.arcs.append(arc)

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()



import time
mapper = SpinMapper(bin_deg=5)
plotter = LivePlotter(mapper)

# 房间参数
ROOM_HALF = 4.0  # 房间半边长（米）

def distance_to_square_wall(angle_deg):
    """给定角度，返回从原点到方形墙壁的距离"""
    theta = np.deg2rad(angle_deg)
    # 计算射线与四条边的交点距离（取最近的正距离）
    dx = ROOM_HALF / abs(np.cos(theta)) if abs(np.cos(theta)) > 1e-6 else np.inf
    dy = ROOM_HALF / abs(np.sin(theta)) if abs(np.sin(theta)) > 1e-6 else np.inf
    return min(dx, dy)

# 模拟扫描
for t in range(200):
    yaw = (t * 2) % 360  # 每帧旋转2°
    # 模拟多方向雷达（例如4个探头在不同偏角）
    fake_data = {
        0: distance_to_square_wall(yaw),
        90: distance_to_square_wall(yaw + 90),
        180: distance_to_square_wall(yaw + 180),
        270: distance_to_square_wall(yaw + 270)
    }
    # 模拟毫米单位（符合 mapper 的输入）
    fake_data = {k: v * 1000 for k, v in fake_data.items()}
    mapper.add_sample(yaw_deg=yaw, ranges_mm=fake_data, timestamp=t)

    plotter.update()
    time.sleep(0.02)