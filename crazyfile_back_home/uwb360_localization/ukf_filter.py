import numpy as np
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.kalman import MerweScaledSigmaPoints
from collections import deque

class PositionUKF:
    def __init__(self, dt=0.025, win_size=5):
        """
        dt: 采样时间 (s)
        win_size: 滑动窗口大小 (滤波后再平滑)
        """
        self.dt = dt
        self.win_size = win_size
        self.buffer_x = deque(maxlen=win_size)
        self.buffer_y = deque(maxlen=win_size)

        # UKF 状态: [px, py, vx, vy]
        points = MerweScaledSigmaPoints(n=4, alpha=0.1, beta=2.0, kappa=0.0)
        self.ukf = UKF(dim_x=4, dim_z=2,
                       fx=self.fx, hx=self.hx, dt=dt,
                       points=points)
        # 初始状态
        self.ukf.x = np.zeros(4)
        self.ukf.P *= 10.0

        # 过程噪声
        q = 1.0
        self.ukf.Q = np.array([[dt**4/4, 0, dt**3/2, 0],
                               [0, dt**4/4, 0, dt**3/2],
                               [dt**3/2, 0, dt**2, 0],
                               [0, dt**3/2, 0, dt**2]]) * q**2

        # 测量噪声 (px, py)
        r_pos = 0.1  # m
        self.ukf.R = np.diag([r_pos**2, r_pos**2])

    # 状态转移函数
    def fx(self, x, dt):
        px, py, vx, vy = x
        px += vx * dt
        py += vy * dt
        return np.array([px, py, vx, vy])

    # 测量函数
    def hx(self, x):
        return np.array([x[0], x[1]])

    def update(self, x_meas, y_meas):
        """
        输入: x_meas, y_meas (测量坐标，单位 m 或 cm，一定要统一)
        输出: x_filt, y_filt (滤波 + 平滑后的坐标)
        """
        z = np.array([x_meas, y_meas])
        self.ukf.predict()
        self.ukf.update(z)

        x_filt, y_filt = self.ukf.x[0], self.ukf.x[1]

        # 加入滑动窗口
        self.buffer_x.append(x_filt)
        self.buffer_y.append(y_filt)

        # 平滑输出
        x_smooth = np.mean(self.buffer_x)
        y_smooth = np.mean(self.buffer_y)
        return x_smooth, y_smooth
