"""
OnlineTransformEstimator
- 目标：实时将坐标系 A(x1,y1) 映射到坐标系 B(x2,y2)
- 功能：
  - 在滑动窗口内估计刚性变换 (rotation R, translation t)，可选尺度
  - 能估计并补偿整数帧延迟（基于互相关）
  - 可选 RANSAC 来提高鲁棒性（剔除 outlier）
  - 对估计参数（角度/tx/ty）做指数平滑
  - 提供 map_point(x1) 把单点从系 A 映射到系 B
  - 支持带时间戳的 update（若有异步采样）
"""

import numpy as np
import math
from collections import deque

def rotation_from_R(R):
    # 返回 angle (rad) 从 2x2 旋转矩阵
    return math.atan2(R[1,0], R[0,0])

def make_rotation(theta):
    return np.array([[math.cos(theta), -math.sin(theta)],
                     [math.sin(theta),  math.cos(theta)]])

def compute_rigid_transform(A, B, with_scale=False):
    """
    A, B: Nx2 numpy arrays (配对点)，求 s, R, t 使得 s*R*A + t = B 最小二乘
    返回 (R (2x2), t (2,), s (float))
    """
    assert A.shape == B.shape
    N = A.shape[0]
    if N == 0:
        return np.eye(2), np.zeros(2), 1.0
    mu_A = A.mean(axis=0)
    mu_B = B.mean(axis=0)
    A_c = A - mu_A
    B_c = B - mu_B
    H = A_c.T @ B_c
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1,:] *= -1
        R = Vt.T @ U.T
    if with_scale:
        varA = (A_c**2).sum()
        s = S.sum() / (varA + 1e-12)
    else:
        s = 1.0
    t = mu_B - s * (R @ mu_A)
    return R, t, s

def estimate_delay_by_xcorr(a, b, max_lag=50):
    """
    a, b : 1D numpy arrays (时间序列的同一分量)
    返回整数 lag，表示 b 相对 a 的延迟（b delayed relative to a）
    如果找不到明显峰值则返回 0（可视需求改为 None）
    """
    if len(a) == 0 or len(b) == 0:
        return 0
    L = min(len(a), len(b))
    a0 = a[-L:].astype(float) - a[-L:].mean()
    b0 = b[-L:].astype(float) - b[-L:].mean()
    corr = np.correlate(a0, b0, mode='full')
    lags = np.arange(-L+1, L)
    idx = np.argmax(corr)
    lag = lags[idx]
    if abs(lag) > max_lag:
        return 0
    return int(lag)

class OnlineTransformEstimator:
    def __init__(self,
                 window_size=100,
                 max_delay_search=30,
                 smoothing_alpha=0.15,
                 use_ransac=False,
                 ransac_iters=100,
                 ransac_tol=0.05,
                 with_scale=False):
        """
        参数说明：
        - window_size: 用于估计的滑动窗口长度（帧数）
        - max_delay_search: 延迟搜索最大帧数
        - smoothing_alpha: 对角度/tx/ty 做 EMA 平滑系数 (0-1)
        - use_ransac: 是否对配对点用 RANSAC 提高鲁棒性
        - ransac_iters, ransac_tol: RANSAC 参数
        - with_scale: 是否估计尺度 s
        """
        self.window_size = window_size
        self.max_delay_search = max_delay_search
        self.smoothing_alpha = smoothing_alpha
        self.use_ransac = use_ransac
        self.ransac_iters = ransac_iters
        self.ransac_tol = ransac_tol
        self.with_scale = with_scale

        # 缓冲区（deque）保存最近点对，元素为 (ts, x1, y1, x2, y2)
        self.buffer = deque(maxlen=window_size*2)  # 多留一点以便 delay align
        # 平滑后的参数（初始无效）
        self.ema_theta = None
        self.ema_tx = None
        self.ema_ty = None
        self.ema_s = None

        # 最近一次估计结果
        self.last_R = np.eye(2)
        self.last_t = np.zeros(2)
        self.last_s = 1.0
        self.last_lag = 0

    def add_pair(self, x1y1, x2y2, ts=None):
        """
        添加一帧配对点
        - x1y1: (2,) or list tuple
        - x2y2: (2,)
        - ts: 可选时间戳（float），不提供则按序号处理
        """
        if ts is None:
            ts = len(self.buffer) if len(self.buffer) == 0 else (self.buffer[-1][0] + 1)
        self.buffer.append((ts, float(x1y1[0]), float(x1y1[1]), float(x2y2[0]), float(x2y2[1])))

        # 每次添加后我们可以尝试更新估计（也可由外部按固定频率触发 update()）
        return self.update_estimate()

    def _get_aligned_window(self):
        """
        从 buffer 中取最近 window_size 对数据并尝试对齐（基于互相关估计延迟）
        返回两个等长的 numpy arrays A (Nx2 from x1) and B (Nx2 from x2)
        """
        if len(self.buffer) < 10:
            return np.zeros((0,2)), np.zeros((0,2))

        # 取最近的 window_size*2 数据以便延迟对齐
        data = list(self.buffer)[-self.window_size*2:]
        ts = np.array([d[0] for d in data])
        a = np.vstack([[d[1], d[2]] for d in data])
        b = np.vstack([[d[3], d[4]] for d in data])

        # 用 x 分量做互相关估计延迟（可以使用速度幅值等更稳健信号）
        lag = estimate_delay_by_xcorr(a[:,0], b[:,0], max_lag=min(self.max_delay_search, len(a)//4))
        self.last_lag = lag

        # 对齐：如果 lag>0 则 b 相对于 a 延迟 lag（说明 b 滞后）
        if lag > 0:
            A = a[lag:]
            B = b[:len(A)]
        elif lag < 0:
            B = b[-lag:]
            A = a[:len(B)]
        else:
            A = a
            B = b

        # 最终截取到 window_size（从尾部）
        if len(A) > self.window_size:
            A = A[-self.window_size:]
            B = B[-self.window_size:]
        return A, B

    def _ransac_rigid(self, A, B):
        """
        简易 RANSAC：随机采两点估计变换，计算内点数，返回最佳 R,t
        """
        N = len(A)
        best_inliers = []
        best_R, best_t, best_s = np.eye(2), np.zeros(2), 1.0
        if N < 3:
            return best_R, best_t, best_s
        rng = np.random.default_rng()
        for _ in range(self.ransac_iters):
            ids = rng.choice(N, size=2, replace=False)
            try:
                R0, t0, s0 = compute_rigid_transform(A[ids], B[ids], with_scale=self.with_scale)
            except Exception:
                continue
            # 计算误差
            pred = (s0 * (R0 @ A.T)).T + t0
            dists = np.linalg.norm(pred - B, axis=1)
            inliers = np.where(dists <= self.ransac_tol)[0]
            if len(inliers) > len(best_inliers):
                best_inliers = inliers
                best_R, best_t, best_s = R0, t0, s0
        # 用内点做 refine
        if len(best_inliers) >= 3:
            Rf, tf, sf = compute_rigid_transform(A[best_inliers], B[best_inliers], with_scale=self.with_scale)
            return Rf, tf, sf
        else:
            # 没找到好的内点，退回全量估计
            return compute_rigid_transform(A, B, with_scale=self.with_scale)

    def update_estimate(self):
        """
        从 buffer 获取对齐窗口并估计变换，更新平滑参数
        返回 (R, t, s, lag)
        """
        A, B = self._get_aligned_window()
        if len(A) < 3:
            return self.last_R, self.last_t, self.last_s, self.last_lag

        if self.use_ransac:
            R, t, s = self._ransac_rigid(A, B)
        else:
            R, t, s = compute_rigid_transform(A, B, with_scale=self.with_scale)

        # 把 R 转为角度再 EMA 平滑（直接 EMA 矩阵不是很稳定）
        theta = rotation_from_R(R)
        tx, ty = t[0], t[1]

        if self.ema_theta is None:
            self.ema_theta = theta
            self.ema_tx = tx
            self.ema_ty = ty
            self.ema_s = s
        else:
            a = self.smoothing_alpha
            # 注意角度 wrap-around：做最短差值
            diff = (theta - self.ema_theta + math.pi) % (2*math.pi) - math.pi
            self.ema_theta = self.ema_theta + a * diff
            self.ema_tx = (1-a)*self.ema_tx + a*tx
            self.ema_ty = (1-a)*self.ema_ty + a*ty
            self.ema_s = (1-a)*self.ema_s + a*s

        R_sm = make_rotation(self.ema_theta)
        t_sm = np.array([self.ema_tx, self.ema_ty])
        s_sm = self.ema_s

        self.last_R = R_sm
        self.last_t = t_sm
        self.last_s = s_sm

        return R_sm, t_sm, s_sm, self.last_lag

    def map_point(self, x1y1):
        """
        把单点从坐标系 A 映射到系 B： return point2
        根据模型： point2 ~= s * R * point1 + t
        """
        p = np.array([float(x1y1[0]), float(x1y1[1])])
        return (self.last_s * (self.last_R @ p)) + self.last_t

    def batch_process_dataframe(self, df, ts_col=None, x1_cols=('x1','y1'), x2_cols=('x2','y2')):
        """
        辅助函数：给定 pandas.DataFrame（含 x1,y1,x2,y2），按行执行 add_pair 并返回统计结果
        返回 dict 包含 history of estimates 和 per-row mapped point & error
        """
        import pandas as pd
        history = []
        mapped = []
        errors = []
        idx = 0
        for i, row in df.iterrows():
            ts = row[ts_col] if ts_col is not None else idx
            self.add_pair((row[x1_cols[0]], row[x1_cols[1]]),
                          (row[x2_cols[0]], row[x2_cols[1]]),
                          ts=ts)
            mapped_pt = self.map_point((row[x1_cols[0]], row[x1_cols[1]]))
            mapped.append(mapped_pt)
            err = np.linalg.norm(mapped_pt - np.array([row[x2_cols[0]], row[x2_cols[1]]]))
            errors.append(err)
            history.append({
                'idx': i,
                'theta': self.ema_theta,
                'tx': self.ema_tx,
                'ty': self.ema_ty,
                's': self.ema_s,
                'lag': self.last_lag,
                'err': err
            })
            idx += 1
        hist_df = pd.DataFrame(history)
        mapped_arr = np.vstack(mapped) if len(mapped)>0 else np.zeros((0,2))
        return {'history': hist_df, 'mapped': mapped_arr, 'errors': np.array(errors)}


