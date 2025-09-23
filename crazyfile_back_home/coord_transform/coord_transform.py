import numpy as np
import threading
import time

# ====== 全局变量（模拟外部实时更新的坐标） ======
global_fixed_coords = [0,0]
global_moving_coords = [0,0]

class CoordTransform2D:
    def __init__(self, history_len=200):
        self.history_len = history_len
        self.src_points = []  # 移动坐标系
        self.dst_points = []  # 固定坐标系
        self.R = np.eye(2)
        self.t = np.zeros((2, 1))
        self.lock = threading.Lock()

    def add_pair(self, src, dst):
        with self.lock:
            self.src_points.append(np.array(src))
            self.dst_points.append(np.array(dst))
            while len(self.src_points) > self.history_len:
                self.src_points.pop(0)
                self.dst_points.pop(0)

    def compute_transform(self):
        with self.lock:
            if len(self.src_points) < 10:
                return False

            src = np.array(self.src_points).T
            dst = np.array(self.dst_points).T

            src_mean = np.mean(src, axis=1, keepdims=True)
            dst_mean = np.mean(dst, axis=1, keepdims=True)

            src_centered = src - src_mean
            dst_centered = dst - dst_mean

            H = src_centered @ dst_centered.T
            U, _, Vt = np.linalg.svd(H)
            R = Vt.T @ U.T

            if np.linalg.det(R) < 0:
                Vt[1, :] *= -1
                R = Vt.T @ U.T

            t = dst_mean - R @ src_mean

            self.R, self.t = R, t
            return True

    def transform(self, point):
        p = np.array(point).reshape(2, 1)
        with self.lock:
            return (self.R @ p + self.t).flatten()
        
    def transform_rot(self, point): # only rotation
        p = np.array(point).reshape(2, 1)
        with self.lock:
            return (self.R @ p).flatten()

    def get_matrix(self):
        with self.lock:
            M = np.eye(3)
            M[:2, :2] = self.R
            M[:2, 2] = self.t.flatten()
            return M


class CoordTransformThread:
    def __init__(self, transformer: CoordTransform2D, src_coord, dst_coord, interval=0.05):
        self.transformer = transformer
        self.interval = interval
        self.thread = threading.Thread(
            target=self._run,
            args=(src_coord, dst_coord),
            daemon=True
        )
        self.running = False

    def start(self):
        self.running = True
        self.thread.start()

    def stop(self):
        self.running = False
        self.thread.join()

    def _run(self, src_coord, dst_coord):
        while self.running:
            src = np.array(src_coord[:2])
            dst = np.array(dst_coord[:2])
            self.transformer.add_pair(src, dst)
            self.transformer.compute_transform()
            time.sleep(self.interval)
