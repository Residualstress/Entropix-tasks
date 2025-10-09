
import threading

import numpy as np



class SpinMapper:
    """按角度栅格聚合，‘同向覆盖最新’，结束后可填补缺测"""
    def __init__(self, bin_deg=1.0, max_range_m=5.0, min_range_m=0.0):
        self.bin_deg = float(bin_deg)
        self.max_range_m = max_range_m
        self.min_range_m = min_range_m
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
                if d < self.min_range_m:
                    continue
                if d > self.max_range_m:
                    d = self.max_range_m + 10.0
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