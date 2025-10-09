import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Wedge
from matplotlib.patches import Arc

class LivePlotter:
    def __init__(self, mapper, alpha=0.5, plt_lim_m=6):
        self.mapper = mapper
        self.alpha = alpha
        self.bin_deg = mapper.bin_deg
        self.nbins = mapper.nbins
        self.wedges = []
        self.arcs = []

        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(6, 6))
        self.ax.set_aspect('equal', adjustable='box')
        self.ax.set_xlim(-plt_lim_m, plt_lim_m)
        self.ax.set_ylim(-plt_lim_m, plt_lim_m)
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