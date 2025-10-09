import matplotlib.pyplot as plt
from matplotlib.patches import Wedge
import numpy as np

# 模拟SLAM数据：每个角度的距离值
num_sectors = 360
angles = np.linspace(0, 360, num_sectors, endpoint=False)
r_list = 5 + np.sin(np.deg2rad(angles * 3)) * 2  # 模拟变化的距离值

# 扇区角度宽度（单位°）
sector_width = 360 / num_sectors

fig, ax = plt.subplots(subplot_kw={'aspect': 'equal'})

for i, r in enumerate(r_list):
    start_angle = angles[i]
    end_angle = start_angle + sector_width

    # 绘制扇形
    wedge = Wedge(center=(0, 0),
                  r=r,
                  theta1=start_angle,
                  theta2=end_angle,
                  facecolor='C0',
                  edgecolor='none',  # 不要边框
                  alpha=0.5)
    ax.add_patch(wedge)

# 设定显示范围
max_r = max(r_list)
ax.set_xlim(-max_r - 1, max_r + 1)
ax.set_ylim(-max_r - 1, max_r + 1)

# 去掉坐标轴，展示更干净的雷达图
ax.axis('off')

plt.show()
