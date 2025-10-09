import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

# 模拟一组雷达极径数据
r_list = [1.2, 1.5, 1.8, 2.0, 1.6, 1.1, 0.9, 1.3, 1.7, 1.9] * 3 + [1.2, 1.5, 1.8, 2.0, 1.6, 1.1]
angle_step = 10  # 每个扇区的角度（单位：度）

fig, ax = plt.subplots()

# 遍历每个扇区，逐个绘制
for i, r in enumerate(r_list):
    theta1 = i * angle_step
    theta2 = (i + 1) * angle_step

    # 填充扇形（半透明）
    wedge = patches.Wedge(
        (0, 0),                 # 圆心
        r,                      # 半径
        theta1, theta2,         # 起止角度
        facecolor='skyblue',
        alpha=0.5,
        edgecolor='none'        # 不画边
    )
    ax.add_patch(wedge)

    # 再画出圆弧边线（只画弧，不画半径）
    theta = np.linspace(np.radians(theta1), np.radians(theta2), 100)
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    ax.plot(x, y, color='skyblue', linewidth=1.5)

# 调整显示比例
ax.set_aspect('equal')
rmax = max(r_list)
ax.set_xlim(-rmax * 1.1, rmax * 1.1)
ax.set_ylim(-rmax * 1.1, rmax * 1.1)
ax.set_title("2D SLAM 扇形可视化", fontsize=12)
plt.show()
