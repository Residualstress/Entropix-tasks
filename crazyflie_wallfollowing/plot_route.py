import pandas as pd
import matplotlib.pyplot as plt

# 读取 CSV 文件
df = pd.read_csv('position_log.csv')  # 换成你的文件名

# 提取 x, y
x = df['x']
y = df['y']

# 绘制轨迹
plt.figure(figsize=(6,6))
plt.plot(x, y, '-o', linewidth=1.5, markersize=3, label='Trajectory')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Trajectory Plot')
plt.legend()
plt.grid(True)
plt.axis('equal')  # 保持比例一致
plt.show()
