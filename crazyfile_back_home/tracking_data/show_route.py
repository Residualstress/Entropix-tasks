import pandas as pd
import matplotlib.pyplot as plt

# 读取数据
file_path = "data5.csv"  # 换成你的文件路径
df = pd.read_csv(file_path)

# 绘制轨迹
plt.figure(figsize=(8, 6))
plt.plot(df["x1"], df["y1"], label="Trajectory x1y1", alpha=0.8)
plt.plot(df["x2"], df["y2"], label="Trajectory x2y2", alpha=0.8)

# 标记起点和终点
plt.scatter(df["x1"].iloc[0], df["y1"].iloc[0], color="blue", marker="o", s=80, label="Start x1y1")
plt.scatter(df["x1"].iloc[-1], df["y1"].iloc[-1], color="blue", marker="x", s=80, label="End x1y1")

plt.scatter(df["x2"].iloc[0], df["y2"].iloc[0], color="orange", marker="o", s=80, label="Start x2y2")
plt.scatter(df["x2"].iloc[-1], df["y2"].iloc[-1], color="orange", marker="x", s=80, label="End x2y2")

# 图形设置
plt.xlabel("X")
plt.ylabel("Y")
plt.title("Trajectories of x1y1 and x2y2")
plt.legend()
plt.axis("equal")
plt.grid(True)
plt.show()
