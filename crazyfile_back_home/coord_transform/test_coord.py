import numpy as np
import time
from coord_transform import CoordTransform2D, CoordTransformThread, global_fixed_coords, global_moving_coords

# ===== 定义真实旋转 + 平移 =====
theta = np.deg2rad(30)
R_true = np.array([[np.cos(theta), -np.sin(theta)],
                   [np.sin(theta),  np.cos(theta)]])
t_true = np.array([[2.0], [5.0]])

def apply_true_transform(point):
    p = np.array(point).reshape(2,1)
    return (R_true @ p + t_true).flatten()

# ===== 模拟带噪声数据 =====
np.random.seed(42)
noise_level = 0.1  # 噪声标准差 (单位：坐标值)

for _ in range(100):
    p_src = np.random.uniform(-10, 10, size=2)   # 移动坐标系下的点
    p_dst = apply_true_transform(p_src)

    # 给固定坐标系和移动坐标系都加上噪声（模拟测量误差）
    p_src_noisy = p_src + np.random.normal(0, noise_level, size=2)
    p_dst_noisy = p_dst + np.random.normal(0, noise_level, size=2)

    global_moving_coords.append(p_src_noisy)
    global_fixed_coords.append(p_dst_noisy)

# ===== 启动线程 =====
ct = CoordTransform2D(history_len=100)
ct_thread = CoordTransformThread(ct, interval=0.01)
ct_thread.start()

# ===== 等待收敛 =====
time.sleep(1.5)

# 输出估计结果
print("Estimated R:\n", ct.R)
print("True R:\n", R_true)
print("Estimated t:", ct.t.flatten())
print("True t:", t_true.flatten())

# print(global_fixed_coords)

for i in range(len(global_moving_coords)):
    p_src = global_moving_coords[i]
    p_dst = global_fixed_coords[i]
    est_point = ct.transform(p_src)
    err = np.linalg.norm(est_point - p_dst)
    print("Point", i, "Error:", err)

# # 测试单点
# test_point = np.array([3, 4])
# est_point = ct.transform(test_point)
# true_point = apply_true_transform(test_point)
# print("Test point:", test_point)
# print("Estimated transformed:", est_point)
# print("True transformed:", true_point)

ct_thread.stop()
