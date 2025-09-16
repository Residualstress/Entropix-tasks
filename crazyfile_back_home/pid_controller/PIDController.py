from collections import deque
import numpy as np
from simple_pid import PID

class PIDController2D:
    def __init__(self, target_point=(0.0, 0.0), kp=1.0, ki=0.0, kd=0.1,
                 output_limit=1.0, history_len=50):
        # 初始化 PID 控制器
        self.pid_x = PID(kp, ki, kd, setpoint=target_point[0])
        self.pid_y = PID(kp, ki, kd, setpoint=target_point[1])

        # 限制输出范围 = 最大速度
        self.pid_x.output_limits = (-output_limit, output_limit)
        self.pid_y.output_limits = (-output_limit, output_limit)

        # 历史误差记录，用于稳定性分析
        self.history_len = history_len
        self.error_history_x = deque(maxlen=history_len)
        self.error_history_y = deque(maxlen=history_len)

    def set_target(self, target_point):
        """设置目标点 (x,y)"""
        self.pid_x.setpoint = target_point[0]
        self.pid_y.setpoint = target_point[1]

    def compute_velocity(self, x, y):
        """ 输入当前位置 (x,y)，输出期望速度 (vx,vy) """
        error_x = self.pid_x.setpoint - x
        error_y = self.pid_y.setpoint - y

        # 记录误差
        self.error_history_x.append(error_x)
        self.error_history_y.append(error_y)

        vx = self.pid_x(x) # 这里必须是正号，不然会影响别uwb的PID
        vy = self.pid_y(y) # 由于在beacon_landing中把err修正为了飞机相对于beacon的坐标，这里为正
        return vx, vy

    def get_stability(self):
        """
        衡量稳定性：
        - 误差的标准差越小，说明越稳定
        - 返回一个 0~1 的稳定性指标（1 表示非常稳定）
        """
        if len(self.error_history_x) < 5:  # 数据太少不计算
            return 0.0

        std_x = np.std(self.error_history_x)
        std_y = np.std(self.error_history_y)

        # 简单归一化，假设 std<0.01 就算稳定 ~ 1.0
        stability_x = 1.0/(1.0 + std_x)
        stability_y = 1.0/(1.0 + std_y)

        return (stability_x + stability_y) / 2.0


class PIDController3D:
    def __init__(self, target_point=(0.0, 0.0, 0.0), 
                 kp=1.0, ki=0.0, kd=0.1,
                 output_limit=1.0, history_len=50):
        # 初始化 PID 控制器
        self.pid_x = PID(kp, ki, kd, setpoint=target_point[0])
        self.pid_y = PID(kp, ki, kd, setpoint=target_point[1])
        self.pid_z = PID(kp, ki, kd, setpoint=target_point[2])

        # 限制输出范围
        self.pid_x.output_limits = (-output_limit, output_limit)
        self.pid_y.output_limits = (-output_limit, output_limit)
        self.pid_z.output_limits = (-output_limit, output_limit)

        # 历史误差记录
        self.history_len = history_len
        self.error_history_x = deque(maxlen=history_len)
        self.error_history_y = deque(maxlen=history_len)
        self.error_history_z = deque(maxlen=history_len)

    def set_target(self, target_point):
        """设置目标点 (x,y,z)"""
        self.pid_x.setpoint = target_point[0]
        self.pid_y.setpoint = target_point[1]
        self.pid_z.setpoint = target_point[2]

    def compute_velocity(self, x, y, z):
        """ 输入当前位置 (x,y,z)，输出期望速度 (vx,vy,vz) """
        error_x = self.pid_x.setpoint - x
        error_y = self.pid_y.setpoint - y
        error_z = self.pid_z.setpoint - z

        # 记录误差
        self.error_history_x.append(error_x)
        self.error_history_y.append(error_y)
        self.error_history_z.append(error_z)

        vx = self.pid_x(x)
        vy = self.pid_y(y)
        vz = self.pid_z(z)

        return vx, vy, vz

    def get_stability(self):
        """
        衡量稳定性：
        - 根据误差标准差评估
        - 返回一个 0~1 的稳定性指标
        """
        if len(self.error_history_x) < 5:
            return 0.0

        std_x = np.std(self.error_history_x)
        std_y = np.std(self.error_history_y)
        std_z = np.std(self.error_history_z)

        stability_x = 1.0/(1.0+std_x)
        stability_y = 1.0/(1.0+std_y)
        stability_z = 1.0/(1.0+std_z)

        return (stability_x + stability_y + stability_z) / 3.0
