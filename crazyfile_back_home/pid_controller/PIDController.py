from collections import deque
import numpy as np
from simple_pid import PID

class PIDController2D:
    def __init__(self, target_point=(0.0, 0.0), kp=1.0, ki=0.3, kd=0.0,
                 output_limit=0.5):
        # 初始化 PID 控制器
        self.pid_x = PID(kp, ki, kd, setpoint=target_point[0])
        self.pid_y = PID(kp, ki, kd, setpoint=target_point[1])

        # 限制输出范围 = 最大速度
        self.pid_x.output_limits = (-output_limit, output_limit)
        self.pid_y.output_limits = (-output_limit, output_limit)

    def set_target(self, target_point):
        """设置目标点 (x,y)"""
        self.pid_x.setpoint = target_point[0]
        self.pid_y.setpoint = target_point[1]

    def compute_velocity(self, x, y):
        """ 输入当前位置 (x,y)，输出期望速度 (vx,vy) """
        vx = self.pid_x(x) 
        vy = self.pid_y(y) 
        return vx, vy

    def get_err(self, x, y):
        error_x = x - self.pid_x.setpoint
        error_y = y - self.pid_y.setpoint
        return error_x, error_y
