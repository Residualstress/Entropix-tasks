class PID:
    def __init__(self, kp, ki, kd, min_output, max_output):
        self.kp = kp  # 比例系数
        self.ki = ki  # 积分系数
        self.kd = kd  # 微分系数
        self.min_output = min_output  # 输出的最小值
        self.max_output = max_output  # 输出的最大值

        self.previous_error = 0  # 上一次的误差
        self.integral = 0  # 积分项

    def compute(self, error):
        # 计算比例项
        p = self.kp * error

        # 计算积分项
        self.integral += error

        # 计算微分项
        d = self.kd * (error - self.previous_error)

        # PID 输出
        output = p + self.ki * self.integral + d

        # 限制输出范围
        output = max(self.min_output, min(output, self.max_output))

        # 更新上一次的误差
        self.previous_error = error

        return output


class PIDController:
    def __init__(self, kp_x, ki_x, kd_x, kp_y, ki_y, kd_y, kp_z, ki_z, kd_z, min_output, max_output):
        # 创建 PID 控制器
        self.pid_x = PID(kp_x, ki_x, kd_x, min_output, max_output)  # 控制 vx（对应 da）
        self.pid_y = PID(kp_y, ki_y, kd_y, min_output, max_output)  # 控制 vy（对应 dx）
        self.pid_z = PID(kp_z, ki_z, kd_z, min_output, max_output)  # 控制 vz（对应 dy）

    def compute_velocity(self, dx, dy, da):
        # 分别计算 vx, vy, vz
        vel_x = self.pid_x.compute(da)  # 根据面积误差计算 vx
        vel_y = self.pid_y.compute(dx)  # 根据 x 误差计算 vy
        vel_z = self.pid_z.compute(dy)  # 根据 y 误差计算 vz

        return vel_x, vel_y, vel_z