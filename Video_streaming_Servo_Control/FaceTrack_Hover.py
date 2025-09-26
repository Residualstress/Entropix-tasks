import sys
import time
from threading import Event
import numpy as np
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper
import sys
from FaceDetection import FaceDetector  # ✅ 你的目标检测模块
import subprocess
from pid_controller import PIDController

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
# 控制目标
TARGET_CX = 160
TARGET_CY = 50
TARGET_AREA = 5000
CX_TOLERANCE = 5
CY_TOLERANCE = 5
Area_TOLERANCE = 350
Vmin = 0.1
Vmax = 0.15

pid_controller = PIDController(
    kp_x=0.00005, ki_x=0.0005, kd_x=0.0,  # 对应原始程序中的 da 系数（控制 vx）
    kp_y=0.002, ki_y=0.0005, kd_y=0.0,    # 对应原始程序中的 dx 系数（控制 vy）
    kp_z=0.0002, ki_z=0.0005, kd_z=0.0,   # 对应原始程序中的 dy 系数（控制 vz）
    min_output=-Vmax, max_output=Vmax
)

def compute_velocity(cx, cy, area):
    dx = cx - TARGET_CX
    dy = cy - TARGET_CY
    da = TARGET_AREA - area

    # 使用 PID 控制器计算 vx, vy, vz
    vel_x, vel_y, vel_z = pid_controller.compute_velocity(dx, dy, da)

    # 如果误差都在容差范围内，返回零速度
    if abs(dx) < CX_TOLERANCE and abs(dy) < CY_TOLERANCE and abs(da) < Area_TOLERANCE:
        return 0.0, 0.0, 0.0

    return vel_x, vel_y, vel_z

# def compute_velocity(cx, cy, area):
#     dx = cx - TARGET_CX
#     dy = cy - TARGET_CY
#     da = TARGET_AREA - area
#
#     vx = vy = vz = 0.0
#     if abs(dx) > CX_TOLERANCE:
#         speed_vy = np.clip(abs(dx) * 0.002, Vmin, Vmax)
#         vy = -speed_vy if dx > 0 else speed_vy  # 右移为负速度，左移为正速度
#     else:
#         vy = 0.0
#
#     if abs(dy) > CY_TOLERANCE:
#         speed_vz = np.clip(abs(dy) * 0.0002, Vmin, Vmax)
#         vz = -speed_vz if dy > 0 else speed_vz  # 下移为负速度，上移为正速度
#     else:
#         vz = 0.0
#
#     if abs(da) > Area_TOLERANCE:
#         speed_vx = np.clip(abs(da) * 0.00005, Vmin, Vmax)
#         vx = speed_vx if da > 0 else -speed_vx
#     else:
#         vx = 0.0
#
#     if abs(dx) < CX_TOLERANCE and abs(dy) < CY_TOLERANCE and abs(da) < Area_TOLERANCE:
#         return 0.0, 0.0, 0.0
#
#     return vx, vy, vz


def control_servo(angle):
    # 调用 servo_control.py 脚本，并传入角度参数
    command = f"python path_to_servo_control.py set {angle}"
    subprocess.run(command, shell=True, check=True)

def start_rebroadcast_server(ip_address):
    # 使用传入的 IP 地址动态设置 --source 参数
    command = f"python path_to_rebroadcast_server.py --source http://{ip_address}:81/stream --host 0.0.0.0 --port 8088"
    # 使用 Popen 让 rebroadcast_server.py 在后台运行
    subprocess.Popen(command, shell=True)

def face_track_control(scf, detector, url):
    with MotionCommander(scf, default_height=1.0) as mc:
        print("[已起飞] 等待视觉控制...")

        try:
            for boxes in detector.run(url):
                if not boxes:
                    continue

                cx, cy, area = boxes[0]
                print(f"[检测] cx={cx}, cy={cy}, area={area}")

                vx, vy, vz = compute_velocity(cx, cy, area)
                print(f"[控制] vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f}")

                mc.start_linear_motion(vx, vy, vz)
                time.sleep(0.1)

                if vx == vy == vz == 0.0:
                    print("[已对准且靠近目标] 降落")
                    control_servo(0)
                    mc.land()  # 自动降落
                    time.sleep(5)
                    scf.cf.commander.send_stop_setpoint()
                    return True

        except KeyboardInterrupt:
            print("[中断] 用户手动退出")
        return False  # 追踪失败或中断


if __name__ == '__main__':
    try:
        cflib.crtp.init_drivers()
        start_rebroadcast_server("10.201.171.123")#摄像头地址
        url = "http://10.201.171.127:8088/mjpeg1"#主机地址
        detector = FaceDetector(model_path="F:/BaiduSyncdisk/PhD_Project/Indoor_MAV/Entropix-tasks/Video_streaming_Servo_Control/yolov11s-face.xml", show=True)

        # 主飞控逻辑
        with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
            scf.cf.platform.send_arming_request(True)
            time.sleep(1.0)

            success = face_track_control(scf, detector, url)

            print("[程序结束] 悬停或中止，已停止运动")
    except Exception as e:
        print(f"初始化失败: {e}")
        sys.exit(1)  # 程序退出，返回非零状态码