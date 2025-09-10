import logging
import sys
import time
from threading import Event, Thread
import pandas as pd
from datetime import datetime
import queue
import requests
from sync_aligner import SyncAligner

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper
from cflib.crazyflie.log import LogConfig

# =========================
# Crazyflie 基础配置
# =========================
URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
DEFAULT_HEIGHT = 0.5
deck_attached_event = Event()

logging.basicConfig(level=logging.ERROR)
log_data = []

# =========================
# 视频录制器（异步 + 懒加载cv2）
# =========================
class VideoRecorder:
    """
    异步录制 MJPEG HTTP 流到本地文件（mp4），双线程：采集线程 + 写盘线程
    - 懒加载 cv2：仅在 start() 内 import，避免无关时加载。
    - stop() 会优雅地停止所有线程并释放资源。
    """
    def __init__(self, ip, output_file="recorded.mp4", resolution_config=5, fps=20.0, max_queue=100, show_window=False):
        # resolution_config: 12-1024p, 11-720p, 8-480p, 7-360p, 5-240p (ESP32-CAM 常见档位)
        self.ip = ip
        self.stream_url = f'http://{ip}:81/stream'
        self.output_file = output_file
        self.fps = fps
        self.max_queue = max_queue
        self.show_window = show_window

        # 控制摄像头分辨率
        try:
            requests.request("GET", f'http://{ip}/control?var=framesize&val={resolution_config}', timeout=2)
        except Exception:
            pass

        self._cap = None
        self._out = None
        self._cv2 = None  # 懒加载后存放模块引用
        self._queue = queue.Queue(maxsize=max_queue)
        self._stop_event = Event()
        self._writer_thread = None
        self._reader_thread = None
        self._started = False

    def _writer(self, width, height):
        # 注意：在某些平台，mp4v 需要系统有 ffmpeg/合适的 backend
        fourcc = self._cv2.VideoWriter_fourcc(*'mp4v')
        self._out = self._cv2.VideoWriter(self.output_file, fourcc, self.fps, (width, height))
        while not self._stop_event.is_set() or not self._queue.empty():
            try:
                frame = self._queue.get(timeout=0.1)
                self._out.write(frame)
            except queue.Empty:
                continue
        self._out.release()
        self._out = None

    def _reader(self):
        # 采集线程：从 HTTP 流读取帧，放入队列（队列满则丢弃）
        width = int(self._cap.get(self._cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self._cap.get(self._cv2.CAP_PROP_FRAME_HEIGHT))

        # 启动写盘线程
        self._writer_thread = Thread(target=self._writer, args=(width, height), daemon=True)
        self._writer_thread.start()

        while not self._stop_event.is_set():
            ret, frame = self._cap.read()
            if not ret:
                # 流断开，稍等重试或直接退出（这里直接退出）
                break
            if not self._queue.full():
                self._queue.put(frame)
            if self.show_window:
                self._cv2.imshow("ESP32 Stream", frame)
                if self._cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        # 结束：停止写盘线程
        self._stop_event.set()

    def start(self):
        if self._started:
            return
        # 懒加载 cv2：仅在真正开始录制时才导入
        import cv2 as _cv2
        self._cv2 = _cv2

        self._cap = self._cv2.VideoCapture(self.stream_url)
        if not self._cap.isOpened():
            raise RuntimeError(f"无法打开视频流: {self.stream_url}")

        self._stop_event.clear()
        self._reader_thread = Thread(target=self._reader, daemon=True)
        self._reader_thread.start()
        self._started = True

    def stop(self):
        if not self._started:
            return
        # 通知 reader 退出
        self._stop_event.set()

        # 关闭采集端
        if self._reader_thread:
            self._reader_thread.join(timeout=5)
            self._reader_thread = None

        if self._cap:
            self._cap.release()
            self._cap = None

        # 等写盘线程把剩余帧写完
        if self._writer_thread:
            self._writer_thread.join(timeout=5)
            self._writer_thread = None

        # 关闭窗口
        if self._cv2 is not None and self.show_window:
            try:
                self._cv2.destroyAllWindows()
            except Exception:
                pass

        self._started = False

# =========================
# Flow deck 检测回调
# =========================
def param_deck_flow(_, value_str):
    value = int(value_str)
    print(value)
    if value:
        deck_attached_event.set()
        print('Deck is attached!')
    else:
        print('Deck is NOT attached!')

# =========================
# 日志采集
# =========================
def log_callback(timestamp, data, logconf):
    # 1) 先更新对齐器（记录首条日志时间、最近cf时间）
    aligner.update_cf_timestamp(timestamp)

    row = {"timestamp": timestamp, "group": logconf.name}
    row.update(data)
    log_data.append(row)

def start_logging(cf, video_recorder=None):
    """
    启动 Crazyflie 日志；如果传入 video_recorder，则同时启动视频录制（异步，不阻塞）
    """
    logs = []

    lg_state = LogConfig(name='State', period_in_ms=50)
    # 位置
    lg_state.add_variable('stateEstimate.x', 'float')
    lg_state.add_variable('stateEstimate.y', 'float')
    lg_state.add_variable('stateEstimate.z', 'float')
    # 速度
    lg_state.add_variable('stateEstimate.vx', 'float')
    # lg_state.add_variable('stateEstimate.vy', 'float')
    # lg_state.add_variable('stateEstimate.vz', 'float')
    # 姿态
    # lg_state.add_variable('stabilizer.roll', 'float')
    lg_state.add_variable('stabilizer.pitch', 'float')
    # lg_state.add_variable('stabilizer.yaw', 'float')

    cf.log.add_config(lg_state)
    lg_state.data_received_cb.add_callback(log_callback)
    lg_state.start()
    logs.append(lg_state)

    # ✅ 同步启动视频录制（异步线程运行，不阻塞）
    if video_recorder is not None:
        try:
            video_recorder.start()
            aligner.mark_video_start()
            print(f"[Video] Recording from {video_recorder.stream_url} -> {video_recorder.output_file}")
        except Exception as e:
            print(f"[Video] 启动失败：{e}")

    return logs

def stop_logging(log_confs, video_recorder=None):
    # 先停日志（让最后一批数据写入）
    for lg in log_confs:
        try:
            lg.stop()
        except Exception:
            pass

    # ✅ 再优雅停止视频录制（保证文件完整写盘）
    if video_recorder is not None:
        try:
            video_recorder.stop()
            aligner.mark_video_stop()
            print("[Video] 录制结束")
        except Exception as e:
            print(f"[Video] 停止失败：{e}")

def save_logs_to_excel():
    if not log_data:
        print("No log data collected")
        return
    # 注入 Marker 行
    aligner.inject_markers_to_log(log_data)
    df = pd.DataFrame(log_data)
    filename = datetime.now().strftime("crazyflie_log_%Y%m%d_%H%M%S.xlsx")
    df.to_excel(filename, index=False)
    print(f"Logs saved to {filename}")

# =========================
# 一个简单的前后直线动作
# =========================
def move_linear_simple(scf, video_recorder=None):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        # ===== 关键点：飞行前启动日志 + 视频录制（异步） =====
        log_confs = start_logging(scf.cf, video_recorder=video_recorder)
        time.sleep(1.0)
        mc.forward(0.5)
        time.sleep(3.0)

        mc.forward(0.5)
        time.sleep(3.0)

        mc.forward(0.5)
        time.sleep(1.0)

        mc.back(1.5)
        time.sleep(1.0)

        # ===== 停止日志 + 停止视频 =====
        stop_logging(log_confs, video_recorder=video_recorder)
        save_logs_to_excel()

# =========================
# 主程序
# =========================
if __name__ == '__main__':
    # 1) 初始化底层驱动
    cflib.crtp.init_drivers()

    aligner = SyncAligner(ts_unit='ms')  # 如果你的 log timestamp 是毫秒（通常是）
    # 2) 配置你摄像头的 IP（HTTP MJPEG 流）
    cam_ip = "10.201.171.40"  # ← 改成你的 ESP32 摄像头 IP
    recorder = VideoRecorder(
        ip=cam_ip,
        output_file="recorded.mp4",
        resolution_config=7,   # 240p，稳定性好；可按需 7/8/11/12
        fps=30.0,
        max_queue=100,
        show_window=False      # True 可窗口预览（注意可能拉低性能）
    )

    # 3) 连接并飞行
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        # 监听 Flow deck
        scf.cf.param.add_update_callback(group='deck', name='bcFlow2', cb=param_deck_flow)
        time.sleep(1.0)

        if not deck_attached_event.wait(timeout=5):
            print('No flow deck detected!')
            sys.exit(1)

        # 起桨（刷驱版需要；若报错可去掉）
        try:
            scf.cf.platform.send_arming_request(True)
            time.sleep(1.0)
        except Exception:
            pass

        move_linear_simple(scf, video_recorder=recorder)
