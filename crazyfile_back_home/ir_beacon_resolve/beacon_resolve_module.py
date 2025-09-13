import cv2
import threading
import queue
import requests
import numpy as np

def angle_between(u, v):
    cos_theta = np.dot(u, v) / (np.linalg.norm(u) * np.linalg.norm(v) + 1e-6)
    theta = np.degrees(np.arccos(np.clip(cos_theta, -1, 1)))
    return theta

def rect_score(pts):
    score = 0
    for i in range(4):
        p0 = pts[i]
        p1 = pts[(i + 1) % 4]
        p2 = pts[(i + 2) % 4]
        v1 = p1 - p0
        v2 = p2 - p1
        ang = angle_between(v1, v2)
        score += abs(90 - ang)
    return score


class BeaconResolver:
    def __init__(self, thresh=40):
        self.thresh = thresh

    def process_frame(self, frame_recv):
        frame = frame_recv.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, thres = cv2.threshold(gray, self.thresh, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thres, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        pts = []
        for cnt in contours:
            M = cv2.moments(cnt)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                pts.append([cx, cy])
                cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)

        pts = np.array(pts, dtype=np.float32)

        info = {
            "center": None,
            "led_points": None,
            "missing_point": None,
            "vec": None
        }

        if len(pts) == 3:
            A, B, C = pts
            D1 = A + B - C
            D2 = A + C - B
            D3 = B + C - A

            candidates = [
                np.array([A, B, C, D1]),
                np.array([A, B, C, D2]),
                np.array([A, B, C, D3])
            ]

            best_rect = None
            best_score = 1e9

            for cand in candidates:
                score = rect_score(cand)
                if score < best_score:
                    best_score = score
                    best_rect = cand

            if best_rect is not None:
                rect_pts_int = best_rect.astype(int).reshape((-1, 1, 2))
                cv2.polylines(frame, [rect_pts_int], True, (255, 0, 0), 2)

                center = np.mean(best_rect, axis=0).astype(int)
                cv2.circle(frame, tuple(center), 6, (0, 0, 255), -1)

                # 找到缺角点 D
                set_pts = {tuple(p) for p in best_rect.astype(int)}
                set_input = {tuple(p) for p in pts.astype(int)}
                missing = list(set_pts - set_input)[0]

                vec = np.array(missing) - np.array(center)
                vec = vec / (np.linalg.norm(vec) + 1e-6)

                # 在图像上画方向箭头
                start_pt = tuple(center)
                end_pt = tuple(missing)
                cv2.arrowedLine(frame, start_pt, end_pt, (0, 0, 255), 2, tipLength=0.3)

                # 存储结果
                info["center"] = tuple(center)
                info["led_points"] = [tuple(p) for p in pts.astype(int)]
                info["missing_point"] = tuple(missing)
                info["vec"] = vec.tolist()

                # 文字标注
                cv2.putText(frame, f"Center: {center}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        return frame, info


# ---------- 视频流 + Beacon 处理封装 -----------
class HttpBeaconResolver:
    def __init__(self, ip, callback, resolution_config=5, exposure_value=30, display=True):
        self.stream_url = f'http://{ip}:81/stream'
        self.stop_event = threading.Event()
        self.cap = None
        self.display = display
        self.callback = callback  # 识别结果回调函数
        self.resolution_config = resolution_config
        self.resolver = BeaconResolver()

        # 设置分辨率
        requests.get(f'http://{ip}/control?var=framesize&val={resolution_config}')
        # ACE DSP ON
        requests.get(f'http://{ip}/control?var=aec2&val=1')
        # ACE SENSOR OFF
        requests.get(f'http://{ip}/control?var=aec&val=0')
        # Exposure Value
        requests.get(f'http://{ip}/control?var=aec_value&val={exposure_value}')
        # AGC OFF
        requests.get(f'http://{ip}/control?var=agc&val=0')

    def start(self):
        self.cap = cv2.VideoCapture(self.stream_url)
        if not self.cap.isOpened():
            raise RuntimeError(f"无法打开视频流: {self.stream_url}")

        self.thread = threading.Thread(target=self._process_loop, daemon=True)
        self.thread.start()
        print("视频识别线程已启动...")

    def _process_loop(self):
        while not self.stop_event.is_set():
            ret, frame = self.cap.read()
            if not ret:
                continue

            # 处理帧
            # frame_proc, info = self.resolver.process_frame(frame)
            frame_proc = frame
            info = None

            # 回调返回结果
            if self.callback is not None:
                self.callback(info, frame_proc)

            if self.display:
                cv2.imshow("Beacon Detection", frame_proc)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.stop()

    def stop(self):
        self.stop_event.set()
        if self.cap:
            self.cap.release()
        if self.display:
            cv2.destroyAllWindows()
        print("视频识别已停止")

# ---------- 使用示例 ----------
if __name__ == "__main__":
    def result_callback(info, frame):
        print("识别结果:", info)
        # 可以在这里做其他处理，比如发消息或者控制无人机

    ip = "10.201.171.40"
    processor = HttpBeaconResolver(ip, result_callback, resolution_config=5, display=True)
    processor.start()

    try:
        while True:
            pass  # 主线程可以做其他事情
    except KeyboardInterrupt:
        processor.stop()
