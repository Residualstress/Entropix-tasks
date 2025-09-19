import cv2
from pupil_apriltags import Detector
import threading
import requests
import time

class AprilTagDetect():
    def __init__(self, families='tag25h9', id_wanted =3, draw=True):
        self.detector = Detector(families=families, nthreads=1)
        self.id_wanted = id_wanted
        self.draw = draw

    def detect(self, frame) -> tuple:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = self.detector.detect(gray)
        detections_wanted = [det for det in detections if det.tag_id == self.id_wanted]
        if len(detections_wanted) == 0:
            return None
        else:
            det = detections_wanted[0]
            center = tuple(det.center.astype(int))

            if self.draw is True:
                cv2.circle(frame, center, 5, (0, 0, 255), -1)

            return center

class HttpAprilResolver:
    def __init__(self, ip, callback, resolution_config=5, display=True):
        self.stream_url = f'http://{ip}:81/stream'
        self.stop_event = threading.Event()
        self.cap = None
        self.display = display
        self.callback = callback  # 识别结果回调函数
        self.resolution_config = resolution_config
        self.resolver = AprilTagDetect(families='tag25h9', id_wanted =3, draw=True)

        # 设置分辨率
        requests.get(f'http://{ip}/control?var=framesize&val={resolution_config}')
        requests.get(f'http://{ip}/control?var=quality&val=50') # quality 50
        requests.get(f'http://{ip}/control?var=ae_level&val=-1') # ev -1
        requests.get(f'http://{ip}/control?var=special_effect&val=2') # gray

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
            frame = cv2.flip(frame, 0)
            # 处理帧
            center = self.resolver.detect(frame) # data is center

            # 回调返回结果
            if self.callback is not None:
                self.callback(center)

            if self.display:
                cv2.imshow("Beacon Detection", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.stop()

    def stop(self):
        self.stop_event.set()
        if self.cap:
            self.cap.release()
        if self.display:
            cv2.destroyAllWindows()
        print("视频识别已停止")

if __name__ == "__main__":
    # main()
    HttpAprilResolver('10.201.171.4', None, display=True).start()
    while True:
        time.sleep(1)
