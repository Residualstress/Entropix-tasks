import cv2
import numpy as np
import threading
import queue
import requests

import time


class ChessboardDetector:
    def __init__(self, pattern_size=(9, 6)):
        """
        pattern_size: 棋盘格内角点 (列数, 行数)，比如 9x6
        """
        self.pattern_size = pattern_size
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                         30, 0.001)

    def detect(self, img):
        """
        输入：BGR 或灰度图像
        输出：center(中心点), direction(长边方向单位向量)，找不到返回 None
        """
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if len(img.shape) == 3 else img

        ret, corners = cv2.findChessboardCorners(gray, self.pattern_size, None)
        if not ret:
            return None

        # 亚像素精细化
        corners = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1), self.criteria
        )

        # 中心点（所有角点的平均）
        center = np.mean(corners.reshape(-1, 2), axis=0)

        # 长边方向：用 PCA/SVD 找主方向
        pts = corners.reshape(-1, 2) - center
        _, _, vh = np.linalg.svd(pts)
        direction = vh[0]  # 第一主成分
        direction = direction / np.linalg.norm(direction)

        # 判断长边方向（棋盘列数>行数 => 横向是长边）
        if self.pattern_size[0] < self.pattern_size[1]:
            direction = vh[1]  # 如果竖向更长，则取第二主成分

        return center, direction

    def draw_result(self, img, center, direction, scale=100):
        """
        可视化中心点和方向向量
        """
        c = tuple(map(int, center))
        d = (int(center[0] + direction[0] * scale),
             int(center[1] + direction[1] * scale))
        cv2.circle(img, c, 5, (0, 0, 255), -1)
        cv2.arrowedLine(img, c, d, (255, 0, 0), 2, tipLength=0.2)
        return img
    

class HttpChessboardDetector():
    def __init__(self, ip, pattern_size=(9, 6), callback=None,  resolution_config=5,display=False):
        self.pattern_size = pattern_size
        self.stream_url = f'http://{ip}:81/stream'
        self.stop_event = threading.Event()
        self.cap = None
        self.display = display
        self.callback = callback  # 识别结果回调函数
        self.resolution_config = resolution_config
        self.resolver = ChessboardDetector(pattern_size=pattern_size)
        # 设置分辨率
        requests.get(f'http://{ip}/control?var=framesize&val={resolution_config}')


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
            data = self.resolver.detect(frame)
            # 回调返回结果
            if self.callback is not None and data is not None:
                self.callback(data)

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


# if __name__ == "__main__":
#     img = cv2.imread("crazyfile_back_home/chessboard_beacon/media/1.png")
#     detector = ChessboardDetector(pattern_size=(9, 7))
#     result = detector.detect(img)

#     if result:
#         center, direction = result
#         print("中心点:", center)
#         print("方向向量:", direction)

#         out = detector.draw_result(img, center, direction)
#         cv2.imshow("Result", out)
#         cv2.waitKey(0)
#     else:
#         print("棋盘格未识别到")


def result_callback(data):
    if data is None:
        print("棋盘格未识别到")
        return

    center, direction = data
    print("中心点:", center)
    print("方向向量:", direction)

if __name__ == "__main__":
    detector = HttpChessboardDetector(ip="10.201.171.40", pattern_size=(5, 3), resolution_config=5, callback=result_callback ,display=True)
    detector.start()
    while True:
        time.sleep(1)