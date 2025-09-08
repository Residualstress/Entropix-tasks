import cv2
import threading
import queue
import requests

class VideoRecorder:
    def __init__(self, ip, output_file="output.avi", resolution_config=5, fps=20.0, max_queue=100):
        # resolution_config: 12-1024p, 11-720p, 8-480p, 7-360p, 5-240p
        self.stream_url = f'http://{ip}:81/stream'
        self.output_file = output_file
        self.fps = fps
        self.frame_queue = queue.Queue(maxsize=max_queue)
        self.stop_event = threading.Event()
        self.writer_thread = None
        self.cap = None
        self.out = None
        requests.request("GET", f'http://{ip}/control?var=framesize&val={resolution_config}')

        print("视频流地址:", self.stream_url)
        print(f'http://{ip}/control?var=framesize&val={resolution_config}')


    def _writer(self, width, height):
        fourcc = cv2.VideoWriter_fourcc(*'MP4V')  # 也可以改成 'mp4v'
        self.out = cv2.VideoWriter(self.output_file, fourcc, self.fps, (width, height))

        while not self.stop_event.is_set() or not self.frame_queue.empty():
            try:
                frame = self.frame_queue.get(timeout=0.1)
                self.out.write(frame)
            except queue.Empty:
                continue

        self.out.release()

    def start(self):
        self.cap = cv2.VideoCapture(self.stream_url)
        if not self.cap.isOpened():
            raise RuntimeError(f"无法打开视频流: {self.stream_url}")

        width  = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        # 启动写线程
        self.writer_thread = threading.Thread(target=self._writer, args=(width, height))
        self.writer_thread.start()

        print("开始录制，按 'q' 结束...")
        while True:
            ret, frame = self.cap.read()
            if not ret:
                print("流中断或无法读取帧")
                break

            # 放入队列（满了就丢弃，避免阻塞）
            if not self.frame_queue.full():
                self.frame_queue.put(frame)

            # 可选显示
            cv2.imshow("Stream", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.stop()

    def stop(self):
        self.stop_event.set()
        if self.cap:
            self.cap.release()
        if self.writer_thread:
            self.writer_thread.join()
        cv2.destroyAllWindows()
        print("录制结束，保存为:", self.output_file)


if __name__ == "__main__":
    ip = "192.168.31.26"  # 改成你的HTTP流地址
    # 注意：resolution_config: 12-1024p, 11-720p, 8-480p, 7-360p, 5-240p
    recorder = VideoRecorder(ip, "recorded.mp4", resolution_config=5, fps=30.0, max_queue=500)
    recorder.start()
