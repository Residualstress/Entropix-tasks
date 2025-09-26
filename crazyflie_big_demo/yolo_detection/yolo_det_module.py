import cv2
import threading
import requests
import time
import torch
from ultralytics import YOLO

class YoloDetect():
    def __init__(self, model_path="yolo11n.pt" ,cls_name_wanted ='teddy bear', draw=True):
        model = YOLO(model_path)
        model.fuse()  # fuse conv + bn (可选)
        self.model = model
        self.cls_name_wanted = cls_name_wanted
        self.draw = draw

    def detect(self, frame) -> tuple:
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.model.predict(source=frame,
                                    conf=0.25,
                                    iou=0.45,
                                    imgsz=320,
                                    max_det=10,
                                    verbose=False)  # 不打印过多日志
        # results 是列表，单张图像就取 results[0]
        r = results[0]
        # r.boxes.xyxy, r.boxes.conf, r.boxes.cls
        if r.boxes is not None and len(r.boxes) > 0:
            for box in r.boxes:
                # box.xyxy -> tensor [x1,y1,x2,y2]; box.conf, box.cls
                xyxy = box.xyxy[0].cpu().numpy().astype(int)
                conf = float(box.conf[0].cpu().numpy())
                cls_id = int(box.cls[0].cpu().numpy())
                # 获取类别名（如果模型带了 names）
                try:
                    cls_name = self.model.model.names[cls_id]
                except Exception:
                    cls_name = str(cls_id)

                if cls_name != self.cls_name_wanted:
                    continue

                x1, y1, x2, y2 = xyxy
                box_area = (x2 - x1) * (y2 - y1)
                label = f"{cls_name} {conf:.2f}"
                if self.draw is True:
                    # 画框和标签
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    # 文本背景
                    (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                    cv2.rectangle(frame, (x1, y1 - th - 6), (x1 + tw, y1), (0, 255, 0), -1)
                    cv2.putText(frame, label, (x1, y1 - 4),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
                
                return (int((x1+x2)//2), int((y1+y2)//2)), box_area


class HttpPetDetection:
    def __init__(self, ip, callback, resolution_config=5, display=True):
        self.stream_url = f'http://{ip}:81/stream'
        self.stop_event = threading.Event()
        self.cap = None
        self.display = display
        self.callback = callback  # 识别结果回调函数
        self.resolution_config = resolution_config
        self.resolver = YoloDetect(model_path="yolo11n.pt", cls_name_wanted='teddy bear', draw=True)

        # 设置分辨率
        requests.get(f'http://{ip}/control?var=framesize&val={resolution_config}')
        requests.get(f'http://{ip}/control?var=quality&val=20') # quality 50
        requests.get(f'http://{ip}/control?var=ae_level&val=-1') # ev -1
        requests.get(f'http://{ip}/control?var=special_effect&val=0') # color

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
            # frame = cv2.flip(frame, 0)
            # 处理帧
            center = self.resolver.detect(frame) # data is center

            # 回调返回结果
            if self.callback is not None:
                self.callback(center)

            if self.display:
                cv2.imshow("Pet Detection", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.stop()

    def stop(self):
        self.stop_event.set()
        if self.cap:
            self.cap.release()
        if self.display:
            cv2.destroyAllWindows()
        print("视频识别已停止")

def cb(data):
    print(data)

if __name__ == "__main__":
    # main()
    HttpPetDetection('172.20.10.11', cb, display=True).start()
    while True:
        time.sleep(1)
