import time
import cv2
import numpy as np
from collections import deque
from openvino.runtime import Core
from filterpy.kalman import KalmanFilter

class FaceDetector:
    def __init__(self, model_path="yolov11s-face.xml", device="GPU", show=False):
        self.ie = Core()
        self.model = self.ie.read_model(model_path)
        self.compiled_model = self.ie.compile_model(self.model, device)
        self.input_layer = self.compiled_model.input(0)
        self.output_layer = self.compiled_model.output(0)
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.fps_queue = deque(maxlen=10)
        self.show = show
        # 初始化 Kalman Filter，状态维度为 6（位置 + 速度），观测维度为 3（只观测位置）
        self.kf = KalmanFilter(dim_x=6, dim_z=3)

        # 初始状态向量：[cx, cy, area, cx速度, cy速度, area变化率]
        self.kf.x = np.array([0, 0, 0, 0, 0, 0])

        # 状态转移矩阵 F（6x6）:
        # 描述状态如何从前一时刻变化到当前时刻（简单匀速模型）
        # 例如：cx_new = cx_old + cx_vel
        self.kf.F = np.array([
            [1, 0, 0, 1, 0, 0],  # cx = cx + cx'
            [0, 1, 0, 0, 1, 0],  # cy = cy + cy'
            [0, 0, 1, 0, 0, 1],  # area = area + area'
            [0, 0, 0, 1, 0, 0],  # cx' 保持不变
            [0, 0, 0, 0, 1, 0],  # cy' 保持不变
            [0, 0, 0, 0, 0, 1]  # area' 保持不变
        ])

        # 观测矩阵 H（3x6）:
        # 描述如何从状态变量中得到观测值
        # 我们只能观测 [cx, cy, area]，不能直接测量速度
        self.kf.H = np.array([
            [1, 0, 0, 0, 0, 0],  # 只观测 cx
            [0, 1, 0, 0, 0, 0],  # 只观测 cy
            [0, 0, 1, 0, 0, 0]  # 只观测 area
        ])

        # 状态协方差矩阵 P（初始化为较大值，表示我们对初始状态的不确定性较高）
        # 越大表示初始状态可信度低，滤波器将更多依赖观测
        self.kf.P *= 100.

        # 观测噪声协方差 R（越大表示观测值噪声越大、不可信）
        self.kf.R *= 0.1

        # 过程噪声协方差 Q（控制系统对状态变化的预测不确定度）
        # 越大越相信系统会有“自身变化”，越小则更保守
        # 设为较小值，适合静态或缓慢变化的跟踪目标
        self.kf.Q *= 0.01

    def non_max_suppression(self, predictions, conf_thres=0.5, iou_thres=0.5):
        boxes = predictions[:, :4].tolist()
        scores = predictions[:, 4].tolist()
        indices = cv2.dnn.NMSBoxes(boxes, scores, conf_thres, iou_thres)
        if len(indices) == 0:
            return []
        return predictions[indices.flatten()]

    def run(self, url):
        cap = None
        try:
            # 尝试打开视频流
            cap = cv2.VideoCapture(url)
            if not cap.isOpened():
                raise RuntimeError("Cannot open camera or stream")
        except Exception as e:
            print(f"Error opening video stream: {e}")
            return

        while cap.isOpened():
            # 先清空缓存帧（最多读 N 次）
            for _ in range(5):  # N 可调，越大越新，越小越稳
                cap.grab()
            success, frame = cap.read()
            if not success:
                break

            start = time.time()
            h, w = frame.shape[:2]
            scale_x = w / 640
            scale_y = h / 640

            resized = cv2.resize(frame, (640, 640))
            input_tensor = resized.transpose(2, 0, 1)[np.newaxis].astype(np.float32) / 255.0

            raw_output = self.compiled_model([input_tensor])[self.output_layer]
            raw_output = np.squeeze(raw_output).T

            if raw_output.shape[1] == 5:
                cls = np.zeros((raw_output.shape[0], 1))
                raw_output = np.hstack([raw_output, cls])

            cx = raw_output[:, 0]
            cy = raw_output[:, 1]
            w_box = raw_output[:, 2]
            h_box = raw_output[:, 3]
            conf = raw_output[:, 4]
            cls = raw_output[:, 5]

            x1 = cx - w_box / 2
            y1 = cy - h_box / 2
            x2 = cx + w_box / 2
            y2 = cy + h_box / 2

            decoded_boxes = np.stack([x1, y1, x2, y2, conf, cls], axis=1)
            filtered = decoded_boxes[decoded_boxes[:, 4] > 0.5]
            final_boxes = self.non_max_suppression(filtered, conf_thres=0.5, iou_thres=0.4)

            output_list = []
            annotated_frame = frame.copy()

            # === 遍历检测结果，计算中心点和面积 ===
            for det in final_boxes:
                x1, y1, x2, y2, conf, cls = det
                x1, x2 = sorted([x1, x2])
                y1, y2 = sorted([y1, y2])

                x1 = int(x1 * scale_x)
                x2 = int(x2 * scale_x)
                y1 = int(y1 * scale_y)
                y2 = int(y2 * scale_y)

                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2
                area = (x2 - x1) * (y2 - y1)
                output_list.append((cx, cy, area))

                if self.show:
                    cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 128, 255), 2)
                    cv2.circle(annotated_frame, (cx, cy), 4, (0, 255, 0), -1)
                    cv2.putText(annotated_frame, f"({cx},{cy})", (x1, max(y2 + 15, 15)), self.font, 0.5, (255, 0, 0), 1)
                    cv2.putText(annotated_frame, f"A={area}", (x1, y2 + 35), self.font, 0.5, (255, 0, 0), 1)

            # === Kalman 滤波处理（放在循环后）===
            print()  # 空一行便于视觉区分
            if len(output_list) > 0:
                z = np.array(output_list[0])  # 只处理第一个目标
                print(f"[原始观测] cx={z[0]}, cy={z[1]}, area={z[2]}")
                self.kf.predict()
                self.kf.update(z)
                filtered_cx, filtered_cy, filtered_area = self.kf.x[:3]
                output_list[0] = (int(filtered_cx), int(filtered_cy), int(filtered_area))  # 替换为滤波值
            else:
                self.kf.predict()
                filtered_cx, filtered_cy, filtered_area = self.kf.x[:3]
                output_list.append((int(filtered_cx), int(filtered_cy), int(filtered_area)))  # 用预测值填补

            # === FPS 显示 ===
            end = time.time()
            fps = 1.0 / (end - start)
            self.fps_queue.append(fps)
            avg_fps = sum(self.fps_queue) / len(self.fps_queue)

            if self.show:
                cv2.putText(annotated_frame, f"FPS: {fps:.2f}", (10, 30), self.font, 0.5, (0, 255, 0), 2)
                cv2.putText(annotated_frame, f"Avg: {avg_fps:.2f}", (10, 60), self.font, 0.5, (0, 255, 255), 2)
                cv2.imshow("OpenVINO YOLOv11 Face Detection", annotated_frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

            yield output_list  # 返回滤波后结果

        cap.release()
        cv2.destroyAllWindows()
