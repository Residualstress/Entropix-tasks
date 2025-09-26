import time
import cv2
import numpy as np
from collections import deque
from openvino.runtime import Core

def non_max_suppression(predictions, conf_thres=0.5, iou_thres=0.5):
    """基于 OpenCV 的 NMS"""
    boxes = predictions[:, :4].tolist()
    scores = predictions[:, 4].tolist()

    indices = cv2.dnn.NMSBoxes(boxes, scores, conf_thres, iou_thres)
    if len(indices) == 0:
        return []

    return predictions[indices.flatten()]


# -------------------------------
# 1. 初始化 OpenVINO 推理引擎
# -------------------------------
ie = Core()
model_ir_path = "yolov11s-face.xml"
model = ie.read_model(model=model_ir_path)
compiled_model = ie.compile_model(model=model, device_name="GPU")

input_layer = compiled_model.input(0)
output_layer = compiled_model.output(0)
font = cv2.FONT_HERSHEY_SIMPLEX
# -------------------------------
# 2. 摄像头与FPS初始化
# -------------------------------
url = "http://10.201.171.127:8088/mjpeg1"
#url = "http://10.201.171.123:81/stream"
cap = cv2.VideoCapture(url)
# cap = cv2.VideoCapture(0)
if not cap.isOpened():
    raise RuntimeError("Cannot open camera")
fps_queue = deque(maxlen=10)

# -------------------------------
# 3. 主循环
# -------------------------------
while cap.isOpened():
    # 先清空缓存帧（最多读 N 次）
    for _ in range(5):  # N 可调，越大越新，越小越稳
        cap.grab()

    success, frame = cap.read()
    if not success:
        break

    start = time.time()
    scale_x = frame.shape[1] / 640  # 1920 / 640 = 3.0
    scale_y = frame.shape[0] / 640  # 1080 / 640 = 1.6875
    # 预处理：Resize + 归一化 + NCHW
    resized = cv2.resize(frame, (640, 640))
    input_tensor = resized.transpose(2, 0, 1)[np.newaxis, ...].astype(np.float32) / 255.0

    # 推理输出 (1, 6, N) → (N, 6)
    raw_output = compiled_model([input_tensor])[output_layer]
    raw_output = np.squeeze(raw_output).T  # → (N, 5) or (N, 6)

    # 如果只有5列，补类别列为0
    if raw_output.shape[1] == 5:
        cls = np.zeros((raw_output.shape[0], 1))
        raw_output = np.hstack([raw_output, cls])

    cx = raw_output[:, 0]
    cy = raw_output[:, 1]
    w = raw_output[:, 2]
    h = raw_output[:, 3]
    conf = raw_output[:, 4]
    cls = raw_output[:, 5]

    # 解码为左上角和右下角坐标
    x1 = cx - w / 2
    y1 = cy - h / 2
    x2 = cx + w / 2
    y2 = cy + h / 2

    decoded_boxes = np.stack([x1, y1, x2, y2, conf, cls], axis=1)

    # 置信度过滤
    filtered = decoded_boxes[decoded_boxes[:, 4] > 0.5]

    # NMS
    final_boxes = non_max_suppression(filtered, conf_thres=0.5, iou_thres=0.4)

    # if len(final_boxes) > 0:
    #     print("🧠 Sample Box:", final_boxes[0])
    # -------------------------------
    # 4. 可视化
    # -------------------------------
    annotated_frame = frame.copy()

    for det in final_boxes:
        x1, y1, x2, y2, conf, cls = det

        # 修正坐标：确保左上角和右下角顺序
        x1, x2 = sorted([x1, x2])
        y1, y2 = sorted([y1, y2])

        # 缩放框坐标到原图尺寸
        x1 *= scale_x
        x2 *= scale_x
        y1 *= scale_y
        y2 *= scale_y

        x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])
        cx = (x1 + x2) // 2 #200
        cy = (y1 + y2) // 2 #50
        area = (x2 - x1) * (y2 - y1) #60000时悬停
        # print(f"📍 Detected Box - cx: {cx}, cy: {cy}, area: {area}")
        # ✅ 正确绘制：坐标顺序应为 (x, y)，即 (列, 行)
        cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 128, 255), 2)
        cv2.circle(annotated_frame, (cx, cy), 4, (0, 255, 0), -1)

        # # 文本信息
        # font = cv2.FONT_HERSHEY_SIMPLEX
        # cv2.putText(annotated_frame, f"({cx},{cy})", (x1, max(y2 + 15, 15)),
        #             font, 0.5, (255, 0, 0), 1)
        # cv2.putText(annotated_frame, f"A={area}", (x1, y2 + 35),
        #             font, 0.5, (255, 0, 0), 1)

    # FPS 计算
    end = time.time()
    fps = 1.0 / (end - start)
    fps_queue.append(fps)
    avg_fps = sum(fps_queue) / len(fps_queue)

    # cv2.putText(annotated_frame, f"FPS: {fps:.2f}", (10, 30),
    #             font, 0.5, (0, 255, 0), 2)
    # cv2.putText(annotated_frame, f"Avg: {avg_fps:.2f}", (10, 60),
    #             font, 0.5, (0, 255, 255), 2)

    # 显示图像
    cv2.imshow("OpenVINO YOLOv8 Inference", annotated_frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# 清理资源
cap.release()
cv2.destroyAllWindows()