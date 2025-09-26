# yolo_realtime.py
import time
import cv2
import torch
from ultralytics import YOLO

# -------- CONFIG --------
MODEL_PATH = "yolo11n-pet.pt"   # 改成你的模型文件名/路径
SOURCE = 0                   # 0 = 默认摄像头，或替换为视频/rtsp 链接 / 文件路径
IMG_SIZE = 320               # 输入尺寸（模型训练时常用的尺寸）
CONF_THRESH = 0.25           # 置信度阈值
IOU_THRESH = 0.25            # NMS IoU 阈值
USE_HALF = True              # 如果是 GPU，是否使用半精度以加速
SHOW = True                  # 是否显示窗口
SAVE_OUTPUT = False          # 是否保存带框视频
OUT_FILENAME = "out.avi"
# ------------------------

def main():
    # 选择设备
    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"[info] device: {device}")

    # 加载模型（ultralytics 会自动适配）
    model = YOLO(MODEL_PATH)
    model.fuse()  # fuse conv + bn (可选)
    # 设置模型参数
    # ultralytics 的 predict 接口支持很多参数，我们用最简单的
    if device == "cuda" and USE_HALF:
        try:
            model.model.half()  # 有时能加速推理（取决于模型/torch版本）
        except Exception:
            pass

    # 打开摄像头 / 视频
    cap = cv2.VideoCapture(SOURCE)
    if not cap.isOpened():
        raise RuntimeError(f"无法打开视频源: {SOURCE}")

    # 保存视频准备
    writer = None
    if SAVE_OUTPUT:
        fourcc = cv2.VideoWriter_fourcc(*"XVID")
        fps = cap.get(cv2.CAP_PROP_FPS) or 20.0
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        writer = cv2.VideoWriter(OUT_FILENAME, fourcc, fps, (w, h))

    prev_time = time.time()
    fps_smooth = 0.0
    try:
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                print("[info] 视频结束或无法读取帧")
                break

            t0 = time.time()
            # 转 BGR->RGB
            img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # Run prediction (ultralytics: returns a list of Results)
            # 参数: imgs, conf, iou, device 自动
            results = model.predict(source=img,
                                    conf=CONF_THRESH,
                                    iou=IOU_THRESH,
                                    imgsz=IMG_SIZE,
                                    max_det=300,
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
                        cls_name = model.model.names[cls_id]
                    except Exception:
                        cls_name = str(cls_id)

                    x1, y1, x2, y2 = xyxy
                    label = f"{cls_name} {conf:.2f}"
                    # 画框和标签
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    # 文本背景
                    (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                    cv2.rectangle(frame, (x1, y1 - th - 6), (x1 + tw, y1), (0, 255, 0), -1)
                    cv2.putText(frame, label, (x1, y1 - 4),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)

            # FPS 计算（平滑）
            t1 = time.time()
            dt = t1 - prev_time
            prev_time = t1
            fps_inst = 1.0 / (dt + 1e-6)
            fps_smooth = 0.9 * fps_smooth + 0.1 * fps_inst if fps_smooth else fps_inst
            cv2.putText(frame, f"FPS: {fps_smooth:.1f}", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

            # 显示
            if SHOW:
                cv2.imshow("YOLO Realtime", frame)
                key = cv2.waitKey(1)
                if key == 27 or key == ord("q"):
                    print("[info] 退出按键收到")
                    break

            # 保存
            if writer is not None:
                writer.write(frame)

    finally:
        cap.release()
        if writer:
            writer.release()
        cv2.destroyAllWindows()
        print("[info] 程序结束")

if __name__ == "__main__":
    main()
