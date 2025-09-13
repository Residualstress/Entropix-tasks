import cv2
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

def process_frame(frame_recv):
    frame = frame_recv.copy()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, thres = cv2.threshold(gray, 40, 255, cv2.THRESH_BINARY)
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

            rect = cv2.minAreaRect(best_rect)
            (cx, cy), (w, h), angle = rect

            # 在图像上标注信息
            cv2.putText(frame, f"Center: {center}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.putText(frame, f"Angle: {angle:.2f}", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

    return frame

# ---------------- 主流程 ----------------
cap = cv2.VideoCapture('ir_beacon_resolve/media/2.mp4')  # 改成你的摄像头索引或视频路径

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    frame_processed = process_frame(frame)
    cv2.imshow('Original', frame)
    cv2.imshow('IR Beacon Detection', frame_processed)
    if cv2.waitKey(20) & 0xFF == ord('q'):  # 按 q 退出
        break

cap.release()
cv2.destroyAllWindows()
