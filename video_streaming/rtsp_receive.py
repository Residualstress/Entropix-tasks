import cv2

# RTSP 地址示例（用户名、密码、IP、端口、流路径）
rtsp_url = "rtsp://192.168.31.210:554/mjpeg/1"

cap = cv2.VideoCapture(rtsp_url)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 20)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break
    cv2.imshow("RTSP Stream", frame)

    if cv2.waitKey(20) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
