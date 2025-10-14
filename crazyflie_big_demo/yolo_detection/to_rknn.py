from ultralytics import YOLO

# Load your YOLO model
model = YOLO("yolo11n.pt")

# Export to RKNN format for a specific Rockchip platform
model.export(format="rknn", imgsz=320, name="rk3588")