from ultralytics import YOLO
model = YOLO('runs/train/bottle_lid_yolo/weights/best.onnx')
# run a quick predict on your test image:
results = model.predict(
    source='Testmodels/BottleEsky/Test1.png',
    imgsz=640,
    device='cpu',
    show=False
)
print(results)
