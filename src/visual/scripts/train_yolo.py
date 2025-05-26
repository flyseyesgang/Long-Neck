#!/usr/bin/env python3
import argparse, os, yaml
from ultralytics import YOLO

def write_data(root):
    root = os.path.abspath(root)
    d = {
        'path': root,
        'train': 'images/train',
        'val':   'images/val',
        'nc':    1,
        'names': ['lid']
    }
    out = os.path.join(root, "data.yaml")
    with open(out, 'w') as f:
        yaml.dump(d, f)
    return out

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--dataset",    required=True,
                   help="Root dir with images/train, images/val, labels/train, labels/val")
    p.add_argument("--model",      default="yolov5s.pt")
    p.add_argument("--epochs",     type=int, default=50)
    p.add_argument("--batch",      type=int, default=16)
    p.add_argument("--imgsz",      type=int, default=640)
    p.add_argument("--export-onnx", action="store_true")
    args = p.parse_args()

    # ensure we run from the dataset root
    os.chdir(os.path.dirname(args.dataset) or ".")
    data_yaml = write_data(args.dataset)

    # train
    model = YOLO(args.model)
    model.train(
        data=data_yaml,
        epochs=args.epochs,
        batch=args.batch,
        imgsz=args.imgsz,
        project="runs/train",
        name="bottle_lid_yolo",
        exist_ok=True
    )

    best = "runs/train/bottle_lid_yolo/weights/best.pt"
    print("✅ Training complete. Best weights →", best)

    if args.export_onnx:
        print("🔄 Exporting ONNX…")
        YOLO(best).export(
            format="onnx",
            opset=16,
            simplify=False,
            dynamic=True,
        
        )
        print("✅ ONNX saved to runs/train/bottle_lid_yolo/weights/best.onnx")

if __name__ == "__main__":
    main()
