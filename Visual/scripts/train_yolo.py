# scripts/train_yolo.py
#!/usr/bin/env python3
import argparse, os, yaml
from ultralytics import YOLO

def write_data(root, out="data.yaml"):
    import os
    root = os.path.abspath(root)
    d={'path':root,'train':'images/train','val':'images/val','nc':1,'names':['lid']}
    out = os.path.join(root, "data.yaml")    # write data.yaml next to your images/
    with open(out,'w') as f:
        yaml.dump(d, f)
    return out

def main():
    p=argparse.ArgumentParser()
    p.add_argument("--dataset", required=True,
                   help="root: images/train, images/val, labels/train, labels/val")
    p.add_argument("--model", default="yolov5s.pt")
    p.add_argument("--epochs", type=int, default=50)
    p.add_argument("--batch",  type=int, default=16)
    p.add_argument("--imgsz",  type=int, default=640)
    p.add_argument("--export-onnx", action="store_true")
    args=p.parse_args()

    os.chdir(os.path.dirname(args.dataset) or ".")
    dy=write_data(args.dataset)
    m=YOLO(args.model)
    m.train(data=dy,epochs=args.epochs,batch=args.batch,
            imgsz=args.imgsz,project="runs/train",
            name="bottle_lid_yolo",exist_ok=True)
    best=f"runs/train/bottle_lid_yolo/weights/best.pt"
    print("Best weights →",best)
    if args.export_onnx:
        print("Re‑loading best.pt and exporting static ONNX…")
        model = YOLO(best)
        model.export(
            format="onnx",
            opset=12,          # broadest compatibility
            simplify=True,     # onnx-simplifier
            dynamic=False,     # disable dynamic axes
            imgsz=args.imgsz   # CLI‑style override for input size
        )
        print("Exported simplified, static ONNX → runs/train/bottle_lid_yolo/weights/best.onnx")
if __name__=="__main__":
    main()
