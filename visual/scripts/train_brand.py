#!/usr/bin/env python3
"""
train_brand.py – tiny image-classifier trainer for bottle labels

Layout expected BEFORE running:
brands_aug/
└── BrandName/
    ├── crop1_00.png
    ├── crop1_01.png
    └── …

The script will create                                      (*symlinks only*)
brands_aug/train/<ClassName>/*.png   (80 %)
brands_aug/val  /<ClassName>/*.png   (20 %)

and train a YOLO-v8 classifier (224×224), exporting
runs/classify/brand_cls/weights/best.onnx
"""
import argparse, pathlib, random, shutil, os
from ultralytics import YOLO

# ──────────────────────────────────────────────────────────────────────────
def make_split(root: pathlib.Path, val_frac=0.2):
    """build train/ and val/ symlink folders – returns the class-name list"""
    train_dir = root / "train"
    val_dir   = root / "val"
    for d in [train_dir, val_dir]:
        if d.exists(): shutil.rmtree(d)
        d.mkdir()

    classes = []
    for cls in sorted(p for p in root.iterdir()
                      if p.is_dir() and p.name not in ["train", "val"]):
        classes.append(cls.name)
        (train_dir/cls.name).mkdir()
        (val_dir  /cls.name).mkdir()

        imgs = list(cls.glob("*.png"))
        random.shuffle(imgs)
        split = int(len(imgs)*(1-val_frac))
        for p in imgs[:split]:
            os.symlink(p, train_dir/cls.name/p.name)
        for p in imgs[split:]:
            os.symlink(p, val_dir  /cls.name/p.name)
    return classes

# ──────────────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-root", required=True,
                    help="folder with brands_aug")
    ap.add_argument("--epochs",    type=int, default=60)
    ap.add_argument("--imgsz",     type=int, default=224)
    ap.add_argument("--val-frac",  type=float, default=0.2)
    args = ap.parse_args()

    root = pathlib.Path(args.data_root).resolve()
    if not root.is_dir():
        raise SystemExit(f"[ERROR] {root} is not a folder")

    names = make_split(root, args.val_frac)
    print("💡 classes =", names)

    # ─── train a tiny classifier ────────────────────────────────────────
    model = YOLO("yolov8n-cls.pt")          # tiny classifier
    model.train(
        data    = str(root),                # directory that has train/ val/
        epochs  = args.epochs,
        imgsz   = args.imgsz,
        batch   = 32,
        project = "runs/classify",
        name    = "brand_cls",
        exist_ok= True
    )

    # ─── export ONNX (fixed 1×3×224×224) ───────────────────────────────
    best_pt = "runs/classify/brand_cls/weights/best.pt"
    out     = "runs/classify/brand_cls/weights/best.onnx"
    YOLO(best_pt).export(format="onnx",
                         imgsz    = args.imgsz,
                         simplify = True,
                         dynamic  = False,
                         opset    = 12)
    print(f"\n✅  exported  {out}\n"
          f"→  copy / move it to  visual/model/brand_cls.onnx  "
          f"then  colcon build --packages-select visual")

if __name__ == "__main__":
    main()
