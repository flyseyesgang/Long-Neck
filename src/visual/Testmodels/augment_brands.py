#!/usr/bin/env python3
"""
Generate many augmented label crops from one ‘master’ crop in
brands_raw/<brand>/crop1.png ➜ brands_aug/<brand>/crop1_XX.png
"""
import cv2, random, argparse
from pathlib import Path
import numpy as np

def rand_affine(img, max_rot=20, max_scale=0.10, max_shift=0.10):
    h, w = img.shape[:2]
    ang   = random.uniform(-max_rot, max_rot)
    scale = 1.0 + random.uniform(-max_scale, max_scale)
    M = cv2.getRotationMatrix2D((w/2, h/2), ang, scale)
    M[:, 2] += (random.uniform(-max_shift, max_shift) * w,
                random.uniform(-max_shift, max_shift) * h)
    return cv2.warpAffine(img, M, (w, h), borderMode=cv2.BORDER_REPLICATE)

def jitter(img, max_b=0.25, max_c=0.25):
    b = 1.0 + random.uniform(-max_b, max_b)   # brightness
    c = 1.0 + random.uniform(-max_c, max_c)   # contrast
    out = cv2.convertScaleAbs(img, alpha=c, beta=0)
    return cv2.addWeighted(out, 1, out, 0, int(255*(b-1)))

def augment_one(src, dst_dir, n=40):
    img = cv2.imread(str(src))
    for i in range(n):
        aug = rand_affine(img)
        aug = jitter(aug)
        if random.random() < 0.3:
            k = random.choice([3,5]); aug = cv2.GaussianBlur(aug,(k,k),0)
        cv2.imwrite(str(dst_dir/f"{src.stem}_{i:02d}.png"), aug)

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--raw-dir", required=True,
                   help="e.g. visual/Testmodels/brands_raw")
    p.add_argument("--out-dir", required=True,
                   help="destination e.g. visual/Testmodels/brands_aug")
    p.add_argument("--copies", type=int, default=40,
                   help="augmented images per source crop")
    args = p.parse_args()

    raw_root = Path(args.raw_dir)
    out_root = Path(args.out_dir); out_root.mkdir(parents=True, exist_ok=True)

    for brand in raw_root.iterdir():
        if not brand.is_dir(): continue
        out_dir = out_root / brand.name; out_dir.mkdir(exist_ok=True)
        for png in brand.glob("*.png"):
            augment_one(png, out_dir, args.copies)
            print(f"✓ {png.relative_to(raw_root)} → {out_dir}")

    print("Done ✅")

if __name__ == "__main__":
    main()
