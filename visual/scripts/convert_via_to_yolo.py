#!/usr/bin/env python3
import json, os, argparse
from PIL import Image

def via_to_yolo(via_json, images_dir, labels_dir):
    data = json.load(open(via_json))
    os.makedirs(labels_dir, exist_ok=True)

    for key, item in data.items():
        fn = item.get("filename")
        img_path = os.path.join(images_dir, fn)
        if not os.path.exists(img_path):
            print(f"⚠️  Skipping {fn}, not found in {images_dir}")
            continue

        W, H = Image.open(img_path).size
        base, _ = os.path.splitext(fn)
        out_txt = os.path.join(labels_dir, base + ".txt")
        lines = []

        for region in item.get("regions", {}).values():
            sh = region["shape_attributes"]
            name = sh.get("name", "")
            if name == "circle":
                cx, cy, r = sh["cx"], sh["cy"], sh["r"]
                w, h = 2*r, 2*r
            elif name == "ellipse":
                cx, cy, rx, ry = sh["cx"], sh["cy"], sh["rx"], sh["ry"]
                w, h = 2*rx, 2*ry
            else:
                continue

            x1, y1 = cx - w/2, cy - h/2
            x_center = (x1 + w/2) / W
            y_center = (y1 + h/2) / H
            w_norm = w / W
            h_norm = h / H
            lines.append(f"0 {x_center:.6f} {y_center:.6f} {w_norm:.6f} {h_norm:.6f}")

        with open(out_txt, "w") as f:
            f.write("\n".join(lines))

    print("✅ Converted VIA annotations to YOLO format in:", labels_dir)

if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--via-json",   required=True, help="VIA annotation JSON")
    p.add_argument("--images-dir", required=True, help="Directory with .png images")
    p.add_argument("--labels-dir", required=True, help="Where to write .txt files")
    args = p.parse_args()
    via_to_yolo(args.via_json, args.images_dir, args.labels_dir)
