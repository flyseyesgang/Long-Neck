# view_labels.py  — show random labelled images
import argparse, random, cv2
from pathlib import Path

p = argparse.ArgumentParser()
p.add_argument("--dataset-dir", default=None, help="root of dataset/")
p.add_argument("--split", choices=["train", "val"], default="train")
p.add_argument("--num", type=int, default=5)
args = p.parse_args()

root  = Path(args.dataset_dir) if args.dataset_dir else Path(__file__).parent
imgs_dir   = root / "images"  / args.split
labels_dir = root / "labels"  / args.split

imgs = sorted(imgs_dir.glob("*.png"))
if not imgs:
    raise SystemExit(f"[ERROR] No .png files found in {imgs_dir}")

for img_path in random.sample(imgs, min(args.num, len(imgs))):
    lbl_path = labels_dir / (img_path.stem + ".txt")
    img = cv2.imread(str(img_path))

    if lbl_path.exists():
        for line in lbl_path.read_text().splitlines():
            _, xc, yc, w, h = map(float, line.split())
            H, W = img.shape[:2]
            x1 = int((xc - w/2) * W)
            y1 = int((yc - h/2) * H)
            x2 = int((xc + w/2) * W)
            y2 = int((yc + h/2) * H)
            cv2.rectangle(img, (x1,y1), (x2,y2), (0,255,0), 2)
    cv2.imshow(str(img_path.name), img)
    cv2.waitKey(0)

cv2.destroyAllWindows()
