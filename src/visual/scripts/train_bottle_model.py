#!/usr/bin/env python3
import os
import cv2
import csv
import json
import random
import subprocess
import argparse
from copy import deepcopy

# --- Reward weights ---
TP_WEIGHT  = 10   # correct bottle detected
FN_WEIGHT  = -5   # missed bottle
FP_WEIGHT  = -2   # extra false detection

# --- Default Hough params (seed) ---
DEFAULT_PARAMS = {
    "dp":        1.2,
    "minDist":   None,   # computed from first image
    "param1":    200,
    "param2":    20,
    "minRadius": 8,
    "maxRadius": 40,
}

def load_truth(csv_path):
    truth = {}
    with open(csv_path) as f:
        reader = csv.reader(f)
        header = next(reader)  # skip header
        for fn, cnt in reader:
            fn = fn.strip()
            if '.' not in fn:
                fn += ".png"
            truth[fn] = int(cnt)
    return truth

def compute_default_minDist(img_path):
    gray = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
    if gray is None:
        return 30.0
    return max(1.0, gray.shape[0] / 16.0)

def run_detector(bin_path, img_path, params):
    cmd = [
        bin_path,
        "--image", img_path,
        "--dp",        f"{params['dp']:.3f}",
        "--minDist",   f"{params['minDist']:.3f}",
        "--param1",    str(params['param1']),
        "--param2",    str(params['param2']),
        "--minRadius", str(params['minRadius']),
        "--maxRadius", str(params['maxRadius']),
    ]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        return 0
    data = json.loads(proc.stdout)
    return int(data.get("count", 0))

def score_params(bin_path, truth, image_dir, params):
    # shuffle order each time
    items = list(truth.items())
    random.shuffle(items)
    total_score = 0
    for fn, true_cnt in items:
        img = os.path.join(image_dir, fn)
        pred_cnt = run_detector(bin_path, img, params)
        tp = min(pred_cnt, true_cnt)
        fn_ = max(0, true_cnt - pred_cnt)
        fp = max(0, pred_cnt - true_cnt)
        total_score += (tp * TP_WEIGHT) + (fn_ * FN_WEIGHT) + (fp * FP_WEIGHT)
    return total_score

def perturb(params):
    q = deepcopy(params)
    # Gaussian perturb, clipped to sensible ranges
    q["dp"]        = max(0.1,   random.gauss(q["dp"],       0.2))
    q["minDist"]   = max(1.0,   random.gauss(q["minDist"],  5.0))
    q["param1"]    = max(10,    int(random.gauss(q["param1"],20)))
    q["param2"]    = max(1,     int(random.gauss(q["param2"],10)))
    q["minRadius"] = max(1,     int(random.gauss(q["minRadius"],5)))
    # ensure maxRadius > minRadius
    q["maxRadius"] = max(q["minRadius"]+1,
                         int(random.gauss(q["maxRadius"],10)))
    return q

def main():
    p = argparse.ArgumentParser(
        description="Hill‑climb Hough params on your bottle‑lid testset"
    )
    p.add_argument("--detector",  required=True,
                   help="path to your compiled detector binary")
    p.add_argument("--image-dir", required=True,
                   help="directory with truth‐named .png files")
    p.add_argument("--truth-csv", required=True,
                   help="CSV with header File,Count")
    p.add_argument("--iters",     type=int, default=2000,
                   help="number of hill‑climb iterations")
    p.add_argument("--out",       default="best_params.json",
                   help="where to dump the best params")
    args = p.parse_args()

    # load
    truth = load_truth(args.truth_csv)
    # compute default minDist from first sample
    first_img = next(iter(truth))
    DEFAULT_PARAMS["minDist"] = compute_default_minDist(
        os.path.join(args.image_dir, first_img)
    )

    # score the seed
    best = {
        "params": deepcopy(DEFAULT_PARAMS),
        "score":  score_params(args.detector, truth,
                               args.image_dir, DEFAULT_PARAMS)
    }
    print(f"[seed] score={best['score']:.1f}, params={best['params']}")

    # hill‑climbing
    for i in range(1, args.iters+1):
        cand = perturb(best["params"])
        sc   = score_params(args.detector, truth,
                            args.image_dir, cand)
        if sc > best["score"]:
            best = {"score": sc, "params": cand}
            print(f"[iter {i}] ↑ new best {sc:.1f}  params={cand}")
        elif i % 200 == 0:
            print(f"[iter {i}]  best={best['score']:.1f}")

    # save
    with open(args.out, "w") as f:
        json.dump(best, f, indent=2)
    print("DONE →", best)

if __name__=="__main__":
    main()
