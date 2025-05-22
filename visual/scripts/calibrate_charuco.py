# Enhanced ChArUco calibration debug script
# Compatible with OpenCV 4.11.0+ using CharucoDetector

import os
import yaml
import argparse
import numpy as np
import cv2


def save_debug_frame(img, idx, folder="debug_frames"):
    os.makedirs(folder, exist_ok=True)
    path = os.path.join(folder, f"frame_{idx:03}.png")
    cv2.imwrite(path, img)
    print(f"  → Saved frame {idx} to {path}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--device", default="/dev/video3")
    parser.add_argument("-n", "--num-frames", type=int, default=30)
    args = parser.parse_args()

    print("[INFO] Setting up ChArUco board configuration")
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_1000)
    board = cv2.aruco.CharucoBoard((8, 8), squareLength=0.0185, markerLength=0.013, dictionary=dictionary)
    print("[INFO] Board created: 8x8, square 18.5mm, marker 13mm, DICT_6X6_1000")

    detector_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, detector_params)
    charucodetector = cv2.aruco.CharucoDetector(board)

    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera {args.device}")

    print(f"📷 Starting capture: {args.num_frames} frames. SPACE=save, ESC=quit.")
    captured = []
    frame_id = 0
    while len(captured) < args.num_frames:
        ret, frame = cap.read()
        if not ret:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)
        print(f"[DEBUG] Frame {frame_id + 1}: Detected {0 if ids is None else len(ids)} markers")

        vis = frame.copy()
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(vis, corners, ids)
        cv2.imshow("Charuco Calibration", vis)

        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            break
        elif key == 32 and ids is not None and len(ids) >= 4:
            captured.append(gray.copy())
            save_debug_frame(frame, len(captured))

        frame_id += 1

    cap.release()
    cv2.destroyAllWindows()

    print("[INFO] Beginning ChArUco interpolation...")
    all_corners = []
    all_ids = []
    for i, img in enumerate(captured):
        print(f"[DEBUG] Processing frame {i+1}")
        result = charucodetector.detectBoard(img)

        if result is None:
            print(f"[WARN] Frame {i+1}: No result returned")
            continue

        if hasattr(result, 'getCharucoCorners'):
            charuco_corners = result.getCharucoCorners()
            charuco_ids = result.getCharucoIds()
        else:
            charuco_corners = result[0]
            charuco_ids = result[1]

        if charuco_corners is not None and charuco_ids is not None and len(charuco_ids) > 3:
            print(f"[DEBUG] Frame {i+1}: Got {len(charuco_ids)} ChArUco corners")
            all_corners.append(charuco_corners)
            all_ids.append(charuco_ids)

            dbg = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            cv2.aruco.drawDetectedCornersCharuco(dbg, charuco_corners, charuco_ids)
            cv2.imshow("Charuco Interpolation Debug", dbg)
            cv2.waitKey(200)
        else:
            print(f"[WARN] Frame {i+1}: No ChArUco corners detected")

    print(f"📈 Using {len(all_corners)} valid frames for calibration")
    if len(all_corners) < 4:
        print("❌ Not enough valid detections — check print scale, lighting, or angles.")
        return

    # ==== CALIBRATION ====
    ret, K, dist, _, _ = cv2.aruco.calibrateCameraCharuco(
        charucoCorners=all_corners,
        charucoIds=all_ids,
        board=board,
        imageSize=captured[0].shape[::-1],
        cameraMatrix=None,
        distCoeffs=None,
        flags=cv2.CALIB_RATIONAL_MODEL
    )

    print(f"✅ RMS error: {ret:.4f}")
    out_dir = os.path.dirname(__file__)
    with open(os.path.join(out_dir, "camera_matrix.yaml"), "w") as f:
        yaml.dump({"camera_matrix": K.tolist()}, f)
    with open(os.path.join(out_dir, "dist_coeffs.yaml"), "w") as f:
        yaml.dump({"dist_coeff": dist.flatten().tolist()}, f)
    print("✅ Saved camera_matrix.yaml and dist_coeffs.yaml")

if __name__ == "__main__":
    main()
