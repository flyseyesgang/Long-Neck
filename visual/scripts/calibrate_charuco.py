# Final OpenCV 4.11-compatible ChArUco calibration script (fully working)

import yaml
import os
import argparse
import numpy as np
import cv2

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--device", default="/dev/video3")
    parser.add_argument("-n", "--num-frames", type=int, default=30)
    args = parser.parse_args()

    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera {args.device}")

    # ==== BOARD SETUP ====
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_1000)
    board = cv2.aruco.CharucoBoard(
        (8, 8),
        squareLength=0.0185,  # 18.5 mm
        markerLength=0.013,   # 13 mm
        dictionary=dictionary
    )

    detector_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, detector_params)
    charucodetector = cv2.aruco.CharucoDetector(board)

    print(f"📷 Capturing {args.num_frames} frames. SPACE=save, ESC=quit.")
    captured = []
    while len(captured) < args.num_frames:
        ret, frame = cap.read()
        if not ret:
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)
        vis = frame.copy()
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(vis, corners, ids)
        cv2.imshow("Charuco Calibration", vis)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            break
        elif key == 32 and ids is not None and len(ids) >= 4:
            captured.append(frame.copy())
            print(f"  → Saved frame {len(captured)}/{args.num_frames}")
    cap.release()
    cv2.destroyAllWindows()

    # ==== CHARUCO CORNER DETECTION ====
    all_corners = []
    all_ids = []

    for i, img in enumerate(captured):
        result = charucodetector.detectBoard(img)

        if result is not None and len(result) >= 3:
            charuco_corners = result[1]
            charuco_ids = result[2]
            if charuco_corners is not None and charuco_ids is not None and len(charuco_ids) > 3:
                all_corners.append(charuco_corners)
                all_ids.append(charuco_ids)

                vis = img.copy()
                cv2.aruco.drawDetectedCornersCharuco(vis, charuco_corners, charuco_ids)
                cv2.imshow("Charuco Interpolation Debug", vis)
                cv2.waitKey(100)

    print(f"📈 Using {len(all_corners)} valid frames.")
    if len(all_corners) < 4:
        print("❌ Not enough valid detections — check lighting and angles.")
        return

    # ==== CALIBRATION ====
    ret, K, dist, _, _ = cv2.aruco.calibrateCameraCharuco(
        charucoCorners=all_corners,
        charucoIds=all_ids,
        board=board,
        imageSize=captured[0].shape[1::-1],
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
