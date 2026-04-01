# calibrate_from_dir.py

import argparse
from pathlib import Path
import sys

import cv2
import cv2.aruco as aruco
import numpy as np


IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def create_charuco_board(cols, rows, square_size, marker_size, dictionary):
    # OpenCV version compatibility
    if hasattr(aruco, "CharucoBoard"):
        try:
            return aruco.CharucoBoard(
                (cols, rows),
                square_size,
                marker_size,
                dictionary
            )
        except Exception:
            pass

    if hasattr(aruco, "CharucoBoard_create"):
        return aruco.CharucoBoard_create(
            cols,
            rows,
            square_size,
            marker_size,
            dictionary
        )

    raise RuntimeError("CharucoBoard API not found in your OpenCV aruco module")


def create_detector_parameters():
    if hasattr(aruco, "DetectorParameters"):
        params = aruco.DetectorParameters()
    elif hasattr(aruco, "DetectorParameters_create"):
        params = aruco.DetectorParameters_create()
    else:
        raise RuntimeError("DetectorParameters API not found")

    if hasattr(params, "adaptiveThreshWinSizeMin"):
        params.adaptiveThreshWinSizeMin = 3
    if hasattr(params, "adaptiveThreshWinSizeMax"):
        params.adaptiveThreshWinSizeMax = 23
    if hasattr(params, "adaptiveThreshWinSizeStep"):
        params.adaptiveThreshWinSizeStep = 10
    if hasattr(params, "minMarkerPerimeterRate"):
        params.minMarkerPerimeterRate = 0.03
    if hasattr(params, "cornerRefinementMethod") and hasattr(aruco, "CORNER_REFINE_SUBPIX"):
        params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX

    return params


def build_detector(board, params):
    use_charuco_detector = hasattr(aruco, "CharucoDetector")
    detector = None

    if use_charuco_detector:
        try:
            detector = aruco.CharucoDetector(board, detectorParams=params)
        except TypeError:
            detector = aruco.CharucoDetector(board)

    return use_charuco_detector, detector


def find_charuco(gray, dictionary, board, params, use_charuco_detector, detector):
    if use_charuco_detector and detector is not None:
        result = detector.detectBoard(gray)

        if isinstance(result, tuple):
            if len(result) == 4:
                charuco_corners, charuco_ids, marker_corners, marker_ids = result
            elif len(result) == 3:
                charuco_corners, charuco_ids, marker_corners = result
                marker_ids = None
            else:
                charuco_corners, charuco_ids = result[0], result[1]
                marker_corners, marker_ids = None, None
        else:
            charuco_corners, charuco_ids = None, None
            marker_ids = None
    else:
        marker_corners, marker_ids, _ = aruco.detectMarkers(
            gray,
            dictionary,
            parameters=params
        )

        if marker_ids is None or len(marker_ids) == 0:
            return False, None, None, 0, 0

        interp = aruco.interpolateCornersCharuco(
            markerCorners=marker_corners,
            markerIds=marker_ids,
            image=gray,
            board=board
        )

        if interp is None or len(interp) < 3:
            return False, None, None, len(marker_ids), 0

        retval, charuco_corners, charuco_ids = interp
        if retval is None or retval <= 0 or charuco_ids is None:
            return False, None, None, len(marker_ids), 0

    n_markers = 0 if marker_ids is None else len(marker_ids)
    n_charuco = 0 if charuco_ids is None else len(charuco_ids)
    found = n_charuco > 0

    return found, charuco_corners, charuco_ids, n_markers, n_charuco


def save_yaml_opencv(output_yaml, image_size, camera_matrix, dist_coeffs, rms):
    fs = cv2.FileStorage(str(output_yaml), cv2.FILE_STORAGE_WRITE)
    fs.write("image_width", int(image_size[0]))
    fs.write("image_height", int(image_size[1]))
    fs.write("camera_matrix", camera_matrix)
    fs.write("dist_coeffs", dist_coeffs)
    fs.write("rms", float(rms))
    fs.release()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--image_dir", type=str, required=True, help="Calibration image folder path")
    parser.add_argument("--output_yaml", type=str, default="camera_intrinsics.yaml")
    parser.add_argument("--debug_dir", type=str, default=None, help="Optional debug visualization output dir")

    parser.add_argument("--cols", type=int, default=11)
    parser.add_argument("--rows", type=int, default=6)
    parser.add_argument("--square_size", type=float, default=0.05)
    parser.add_argument("--marker_size", type=float, default=0.037)

    parser.add_argument("--dict_name", type=str, default="DICT_6X6_250")
    parser.add_argument("--min_corners", type=int, default=8, help="Minimum charuco corners per image to accept")
    parser.add_argument("--min_images", type=int, default=10, help="Minimum valid images required for calibration")

    args = parser.parse_args()

    image_dir = Path(args.image_dir)
    output_yaml = Path(args.output_yaml)
    debug_dir = Path(args.debug_dir) if args.debug_dir else None

    if not image_dir.exists() or not image_dir.is_dir():
        print(f"[ERR] image_dir not found: {image_dir}")
        sys.exit(1)

    if not hasattr(aruco, args.dict_name):
        print(f"[ERR] Unknown aruco dictionary: {args.dict_name}")
        sys.exit(1)

    dictionary = aruco.getPredefinedDictionary(getattr(aruco, args.dict_name))
    board = create_charuco_board(
        args.cols,
        args.rows,
        args.square_size,
        args.marker_size,
        dictionary
    )
    params = create_detector_parameters()
    use_charuco_detector, detector = build_detector(board, params)

    image_paths = sorted([p for p in image_dir.iterdir() if p.suffix.lower() in IMG_EXTS])

    if len(image_paths) == 0:
        print(f"[ERR] no images found in: {image_dir}")
        sys.exit(1)

    if debug_dir is not None:
        debug_dir.mkdir(parents=True, exist_ok=True)

    print(f"[INFO] image_dir       : {image_dir}")
    print(f"[INFO] output_yaml    : {output_yaml}")
    print(f"[INFO] board          : {args.cols}x{args.rows}")
    print(f"[INFO] square_size    : {args.square_size}")
    print(f"[INFO] marker_size    : {args.marker_size}")
    print(f"[INFO] dictionary     : {args.dict_name}")
    print(f"[INFO] detector_api   : {'new' if use_charuco_detector else 'legacy'}")
    print(f"[INFO] num_images     : {len(image_paths)}")

    all_charuco_corners = []
    all_charuco_ids = []
    image_size = None

    ok_count = 0
    miss_count = 0

    for path in image_paths:
        img = cv2.imread(str(path))
        if img is None:
            print(f"[MISS] failed to read: {path}")
            miss_count += 1
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        if image_size is None:
            image_size = gray.shape[::-1]
        else:
            if gray.shape[::-1] != image_size:
                print(f"[MISS] size mismatch: {path} | got={gray.shape[::-1]} expected={image_size}")
                miss_count += 1
                continue

        found, corners, ids, n_markers, n_charuco = find_charuco(
            gray, dictionary, board, params, use_charuco_detector, detector
        )

        if found and n_charuco >= args.min_corners:
            all_charuco_corners.append(corners)
            all_charuco_ids.append(ids)
            ok_count += 1
            print(f"[OK]   {path.name} | markers={n_markers}, charuco={n_charuco}")

            if debug_dir is not None:
                vis = img.copy()
                try:
                    aruco.drawDetectedCornersCharuco(vis, corners, ids)
                except Exception:
                    pass
                cv2.imwrite(str(debug_dir / path.name), vis)
        else:
            miss_count += 1
            print(f"[MISS] {path.name} | markers={n_markers}, charuco={n_charuco}")

    print(f"[INFO] valid_images   : {ok_count}")
    print(f"[INFO] invalid_images : {miss_count}")

    if len(all_charuco_corners) < args.min_images:
        print(f"[ERR] not enough valid images: {len(all_charuco_corners)} < {args.min_images}")
        sys.exit(1)

    try:
        rms, camera_matrix, dist_coeffs, rvecs, tvecs = aruco.calibrateCameraCharuco(
            charucoCorners=all_charuco_corners,
            charucoIds=all_charuco_ids,
            board=board,
            imageSize=image_size,
            cameraMatrix=None,
            distCoeffs=None
        )
    except TypeError:
        rms, camera_matrix, dist_coeffs, rvecs, tvecs = aruco.calibrateCameraCharuco(
            all_charuco_corners,
            all_charuco_ids,
            board,
            image_size,
            None,
            None
        )

    output_yaml.parent.mkdir(parents=True, exist_ok=True)
    save_yaml_opencv(output_yaml, image_size, camera_matrix, dist_coeffs, rms)

    print("\n========== Calibration Result ==========")
    print(f"RMS: {rms:.6f}")
    print("camera_matrix:")
    print(camera_matrix)
    print("dist_coeffs:")
    print(dist_coeffs.reshape(-1))
    print(f"saved: {output_yaml}")
    print("=======================================\n")


if __name__ == "__main__":
    main()