#camera_calib_cli_node.py

import os
import cv2
import time
import queue
import threading
import numpy as np
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# ArUco 관련 모듈 추가
import cv2.aruco as aruco

class CameraCalibCliNode(Node):
    def __init__(self):
        super().__init__("camera_calib_cli_node")

        # [수정] 보드 사양에 맞춘 기본값 변경
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("save_dir", "./calib_images")
        self.declare_parameter("output_yaml", "./camera_intrinsics.yaml")
        self.declare_parameter("output_npz", "./camera_intrinsics.npz")
        
        # 6x11 보드는 가로 사각형 11개, 세로 사각형 6개이므로 
        # OpenCV 내부 코너 기준은 가로 10개, 세로 5개입니다.
        self.declare_parameter("cols", 6)              # 전체 사각형 열 개수
        self.declare_parameter("rows", 11)               # 전체 사각형 행 개수
        self.declare_parameter("square_size", 0.05)     # 50mm -> 0.05m
        self.declare_parameter("marker_size", 0.037)    # 37mm -> 0.037m
        self.declare_parameter("preview", True)

        self.params = aruco.DetectorParameters()

        # 마커 경계선을 더 정밀하게 찾기 위한 설정
        self.params.adaptiveThreshWinSizeMin = 3
        self.params.adaptiveThreshWinSizeMax = 23
        self.params.adaptiveThreshWinSizeStep = 10
        # 마커가 작게 보일 때를 대비한 최소 크기 설정 (0.03 = 화면의 3% 이상)
        self.params.minMarkerPerimeterRate = 0.03 

        # detector 생성 시 파라미터 적용
        self.detector = aruco.CharucoDetector(self.board, detectorParams=self.params)

        # 파라미터 로드
        image_topic = str(self.get_parameter("image_topic").value)
        self.save_dir = str(self.get_parameter("save_dir").value)
        self.output_yaml = str(self.get_parameter("output_yaml").value)
        self.output_npz = str(self.get_parameter("output_npz").value)
        self.cols = int(self.get_parameter("cols").value)
        self.rows = int(self.get_parameter("rows").value)
        self.square_size = float(self.get_parameter("square_size").value)
        self.marker_size = float(self.get_parameter("marker_size").value)
        self.preview = bool(self.get_parameter("preview").value)

        # [추가] Charuco 보드 및 Detector 설정
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.board = aruco.CharucoBoard(
            (self.cols, self.rows), 
            self.square_size, 
            self.marker_size, 
            self.dictionary
        )
        self.detector = aruco.CharucoDetector(self.board)

        # 디버그 폴더 생성 로직 (기존과 동일)
        Path(self.save_dir).mkdir(parents=True, exist_ok=True)
        self.debug_corners_dir = Path(self.save_dir) / "debug_corners"
        self.debug_corners_dir.mkdir(parents=True, exist_ok=True)

        self.bridge = CvBridge()
        self.latest_frame = None
        self.lock = threading.Lock()
        self.cmd_queue = queue.Queue()

        self.sub = self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.input_thread = threading.Thread(target=self.stdin_loop, daemon=True)
        self.input_thread.start()

        self.get_logger().info(f"Charuco Calib Node Started: {self.cols}x{self.rows}")
        self.print_help()

    def print_help(self):
        print("\n================ Charuco Calibration CLI ================")
        print("s : save current frame (if Charuco board is detected)")
        print("c : run calibration from saved images")
        print("q : quit")
        print("========================================================\n")

    def stdin_loop(self):
        while True:
            try:
                cmd = input("> ").strip().lower()
                self.cmd_queue.put(cmd)
                if cmd == "q": break
            except: break

    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        with self.lock:
            self.latest_frame = frame.copy()

    def find_charuco(self, gray):
        """Charuco 보드 감지 핵심 로직"""
        charuco_corners, charuco_ids, marker_corners, marker_ids = self.detector.detectBoard(gray)
        
        # 코너가 최소 4개 이상 발견되어야 유효함
        if charuco_ids is not None and len(charuco_ids) >= 4:
            return True, charuco_corners, charuco_ids
        return False, None, None

    def save_current_frame(self):
        with self.lock:
            if self.latest_frame is None: return
            frame = self.latest_frame.copy()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, _, _ = self.find_charuco(gray)

        if not found:
            self.get_logger().warning("Charuco board NOT detected, frame not saved")
            return

        idx = len(list(Path(self.save_dir).glob("*.jpg")))
        out_path = os.path.join(self.save_dir, f"calib_{idx:03d}.jpg")
        cv2.imwrite(out_path, frame)
        self.get_logger().info(f"saved: {out_path}")

    def run_calibration(self):
        image_paths = sorted([str(p) for p in Path(self.save_dir).glob("*.jpg")])
        if not image_paths:
            self.get_logger().warning("no images found")
            return

        all_charuco_corners = []
        all_charuco_ids = []
        image_size = None

        self.get_logger().info("Processing images...")
        for path in image_paths:
            img = cv2.imread(path)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            if image_size is None: image_size = gray.shape[::-1]

            found, corners, ids = self.find_charuco(gray)
            if found:
                all_charuco_corners.append(corners)
                all_charuco_ids.append(ids)
                
                # 디버그 이미지 저장
                vis = img.copy()
                aruco.drawDetectedCornersCharuco(vis, corners, ids)
                cv2.imwrite(str(self.debug_corners_dir / Path(path).name), vis)
                self.get_logger().info(f"[OK] {path}")

        if len(all_charuco_corners) < 10:
            self.get_logger().warning(f"Not enough valid images ({len(all_charuco_corners)})")
            return

        # [수정] Charuco 전용 캘리브레이션 함수 사용
        rms, camera_matrix, dist_coeffs, rvecs, tvecs = aruco.calibrateCameraCharuco(
            charucoCorners=all_charuco_corners,
            charucoIds=all_charuco_ids,
            board=self.board,
            imageSize=image_size,
            cameraMatrix=None,
            distCoeffs=None
        )

        # YAML 저장
        fs = cv2.FileStorage(self.output_yaml, cv2.FILE_STORAGE_WRITE)
        fs.write("image_width", image_size[0])
        fs.write("image_height", image_size[1])
        fs.write("camera_matrix", camera_matrix)
        fs.write("dist_coeffs", dist_coeffs)
        fs.write("rms", float(rms))
        fs.release()

        self.get_logger().info(f"Calibration Done! RMS: {rms:.6f}")
        self.get_logger().info(f"Saved to: {self.output_yaml}")

    def timer_callback(self):
        while not self.cmd_queue.empty():
            cmd = self.cmd_queue.get()
            if cmd == "s": self.save_current_frame()
            elif cmd == "c": self.run_calibration()
            elif cmd == "q": rclpy.shutdown(); return

        if self.preview:
            with self.lock:
                frame = self.latest_frame.copy() if self.latest_frame is not None else None
            
            if frame is not None:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                found, corners, ids = self.find_charuco(gray)
                
                vis = frame.copy()
                if found:
                    aruco.drawDetectedCornersCharuco(vis, corners, ids)
                    status = "Charuco Detected"
                    color = (0, 255, 0)
                else:
                    status = "NOT Detected"
                    color = (0, 0, 255)
                
                cv2.putText(vis, status, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.2, color, 2)
                cv2.imshow("Charuco_Calib_Preview", vis)
                cv2.waitKey(1)

def main():
    rclpy.init()
    node = CameraCalibCliNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == "__main__":
    main()