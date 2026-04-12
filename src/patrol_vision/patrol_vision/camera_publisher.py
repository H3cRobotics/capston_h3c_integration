# camera_publisher.py
import os
import cv2
import rclpy
import numpy as np
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from pathlib import Path

# 카메라 세팅
import subprocess

"""
auto_exposure: 1 (Manual Mode)
exposure_time_absolute: 800
gain: 0
white_balance_automatic: 0
white_balance_temperature: 4900
focus_automatic_continuous: 0
"""


class CameraPublisher(Node):

	def run_cmd(self, cmd):
		return subprocess.run(cmd, check=True, capture_output=True, text=True)

	def setup_camera(self):
		cmds = [
			["v4l2-ctl", "-d", self.video_device, "--set-ctrl=auto_exposure=3"],   # 노출 자동
			["v4l2-ctl", "-d", self.video_device, "--set-ctrl=white_balance_automatic=0"],  # WB 자동 끔
			["v4l2-ctl", "-d", self.video_device, "--set-ctrl=white_balance_temperature=4500"],  # WB 고정
			["v4l2-ctl", "-d", self.video_device, "--set-ctrl=focus_automatic_continuous=0"]  # AF 끔
		]

		for cmd in cmds:
			try:
				self.run_cmd(cmd)
				self.get_logger().info(f"ok: {' '.join(cmd)}")
			except subprocess.CalledProcessError as e:
				err = e.stderr.strip() if e.stderr else "no stderr"
				self.get_logger().warning(f"failed: {' '.join(cmd)} / {err}")

	def load_calibration(self, calib_file, width, height):
		"""
		노드 시작 시 1번만 호출.
		calibration 파일이 있고 해상도가 맞으면 remap용 map 생성.
		"""
		self.use_calibration = False
		self.map1 = None
		self.map2 = None

		if not calib_file:
			self.get_logger().info("calibration file path is empty -> raw publish")
			return

		if not os.path.exists(calib_file):
			self.get_logger().info(f"calibration file not found: {calib_file} -> raw publish")
			return

		fs = cv2.FileStorage(calib_file, cv2.FILE_STORAGE_READ)
		if not fs.isOpened():
			self.get_logger().warning(f"failed to open calibration file: {calib_file}")
			return

		calib_width = int(fs.getNode("image_width").real())
		calib_height = int(fs.getNode("image_height").real())
		camera_matrix = fs.getNode("camera_matrix").mat()
		dist_coeffs = fs.getNode("dist_coeffs").mat()
		fs.release()

		if camera_matrix is None or dist_coeffs is None:
			self.get_logger().warning("invalid calibration file contents -> raw publish")
			return

		if calib_width != int(width) or calib_height != int(height):
			self.get_logger().warning(
				f"calib size mismatch: calib={calib_width}x{calib_height}, "
				f"camera={int(width)}x{int(height)} -> raw publish"
			)
			return

		new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(
			camera_matrix,
			dist_coeffs,
			(int(width), int(height)),
			0,
			(int(width), int(height))
		)

		self.map1, self.map2 = cv2.initUndistortRectifyMap(
			camera_matrix,
			dist_coeffs,
			None,
			new_camera_matrix,
			(int(width), int(height)),
			cv2.CV_16SC2
		)

		self.use_calibration = True
		self.get_logger().info(f"loaded calibration: {calib_file}")
		self.get_logger().info("camera publisher will publish undistorted image")

	def __init__(self):
		super().__init__("camera_publisher")

		self.declare_parameter("calib_file", "./data/camera_intrinsics.yaml")
		self.declare_parameter(
            "video_device",
            "/dev/v4l/by-id/usb-046d_MX_Brio_2535ZB33GY68-video-index0"
        )
		self.video_device = str(self.get_parameter("video_device").value)
		self.calib_file = str(self.get_parameter("calib_file").value)
		self.get_logger().info(f"calib_file path : {self.calib_file}")
				
		self.publisher = self.create_publisher(
			Image,
			"/camera/color/image_raw",
			10
		)

		self.bridge = CvBridge()

		self.cap = cv2.VideoCapture(self.video_device, cv2.CAP_V4L2)
		self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
		self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
		self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
		self.cap.set(cv2.CAP_PROP_FPS, 30)
		self.setup_camera()

		width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
		height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
		fps = self.cap.get(cv2.CAP_PROP_FPS)
		self.get_logger().info(f"camera resolution: {width} x {height}")
		self.get_logger().info(f"camera fps: {fps}")

		if not self.cap.isOpened():
			raise RuntimeError("camera open failed")

		# 시작 시 1번만 calibration 파일 읽음
		self.load_calibration(self.calib_file, width, height)

		self.timer = self.create_timer(
			1.0 / 30.0,
			self.timer_callback
		)

		self.get_logger().info("camera publisher started")

	def timer_callback(self):
		ret, frame = self.cap.read()

		if not ret:
			return

		# calibration 파일이 시작 시 정상 로드된 경우에만 적용
		if self.use_calibration:
			frame = cv2.remap(frame, self.map1, self.map2, interpolation=cv2.INTER_LINEAR)

		msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
		self.publisher.publish(msg)


def main():
	rclpy.init()
	node = CameraPublisher()

	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		node.get_logger().info("KeyboardInterrupt received, shutting down.")
	finally:
		if hasattr(node, "cap") and node.cap is not None:
			node.cap.release()
		node.destroy_node()
		rclpy.shutdown()


if __name__ == "__main__":
	main()