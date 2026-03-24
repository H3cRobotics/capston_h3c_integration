#!/usr/bin/env python3

import cv2
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

#카메라 세팅
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
            ["v4l2-ctl", "-d", "/dev/video0", "--set-ctrl=auto_exposure=3"],   # 노출 자동
            ["v4l2-ctl", "-d", "/dev/video0", "--set-ctrl=white_balance_automatic=0"],  # WB 자동 끔
            ["v4l2-ctl", "-d", "/dev/video0", "--set-ctrl=white_balance_temperature=4500"],  # WB 고정
            ["v4l2-ctl", "-d", "/dev/video0", "--set-ctrl=focus_automatic_continuous=0"]  # AF 끔
        ]
        

        for cmd in cmds:
            try:
                self.run_cmd(cmd)
                self.get_logger().info(f"ok: {' '.join(cmd)}")
            except subprocess.CalledProcessError as e:
                err = e.stderr.strip() if e.stderr else "no stderr"
                self.get_logger().warning(f"failed: {' '.join(cmd)} / {err}")


    def __init__(self):
        super().__init__("camera_publisher")

        self.publisher = self.create_publisher(
            Image,
            "/camera/color/image_raw",
            10
        )

        self.bridge = CvBridge()

        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
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

        self.timer = self.create_timer(
            1.0 / 30.0,
            self.timer_callback
        )

        self.get_logger().info("camera publisher started")

    def timer_callback(self):
        ret, frame = self.cap.read()

        if not ret:
            return
        frame = cv2.flip(frame, 0) #상하반전
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