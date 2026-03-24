#!/usr/bin/env python3

import threading
import cv2
import requests

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String, Empty
from cv_bridge import CvBridge

from .cap_and_send import FrameBuffer, capture_and_send
from .webrtc_sender import WebRTCSender

"""
8090 잔여물 남아있을 때

pkill -f patrol_http_bridge
pkill -f realsense
pkill -f camera_publisher
pkill -f python3

카메라 못 잡을 때

ls /dev/video*
v4l2-ctl --list-devices
ls -l /dev/v4l/by-id
for d in /dev/video0 /dev/video1 /dev/video2 /dev/video3 /dev/video4 /dev/video5; do
  echo "===== $d ====="
  v4l2-ctl -d $d --list-formats-ext 2>/dev/null
done
"""

N_FRAMES = 5
SAMPLE_DT = 0.2

CAPTURE_TIMEOUT_S = 5.0
POST_TIMEOUT_S = 10.0


class CaptureSender(Node):
    def __init__(self):
        super().__init__("capture_sender")

        # -------------------------------
        # ROS parameters
        # -------------------------------
        self.declare_parameter("server_url", "http://127.0.0.1:8000")
        self.declare_parameter("signaling_url", "http://127.0.0.1:8001")
        self.declare_parameter("n_frames", N_FRAMES)
        self.declare_parameter("sample_dt", SAMPLE_DT)
        self.declare_parameter("capture_timeout_s", CAPTURE_TIMEOUT_S)
        self.declare_parameter("post_timeout_s", POST_TIMEOUT_S)

        self.server_url = str(self.get_parameter("server_url").value).rstrip("/")
        self.signaling_url = str(self.get_parameter("signaling_url").value).rstrip("/")
        self.n_frames = int(self.get_parameter("n_frames").value)
        self.sample_dt = float(self.get_parameter("sample_dt").value)
        self.capture_timeout_s = float(self.get_parameter("capture_timeout_s").value)
        self.post_timeout_s = float(self.get_parameter("post_timeout_s").value)

        self.get_logger().info(f"server_url     = {self.server_url}")
        self.get_logger().info(f"signaling_url  = {self.signaling_url}")
        self.get_logger().info(f"n_frames       = {self.n_frames}")
        self.get_logger().info(f"sample_dt      = {self.sample_dt}")
        self.get_logger().info(f"capture_timeout_s = {self.capture_timeout_s}")
        self.get_logger().info(f"post_timeout_s    = {self.post_timeout_s}")

        self.bridge = CvBridge()
        self.buffer = FrameBuffer()

        self.webrtc_sender = WebRTCSender(
            buffer=self.buffer,
            signaling_base_url=self.signaling_url,
        )
        self.webrtc_sender.start()

        self.place_id = "00"
        self.query_gt = "normal"

        self.lock = threading.Lock()
        self.sending = False

        self.create_subscription(
            Image,
            "/camera/color/image_raw",
            self.image_callback,
            10,
        )

        self.create_subscription(
            String,
            "/patrol/current_place",
            self.place_cb,
            10,
        )

        self.create_subscription(
            String,
            "/patrol/query_gt",
            self.query_gt_cb,
            10,
        )

        self.create_subscription(
            Empty,
            "/patrol/capture_trigger",
            self.trigger_cb,
            10,
        )

        self.get_logger().info("capture sender ready")

    def image_callback(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.buffer.update(rgb)

    def place_cb(self, msg: String):
        self.place_id = msg.data.strip()
        self.get_logger().info(f"place -> {self.place_id}")

    def query_gt_cb(self, msg: String):
        label = msg.data.strip().lower()

        if label in ["normal", "abnormal"]:
            self.query_gt = label
            self.get_logger().info(f"query_gt -> {self.query_gt}")
        else:
            self.get_logger().warning(f"invalid query_gt: {msg.data}")

    def get_server_mode(self) -> str:
        url = f"{self.server_url}/places/{self.place_id}"

        r = requests.get(url, timeout=3)
        r.raise_for_status()
        data = r.json()

        place = data.get("place")
        if place is None:
            raise RuntimeError(f"no place data returned for place_id={self.place_id}")

        mode = str(place.get("mode", "bank"))

        if mode not in ["bank", "th_calib", "query"]:
            self.get_logger().warning(
                f"unexpected mode '{mode}' from server. fallback -> bank"
            )
            mode = "bank"

        return mode

    def trigger_cb(self, msg: Empty):
        with self.lock:
            if self.sending:
                self.get_logger().info("already sending")
                return

            self.sending = True

        threading.Thread(
            target=self.send_worker,
            daemon=True,
        ).start()

    def send_worker(self):
        try:
            mode = self.get_server_mode()

            if mode in ["bank", "th_calib"]:
                label = "normal"
            else:
                label = self.query_gt

            self.get_logger().info(
                f"send place={self.place_id} mode={mode} gt={label}"
            )

            capture_and_send(
                buffer=self.buffer,
                server_url=self.server_url,
                place_id=self.place_id,
                mode=mode,
                n_frames=self.n_frames,
                sample_dt=self.sample_dt,
                capture_timeout_s=self.capture_timeout_s,
                post_timeout_s=self.post_timeout_s,
                gt=label,
            )

        except Exception as e:
            self.get_logger().error(f"send_worker failed: {e}")

        finally:
            with self.lock:
                self.sending = False


def main():
    rclpy.init()
    node = CaptureSender()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received, shutting down.")
    finally:
        try:
            node.webrtc_sender.stop()
        except Exception:
            pass

        try:
            node.destroy_node()
        except Exception:
            pass

        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()