#!/usr/bin/env python3

import threading
import cv2

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

from datetime import datetime
from .cap_and_send import FrameBuffer, post_batch
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
        self.declare_parameter("image_topic", "/camera/color/image_raw")

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
        self.image_topic = str(self.get_parameter("image_topic").value)


        self.get_logger().info(f"server_url     = {self.server_url}")
        self.get_logger().info(f"signaling_url  = {self.signaling_url}")
        self.get_logger().info(f"n_frames       = {self.n_frames}")
        self.get_logger().info(f"sample_dt      = {self.sample_dt}")
        self.get_logger().info(f"capture_timeout_s = {self.capture_timeout_s}")
        self.get_logger().info(f"post_timeout_s    = {self.post_timeout_s}")

        self.bridge = CvBridge()
        self.buffer = FrameBuffer()

        self.webrtc_sender = WebRTCSender(
            signaling_base_url=self.signaling_url,
            image_topic=self.image_topic,
        )
        self.webrtc_sender.start()

        self.place_id = "00"

        self.lock = threading.Lock()
        self.sending = False

        self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10,
        )

        self.create_subscription(
            String,
            "/patrol/capture_trigger",
            self.trigger_cb,
            10,
        )
        self.capture_done_pub = self.create_publisher(
            String,
            "/patrol/capture_done",
            10,
        )

        self.get_logger().info("capture sender ready")

    def image_callback(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.buffer.update(rgb)

    def trigger_cb(self, msg: String):
        place_id = msg.data.strip()

        if not place_id:
            self.get_logger().warning("empty place_id in /patrol/capture_trigger")
            return

        with self.lock:
            if self.sending:
                self.get_logger().info("already sending")
                return

            self.sending = True
            self.place_id = place_id

        self.get_logger().info(f"capture trigger received: place_id={self.place_id}")

        threading.Thread(
            target=self.send_worker,
            daemon=True,
        ).start()

    def publish_capture_result(self, status: str, place_id: str):
        msg = String()
        msg.data = f"{status}:{place_id}"
        self.capture_done_pub.publish(msg)
        self.get_logger().info(f"published /patrol/capture_done = {msg.data}")

    def send_worker(self):
        current_place_id = self.place_id

        try:
            self.get_logger().info(f"capture start place={current_place_id}")

            frames = self.buffer.capture_n(
                n_frames=self.n_frames,
                sample_dt=self.sample_dt,
                timeout_s=self.capture_timeout_s,
            )

        except Exception as e:
            self.get_logger().error(f"capture failed: {e}")
            self.publish_capture_result("fail", current_place_id)

            with self.lock:
                self.sending = False
            return

        self.publish_capture_result("done", current_place_id)

        try:
            self.get_logger().info(f"upload start place={current_place_id}")

            meta = {
                "place_id": current_place_id,
                "timestamp": datetime.now().isoformat(),
                "n_frames": len(frames),
            }

            post_batch(
                server_url=self.server_url,
                images_rgb=frames,
                meta=meta,
                timeout_s=self.post_timeout_s,
            )

            self.get_logger().info(f"upload done place={current_place_id}")

        except Exception as e:
            self.get_logger().error(f"upload failed after capture done: {e}")

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