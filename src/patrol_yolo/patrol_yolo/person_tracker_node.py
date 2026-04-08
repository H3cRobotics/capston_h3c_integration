# person_tracker_node.py

import json
import time
import threading
from typing import Optional, List, Dict, Any, Tuple

import cv2
import numpy as np
from ultralytics import YOLO

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge

from message_filters import Subscriber, ApproximateTimeSynchronizer


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class PersonTrackerNode(Node):
    def __init__(self):
        super().__init__("person_tracker_node")

        self.declare_parameter("mode", "webcam")  # webcam | realsense

        self.declare_parameter("webcam_color_topic", "/camera/color/image_raw")

        self.declare_parameter("realsense_color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("realsense_depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")

        self.declare_parameter("model_path", "yolov8n.pt")
        self.declare_parameter("tracker_cfg", "bytetrack.yaml")
        self.declare_parameter("conf_thres", 0.35)
        self.declare_parameter("iou_thres", 0.45)
        self.declare_parameter("use_gpu", True)

        self.declare_parameter("annotated_topic", "/person_tracking/annotated")
        self.declare_parameter("tracks_topic", "/person_tracking/tracks_json")

        self.declare_parameter("depth_roi_half", 3)
        self.declare_parameter("min_depth_m", 0.2)
        self.declare_parameter("max_depth_m", 8.0)
        self.declare_parameter("depth_scale", 0.001)

        self.declare_parameter("sync_slop", 0.08)
        self.declare_parameter("queue_size", 5)

        self.declare_parameter("worker_sleep_ms", 2)
        self.declare_parameter("stale_frame_ms", 300)
        self.declare_parameter("log_fps", True)
        self.declare_parameter("max_process_fps", 10.0)

        self.declare_parameter("imgsz", 640)
        self.imgsz = int(self.get_parameter("imgsz").value)

        self.mode = str(self.get_parameter("mode").value).lower().strip()

        self.webcam_color_topic = self.get_parameter("webcam_color_topic").value
        self.realsense_color_topic = self.get_parameter("realsense_color_topic").value
        self.realsense_depth_topic = self.get_parameter("realsense_depth_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value

        self.model_path = self.get_parameter("model_path").value
        self.tracker_cfg = self.get_parameter("tracker_cfg").value
        self.conf_thres = float(self.get_parameter("conf_thres").value)
        self.iou_thres = float(self.get_parameter("iou_thres").value)
        self.use_gpu = bool(self.get_parameter("use_gpu").value)

        self.annotated_topic = self.get_parameter("annotated_topic").value
        self.tracks_topic = self.get_parameter("tracks_topic").value

        self.depth_roi_half = int(self.get_parameter("depth_roi_half").value)
        self.min_depth_m = float(self.get_parameter("min_depth_m").value)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)
        self.depth_scale = float(self.get_parameter("depth_scale").value)

        self.sync_slop = float(self.get_parameter("sync_slop").value)
        self.queue_size = int(self.get_parameter("queue_size").value)

        self.worker_sleep_ms = int(self.get_parameter("worker_sleep_ms").value)
        self.stale_frame_ms = int(self.get_parameter("stale_frame_ms").value)
        self.log_fps = bool(self.get_parameter("log_fps").value)
        self.max_process_fps = float(self.get_parameter("max_process_fps").value)

        if self.mode not in ["webcam", "realsense"]:
            self.get_logger().warn(f"Unknown mode '{self.mode}', fallback to webcam")
            self.mode = "webcam"

        self.use_depth = (self.mode == "realsense")

        self.bridge = CvBridge()

        self.fx: Optional[float] = None
        self.fy: Optional[float] = None
        self.cx0: Optional[float] = None
        self.cy0: Optional[float] = None

        self.frame_lock = threading.Lock()
        self.latest_packet: Optional[Dict[str, Any]] = None
        self.worker_busy = False
        self.stop_event = threading.Event()

        self.last_proc_time = None
        self.proc_fps = 0.0
        self.last_infer_time = 0.0

        self.get_logger().info(f"Loading YOLO model: {self.model_path}")
        self.model = YOLO(self.model_path, task = "detect")

        self.annotated_pub = self.create_publisher(Image, self.annotated_topic, 10)
        self.tracks_pub = self.create_publisher(String, self.tracks_topic, 10)

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            10
        )

        if self.mode == "webcam":
            self.get_logger().info(f"[MODE] webcam | color_topic={self.webcam_color_topic}")
            self.color_sub_only = self.create_subscription(
                Image,
                self.webcam_color_topic,
                self.color_callback,
                10
            )

        else:
            self.get_logger().info(
                f"[MODE] realsense | color_topic={self.realsense_color_topic} | depth_topic={self.realsense_depth_topic}"
            )
            self.color_sub = Subscriber(self, Image, self.realsense_color_topic)
            self.depth_sub = Subscriber(self, Image, self.realsense_depth_topic)

            self.ts = ApproximateTimeSynchronizer(
                [self.color_sub, self.depth_sub],
                queue_size=self.queue_size,
                slop=self.sync_slop
            )
            self.ts.registerCallback(self.sync_callback)

        self.worker_thread = threading.Thread(target=self.worker_loop, daemon=True)
        self.worker_thread.start()

        self.get_logger().info("PersonTrackerNode initialized with worker thread.")

    def camera_info_callback(self, msg: CameraInfo):
        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx0 = float(msg.k[2])
        self.cy0 = float(msg.k[5])

    def color_callback(self, color_msg: Image):
        try:
            color = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Color image conversion failed: {e}")
            return

        self.get_logger().info(
            f"[CB] color frame in stamp={color_msg.header.stamp.sec}.{color_msg.header.stamp.nanosec}",
            throttle_duration_sec=2.0
        )

        packet = {
            "stamp_ns": self.msg_to_ns(color_msg),
            "recv_ns": self.get_clock().now().nanoseconds,
            "color_msg": color_msg,
            "color": color,
            "depth": None,
        }
        self.set_latest_packet(packet)

    def sync_callback(self, color_msg: Image, depth_msg: Image):
        try:
            color = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Color image conversion failed: {e}")
            return

        try:
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f"Depth image conversion failed: {e}")
            return

        packet = {
            "stamp_ns": self.msg_to_ns(color_msg),
            "color_msg": color_msg,
            "color": color,
            "depth": depth,
        }
        self.set_latest_packet(packet)

    def set_latest_packet(self, packet: Dict[str, Any]):
        with self.frame_lock:
            self.latest_packet = packet

    def worker_loop(self):
        while not self.stop_event.is_set():
            now = time.time()

            if self.max_process_fps > 0:
                min_interval = 1.0 / self.max_process_fps
                if (now - self.last_infer_time) < min_interval:
                    time.sleep(self.worker_sleep_ms / 1000.0)
                    continue

            packet = None

            with self.frame_lock:
                if self.latest_packet is not None and not self.worker_busy:
                    packet = self.latest_packet
                    self.latest_packet = None
                    self.worker_busy = True

            if packet is None:
                time.sleep(self.worker_sleep_ms / 1000.0)
                continue

            try:
                age_ms = (time.time_ns() - packet["stamp_ns"]) / 1e6
                if age_ms > self.stale_frame_ms:
                    self.get_logger().warn(
                        f"Dropping stale frame: {age_ms:.1f} ms",
                        throttle_duration_sec=2.0
                    )
                    continue

                self.last_infer_time = time.time()

                annotated, tracks = self.run_tracking(
                    color=packet["color"],
                    depth=packet["depth"]
                )

                self.publish_outputs(
                    color_msg=packet["color_msg"],
                    annotated=annotated,
                    tracks=tracks
                )

                self.update_fps()

            except Exception as e:
                self.get_logger().error(f"Worker loop failed: {e}")

            finally:
                with self.frame_lock:
                    self.worker_busy = False

    def run_tracking(
        self,
        color: np.ndarray,
        depth: Optional[np.ndarray] = None
    ) -> Tuple[np.ndarray, List[Dict[str, Any]]]:
        annotated = color.copy()
        device = 0 if self.use_gpu else "cpu"

        results = self.model.track(
            source=color,
            persist=True,
            classes=[0],
            conf=self.conf_thres,
            iou=self.iou_thres,
            tracker=self.tracker_cfg,
            verbose=False,
            device=device,
            imgsz=self.imgsz,
        )

        tracks: List[Dict[str, Any]] = []

        if len(results) == 0 or results[0].boxes is None:
            return annotated, tracks

        boxes = results[0].boxes

        for box in boxes:
            cls_id = int(box.cls.item()) if box.cls is not None else -1
            if cls_id != 0:
                continue

            xyxy = box.xyxy[0].detach().cpu().numpy().astype(int)
            x1, y1, x2, y2 = xyxy.tolist()

            conf = float(box.conf.item()) if box.conf is not None else 0.0
            track_id = int(box.id.item()) if box.id is not None else -1

            u = int((x1 + x2) / 2)
            v = int((y1 + y2) / 2)

            item: Dict[str, Any] = {
                "person_id": track_id,
                "x": u,
                "y": v,
                "confidence": conf,
                "bbox_xyxy": [x1, y1, x2, y2],
            }

            if self.fx is not None and self.fy is not None and self.cx0 is not None and self.cy0 is not None:
                item["x_norm"] = float((u - self.cx0) / self.fx)
                item["y_norm"] = float((v - self.cy0) / self.fy)

            z_m = None
            if depth is not None:
                z_m = self.get_depth_median(depth, u, v, self.depth_roi_half)
                if z_m is not None:
                    item["z"] = float(z_m)

            tracks.append(item)

            if depth is None:
                color_box = (0, 255, 0)
            else:
                color_box = (0, 255, 0) if z_m is not None else (0, 0, 255)

            cv2.rectangle(annotated, (x1, y1), (x2, y2), color_box, 2)
            cv2.circle(annotated, (u, v), 4, (255, 0, 0), -1)

            if z_m is None:
                label = f"id={track_id} ({u},{v}) {conf:.2f}"
            else:
                label = f"id={track_id} ({u},{v}) z={z_m:.2f}m {conf:.2f}"

            if self.log_fps:
                label += f" fps={self.proc_fps:.1f}"

            cv2.putText(
                annotated,
                label,
                (x1, max(0, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color_box,
                2
            )

        return annotated, tracks

    def get_depth_median(self, depth_img: np.ndarray, u: int, v: int, half: int) -> Optional[float]:
        h, w = depth_img.shape[:2]

        u0 = clamp(u - half, 0, w - 1)
        u1 = clamp(u + half + 1, 0, w)
        v0 = clamp(v - half, 0, h - 1)
        v1 = clamp(v + half + 1, 0, h)

        patch = depth_img[v0:v1, u0:u1]
        if patch.size == 0:
            return None

        if patch.dtype == np.uint16:
            vals = patch.astype(np.float32) * self.depth_scale
        else:
            vals = patch.astype(np.float32)

        vals = vals[np.isfinite(vals)]
        vals = vals[(vals > self.min_depth_m) & (vals < self.max_depth_m)]

        if len(vals) == 0:
            return None

        return float(np.median(vals))

    def publish_outputs(self, color_msg: Image, annotated: np.ndarray, tracks: List[Dict[str, Any]]):
        try:
            ann_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            ann_msg.header = color_msg.header
            self.annotated_pub.publish(ann_msg)
        except Exception as e:
            self.get_logger().error(f"Annotated publish failed: {e}")

        payload = {
            "mode": self.mode,
            "header": {
                "stamp_sec": int(color_msg.header.stamp.sec),
                "stamp_nanosec": int(color_msg.header.stamp.nanosec),
                "frame_id": color_msg.header.frame_id,
            },
            "num_persons": len(tracks),
            "tracks": tracks,
            "proc_fps": self.proc_fps,
            "max_process_fps": self.max_process_fps,
        }

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.tracks_pub.publish(msg)

    def msg_to_ns(self, msg: Image) -> int:
        return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)

    def update_fps(self):
        now = time.time()
        if self.last_proc_time is not None:
            dt = now - self.last_proc_time
            if dt > 0:
                inst_fps = 1.0 / dt
                if self.proc_fps == 0.0:
                    self.proc_fps = inst_fps
                else:
                    self.proc_fps = 0.9 * self.proc_fps + 0.1 * inst_fps
        self.last_proc_time = now

    def destroy_node(self):
        self.stop_event.set()
        if hasattr(self, "worker_thread") and self.worker_thread.is_alive():
            self.worker_thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PersonTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()