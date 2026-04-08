import json
import time
import threading
from datetime import datetime
from typing import Optional, Dict, Any, List

import cv2
import requests

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D
from cv_bridge import CvBridge


class PersonEventSenderNode(Node):
    def __init__(self):
        super().__init__("person_event_sender_node")

        # topics
        self.declare_parameter("tracks_topic", "/person_tracking/tracks_json")
        self.declare_parameter("annotated_topic", "/person_tracking/annotated")
        self.declare_parameter("robot_pose_topic", "/robot_pose")

        # server
        self.declare_parameter("server_url", "http://127.0.0.1:8000/person_event")
        self.declare_parameter("request_timeout_sec", 3.0)

        # detection filters
        self.declare_parameter("min_confidence", 0.6)
        self.declare_parameter("min_consecutive_hits", 2)
        self.declare_parameter("absence_reset_sec", 2.0)
        self.declare_parameter("global_cooldown_sec", 1.0)

        # dwelling
        self.declare_parameter("dwell_time_sec", 10.0)

        # bbox filter
        self.declare_parameter("min_bbox_w", 40)
        self.declare_parameter("min_bbox_h", 80)

        # optional depth filter
        self.declare_parameter("use_depth_filter", False)
        self.declare_parameter("min_depth_m", 0.5)
        self.declare_parameter("max_depth_m", 4.0)

        # image encoding
        self.declare_parameter("jpeg_quality", 90)

        # local debug save
        self.declare_parameter("save_local_debug", True)
        self.declare_parameter("debug_dir", "/tmp/person_event_sender")

        self.tracks_topic = str(self.get_parameter("tracks_topic").value)
        self.annotated_topic = str(self.get_parameter("annotated_topic").value)
        self.robot_pose_topic = str(self.get_parameter("robot_pose_topic").value)

        self.server_url = str(self.get_parameter("server_url").value)
        self.request_timeout_sec = float(self.get_parameter("request_timeout_sec").value)

        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.min_consecutive_hits = int(self.get_parameter("min_consecutive_hits").value)
        self.absence_reset_sec = float(self.get_parameter("absence_reset_sec").value)
        self.global_cooldown_sec = float(self.get_parameter("global_cooldown_sec").value)

        self.dwell_time_sec = float(self.get_parameter("dwell_time_sec").value)

        self.min_bbox_w = int(self.get_parameter("min_bbox_w").value)
        self.min_bbox_h = int(self.get_parameter("min_bbox_h").value)

        self.use_depth_filter = bool(self.get_parameter("use_depth_filter").value)
        self.min_depth_m = float(self.get_parameter("min_depth_m").value)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)

        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)

        self.save_local_debug = bool(self.get_parameter("save_local_debug").value)
        self.debug_dir = str(self.get_parameter("debug_dir").value)

        self.bridge = CvBridge()
        self.lock = threading.Lock()

        self.latest_annotated_msg: Optional[Image] = None
        self.latest_pose_msg: Optional[Pose2D] = None

        # global presence state
        self.presence_hits = 0
        self.person_present = False
        self.last_valid_seen_time = 0.0
        self.last_sent_time = 0.0

        # per-person tracking for dwelling
        # {
        #   "7": {
        #       "first_seen": ...,
        #       "last_seen": ...,
        #       "dwell_sent": False
        #   }
        # }
        self.person_states: Dict[str, Dict[str, Any]] = {}

        self.annotated_sub = self.create_subscription(
            Image,
            self.annotated_topic,
            self.annotated_callback,
            10
        )

        self.pose_sub = self.create_subscription(
            Pose2D,
            self.robot_pose_topic,
            self.pose_callback,
            10
        )

        self.tracks_sub = self.create_subscription(
            String,
            self.tracks_topic,
            self.tracks_callback,
            10
        )

        if self.save_local_debug:
            import os
            os.makedirs(self.debug_dir, exist_ok=True)

        self.get_logger().info(
            f"[INIT] PersonEventSenderNode | "
            f"tracks={self.tracks_topic} | "
            f"annotated={self.annotated_topic} | "
            f"robot_pose={self.robot_pose_topic}"
        )

    def annotated_callback(self, msg: Image):
        with self.lock:
            self.latest_annotated_msg = msg

    def pose_callback(self, msg: Pose2D):
        with self.lock:
            self.latest_pose_msg = msg

    def tracks_callback(self, msg: String):
        now = time.time()

        try:
            payload = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"[JSON] parse failed: {e}")
            return

        valid_tracks = self.extract_valid_tracks(payload)

        if not valid_tracks:
            self.cleanup_missing_tracks(seen_ids=set(), now=now)

            if (now - self.last_valid_seen_time) > self.absence_reset_sec:
                if self.person_present or self.presence_hits != 0:
                    self.get_logger().info("[STATE] reset to NO_PERSON")
                self.presence_hits = 0
                self.person_present = False
            return

        self.last_valid_seen_time = now
        self.presence_hits += 1

        seen_ids = self.update_person_states(valid_tracks, now)
        self.cleanup_missing_tracks(seen_ids=seen_ids, now=now)

        # 1) first presence event
        if (
            self.presence_hits >= self.min_consecutive_hits
            and not self.person_present
            and (now - self.last_sent_time) >= self.global_cooldown_sec
        ):
            self.send_event_if_possible(
                event_type="person_present",
                num_persons=len(valid_tracks)
            )

        # 2) dwelling event per stable person_id
        for tr in valid_tracks:
            person_id = str(tr.get("person_id", -1))
            if person_id == "-1":
                continue

            st = self.person_states.get(person_id)
            if st is None:
                continue

            dwell_elapsed = now - st["first_seen"]
            if dwell_elapsed >= self.dwell_time_sec and not st["dwell_sent"]:
                sent = self.send_event_if_possible(
                    event_type="person_dwelling",
                    num_persons=len(valid_tracks),
                    dwell_person_id=person_id,
                    dwell_time_sec=round(dwell_elapsed, 1)
                )
                if sent:
                    st["dwell_sent"] = True

    def extract_valid_tracks(self, payload: Dict[str, Any]) -> List[Dict[str, Any]]:
        tracks = payload.get("tracks", [])
        valid_tracks: List[Dict[str, Any]] = []

        for tr in tracks:
            conf = float(tr.get("confidence", 0.0))
            if conf < self.min_confidence:
                continue

            bbox = tr.get("bbox_xyxy", None)
            if not isinstance(bbox, list) or len(bbox) != 4:
                continue

            x1, y1, x2, y2 = bbox
            bw = max(0, int(x2) - int(x1))
            bh = max(0, int(y2) - int(y1))

            if bw < self.min_bbox_w or bh < self.min_bbox_h:
                continue

            if self.use_depth_filter:
                z = tr.get("z", None)
                if z is None:
                    continue
                z = float(z)
                if z < self.min_depth_m or z > self.max_depth_m:
                    continue

            valid_tracks.append(tr)

        return valid_tracks

    def update_person_states(self, valid_tracks: List[Dict[str, Any]], now: float) -> set:
        seen_ids = set()

        for tr in valid_tracks:
            person_id = str(tr.get("person_id", -1))
            if person_id == "-1":
                continue

            seen_ids.add(person_id)

            if person_id not in self.person_states:
                self.person_states[person_id] = {
                    "first_seen": now,
                    "last_seen": now,
                    "dwell_sent": False,
                }
            else:
                self.person_states[person_id]["last_seen"] = now

        return seen_ids

    def cleanup_missing_tracks(self, seen_ids: set, now: float):
        to_delete = []

        for pid, st in self.person_states.items():
            if pid in seen_ids:
                continue
            if (now - st["last_seen"]) > self.absence_reset_sec:
                to_delete.append(pid)

        for pid in to_delete:
            del self.person_states[pid]

    def pose_to_dict(self, pose_msg: Pose2D) -> Dict[str, Any]:
        return {
            "x": float(pose_msg.x),
            "y": float(pose_msg.y),
            "yaw": float(pose_msg.theta),
        }

    def send_event_if_possible(
        self,
        event_type: str,
        num_persons: int,
        dwell_person_id: Optional[str] = None,
        dwell_time_sec: Optional[float] = None,
    ) -> bool:
        with self.lock:
            ann_msg = self.latest_annotated_msg
            pose_msg = self.latest_pose_msg

        if ann_msg is None:
            self.get_logger().warn(f"[SEND] skipped ({event_type}): no annotated frame cached")
            return False

        if pose_msg is None:
            self.get_logger().warn(f"[SEND] skipped ({event_type}): no robot pose cached")
            return False

        try:
            annotated = self.bridge.imgmsg_to_cv2(ann_msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"[CV] annotated conversion failed: {e}")
            return False

        ok, enc = cv2.imencode(
            ".jpg",
            annotated,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        )
        if not ok:
            self.get_logger().error("[JPEG] encode failed")
            return False

        event: Dict[str, Any] = {
            "event_type": event_type,
            "event_time": datetime.now().astimezone().isoformat(timespec="seconds"),
            "robot_pose": self.pose_to_dict(pose_msg),
            "num_persons": int(num_persons),
        }

        if dwell_person_id is not None:
            event["person_id"] = dwell_person_id
        if dwell_time_sec is not None:
            event["dwell_time_sec"] = float(dwell_time_sec)

        jpg_bytes = enc.tobytes()

        threading.Thread(
            target=self._send_event,
            args=(event, jpg_bytes),
            daemon=True
        ).start()

        return True

    def _send_event(self, event: Dict[str, Any], jpg_bytes: bytes):
        try:
            files = {
                "image": ("person_event.jpg", jpg_bytes, "image/jpeg")
            }
            data = {
                "event_json": json.dumps(event, ensure_ascii=False)
            }

            resp = requests.post(
                self.server_url,
                data=data,
                files=files,
                timeout=self.request_timeout_sec
            )

            if 200 <= resp.status_code < 300:
                now = time.time()

                if event["event_type"] == "person_present":
                    self.person_present = True
                    self.last_sent_time = now

                self.get_logger().info(
                    f"[SEND] success status={resp.status_code} "
                    f"type={event['event_type']} "
                    f"time={event['event_time']} "
                    f"num_persons={event['num_persons']}"
                )

                if self.save_local_debug:
                    self.save_debug_copy(event, jpg_bytes)
            else:
                self.get_logger().warn(
                    f"[SEND] failed status={resp.status_code} body={resp.text[:200]}"
                )

        except Exception as e:
            self.get_logger().error(f"[SEND] exception: {e}")

    def save_debug_copy(self, event: Dict[str, Any], jpg_bytes: bytes):
        try:
            import os

            ts = int(time.time() * 1000)
            img_path = os.path.join(self.debug_dir, f"{ts}.jpg")
            json_path = os.path.join(self.debug_dir, f"{ts}.json")

            with open(img_path, "wb") as f:
                f.write(jpg_bytes)

            with open(json_path, "w", encoding="utf-8") as f:
                json.dump(event, f, ensure_ascii=False, indent=2)

        except Exception as e:
            self.get_logger().warn(f"[DEBUG SAVE] failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = PersonEventSenderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()