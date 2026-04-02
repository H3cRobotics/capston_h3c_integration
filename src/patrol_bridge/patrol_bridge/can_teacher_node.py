#!/usr/bin/env python3
import re
import time
import queue
import threading
from typing import Optional, Dict, Any

import can
import requests
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D


class CanTeacherNode(Node):
    def __init__(self):
        super().__init__("can_teacher_node")

        # ---------- Parameters ----------
        self.declare_parameter("server_url", "http://192.168.0.16:8000")
        self.declare_parameter("robot_pose_topic", "/robot_pose")

        self.declare_parameter("can_interface", "can0")
        self.declare_parameter("teach_can_id", 0x102)

        self.declare_parameter("default_patrol_enabled", True)
        self.declare_parameter("place_prefix", "P")

        self.declare_parameter("debounce_sec", 1.0)
        self.declare_parameter("http_timeout_sec", 3.0)
        self.declare_parameter("queue_size", 10)

        self.server_url = str(self.get_parameter("server_url").value).rstrip("/")
        self.robot_pose_topic = str(self.get_parameter("robot_pose_topic").value)

        self.can_interface = str(self.get_parameter("can_interface").value)
        self.teach_can_id = int(self.get_parameter("teach_can_id").value)

        self.default_patrol_enabled = bool(self.get_parameter("default_patrol_enabled").value)
        self.place_prefix = str(self.get_parameter("place_prefix").value or "P")

        self.debounce_sec = float(self.get_parameter("debounce_sec").value)
        self.http_timeout_sec = float(self.get_parameter("http_timeout_sec").value)
        self.queue_size = int(self.get_parameter("queue_size").value)

        # ---------- Pose cache ----------
        self.pose_lock = threading.Lock()
        self.x: Optional[float] = None
        self.y: Optional[float] = None
        self.yaw: Optional[float] = None
        self.last_pose_time: Optional[float] = None

        # ---------- Teach queue ----------
        self.teach_queue: "queue.Queue[Dict[str, Any]]" = queue.Queue(maxsize=self.queue_size)

        # ---------- CAN state ----------
        self.bus = None
        self.last_trigger_time = 0.0
        self.prev_pressed = False

        # ---------- Thread control ----------
        self.stop_event = threading.Event()
        self.can_thread: Optional[threading.Thread] = None
        self.http_thread: Optional[threading.Thread] = None

        # ---------- ROS subscription ----------
        self.create_subscription(
            Pose2D,
            self.robot_pose_topic,
            self.robot_pose_callback,
            10,
        )

        # ---------- HTTP session ----------
        self.session = requests.Session()

        # ---------- Init CAN ----------
        self._init_can()

        # ---------- Start threads ----------
        self.can_thread = threading.Thread(target=self.can_loop, daemon=True)
        self.can_thread.start()

        self.http_thread = threading.Thread(target=self.http_worker_loop, daemon=True)
        self.http_thread.start()

        self.get_logger().info(
            f"CanTeacherNode started | pose_topic={self.robot_pose_topic} | "
            f"server={self.server_url} | can_if={self.can_interface} | "
            f"teach_can_id=0x{self.teach_can_id:X}"
        )

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------
    def robot_pose_callback(self, msg: Pose2D) -> None:
        with self.pose_lock:
            self.x = float(msg.x)
            self.y = float(msg.y)
            self.yaw = float(msg.theta)
            self.last_pose_time = time.time()

    # ------------------------------------------------------------------
    # CAN
    # ------------------------------------------------------------------
    def _init_can(self) -> None:
        try:
            self.bus = can.interface.Bus(channel=self.can_interface, bustype="socketcan")
            self.get_logger().info(f"CAN bus connected: {self.can_interface}")
        except Exception as e:
            self.get_logger().error(f"Failed to open CAN bus {self.can_interface}: {e}")
            raise

    def can_loop(self) -> None:
        while not self.stop_event.is_set():
            try:
                msg = self.bus.recv(timeout=0.5)
                if msg is None:
                    continue

                if msg.arbitration_id != self.teach_can_id:
                    continue

                if len(msg.data) == 0:
                    self.get_logger().warning("Teach CAN frame received but data is empty")
                    continue

                value = int(msg.data[0])
                pressed = (value == 1)

                now = time.time()
                cooldown_ok = (now - self.last_trigger_time) > self.debounce_sec

                # 상승엣지 기준
                if pressed and (not self.prev_pressed):
                    if cooldown_ok:
                        self.handle_teach_button_pressed()
                        self.last_trigger_time = now
                    else:
                        self.get_logger().warning(
                            f"Teach press ignored by debounce | "
                            f"dt={now - self.last_trigger_time:.3f}s < {self.debounce_sec:.3f}s"
                        )

                self.prev_pressed = pressed

            except Exception as e:
                self.get_logger().error(f"CAN loop error: {e}")
                time.sleep(0.2)

    def handle_teach_button_pressed(self) -> None:
        pose_snapshot = self.get_pose_snapshot()
        if pose_snapshot is None:
            self.get_logger().warning("Teach pressed but robot pose is not ready")
            return

        job = {
            "pressed_at": time.time(),
            "x": pose_snapshot["x"],
            "y": pose_snapshot["y"],
            "yaw": pose_snapshot["yaw"],
        }

        try:
            self.teach_queue.put_nowait(job)
            self.get_logger().info(
                f"Teach queued | "
                f"x={job['x']:.3f}, y={job['y']:.3f}, yaw={job['yaw']:.3f} | "
                f"queue_size={self.teach_queue.qsize()}"
            )
        except queue.Full:
            self.get_logger().error("Teach queue is full; dropping teach request")

    def get_pose_snapshot(self) -> Optional[Dict[str, float]]:
        with self.pose_lock:
            if self.x is None or self.y is None or self.yaw is None:
                return None
            return {
                "x": float(self.x),
                "y": float(self.y),
                "yaw": float(self.yaw),
            }

    # ------------------------------------------------------------------
    # HTTP worker
    # ------------------------------------------------------------------
    def http_worker_loop(self) -> None:
        while not self.stop_event.is_set():
            try:
                job = self.teach_queue.get(timeout=0.5)
            except queue.Empty:
                continue

            try:
                self.process_teach_job(job)
            except Exception as e:
                self.get_logger().error(f"Teach worker unexpected error: {e}")
            finally:
                self.teach_queue.task_done()

    def process_teach_job(self, job: Dict[str, Any]) -> None:
        place_id = self.get_next_place_id()

        payload = {
            "place_id": place_id,
            "x": job["x"],
            "y": job["y"],
            "yaw": job["yaw"],
            "display_name": place_id,
            "patrol_enabled": self.default_patrol_enabled,
        }

        response = self.session.post(
            f"{self.server_url}/robot/teach",
            json=payload,
            timeout=self.http_timeout_sec,
        )
        response.raise_for_status()

        self.get_logger().info(
            f"Teach success | place_id={place_id} | "
            f"x={job['x']:.3f}, y={job['y']:.3f}, yaw={job['yaw']:.3f}"
        )

    def get_next_place_id(self) -> str:
        response = self.session.get(
            f"{self.server_url}/places",
            timeout=self.http_timeout_sec,
        )
        response.raise_for_status()

        places = response.json().get("places", [])
        max_num = 0
        pattern = re.compile(rf"^{re.escape(self.place_prefix)}(\d+)$")

        for place in places:
            place_id = str(place.get("place_id", ""))
            m = pattern.match(place_id)
            if m:
                max_num = max(max_num, int(m.group(1)))

        return f"{self.place_prefix}{max_num + 1:03d}"

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------
    def destroy_node(self):
        self.stop_event.set()

        try:
            if self.bus is not None:
                self.bus.shutdown()
        except Exception:
            pass

        try:
            self.session.close()
        except Exception:
            pass

        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CanTeacherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()