#dummy_pose_input.py

import json
import threading
from typing import List, Dict, Any

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String, Empty


class DummyPoseInputNode(Node):
    def __init__(self):
        super().__init__("dummy_pose_input")

        self.declare_parameter("pose_topic", "/robot_pose")
        self.declare_parameter("goal_topic", "/goal_pose_2d")
        self.declare_parameter("next_place_topic", "/next_place_id")
        self.declare_parameter("status_topic", "/robot_status")
        self.declare_parameter("command_topic", "/patrol/command")
        self.declare_parameter("waypoints_topic", "/patrol/waypoints_json")
        self.declare_parameter("reload_waypoints_topic", "/patrol/reload_waypoints")
        self.declare_parameter("publish_period_sec", 0.2)

        self.pose_topic = self.get_parameter("pose_topic").value
        self.goal_topic = self.get_parameter("goal_topic").value
        self.next_place_topic = self.get_parameter("next_place_topic").value
        self.status_topic = self.get_parameter("status_topic").value
        self.command_topic = self.get_parameter("command_topic").value
        self.waypoints_topic = self.get_parameter("waypoints_topic").value
        self.reload_waypoints_topic = self.get_parameter("reload_waypoints_topic").value
        self.publish_period_sec = float(self.get_parameter("publish_period_sec").value)

        self.pose_pub = self.create_publisher(Pose2D, self.pose_topic, 10)
        self.goal_pub = self.create_publisher(Pose2D, self.goal_topic, 10)
        self.next_place_pub = self.create_publisher(String, self.next_place_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.reload_pub = self.create_publisher(Empty, self.reload_waypoints_topic, 10)

        self.create_subscription(String, self.command_topic, self.command_callback, 10)
        self.create_subscription(String, self.waypoints_topic, self.waypoints_callback, 10)

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_yaw = 0.0

        self.next_place_id = ""
        self.status = "idle"

        self.last_command = ""
        self.waypoints: List[Dict[str, Any]] = []

        self.modes = [
            "pose",
            "goal",
            "next_place_id",
            "robot_status",
            "reload_waypoints",
        ]
        self.mode_idx = 0

        self.lock = threading.Lock()

        self.create_timer(self.publish_period_sec, self.publish_all)

        self.input_thread = threading.Thread(target=self.input_loop, daemon=True)
        self.input_thread.start()

        self.print_help()

    def current_mode(self) -> str:
        return self.modes[self.mode_idx]

    def print_help(self):
        self.get_logger().info(
            "dummy_pose_input started\n"
            f"pose_topic={self.pose_topic}\n"
            f"goal_topic={self.goal_topic}\n"
            f"next_place_topic={self.next_place_topic}\n"
            f"status_topic={self.status_topic}\n"
            f"command_topic={self.command_topic}\n"
            f"waypoints_topic={self.waypoints_topic}\n"
            f"reload_waypoints_topic={self.reload_waypoints_topic}\n"
            "\n"
            "사용법:\n"
            "  m      : 입력 대상 토글\n"
            "  show   : 현재 상태 출력\n"
            "  help   : 도움말 출력\n"
            "\n"
            "현재 선택된 항목에 따라 입력 형식:\n"
            "  pose             -> x y yaw\n"
            "  goal             -> x y yaw\n"
            "  next_place_id    -> P003\n"
            "  robot_status     -> patrol / pause / idle / teach\n"
            "  reload_waypoints -> 아무 입력이나 하면 Empty publish\n"
            f"\n현재 선택: {self.current_mode()}"
        )

    def print_state(self):
        with self.lock:
            print(
                "\n[current state]\n"
                f"  mode          = {self.current_mode()}\n"
                f"  pose          = ({self.current_x:.3f}, {self.current_y:.3f}, {self.current_yaw:.3f})\n"
                f"  goal          = ({self.goal_x:.3f}, {self.goal_y:.3f}, {self.goal_yaw:.3f})\n"
                f"  next_place_id = {self.next_place_id}\n"
                f"  robot_status  = {self.status}\n"
                f"  last_command  = {self.last_command}\n"
                f"  n_waypoints   = {len(self.waypoints)}\n"
            )

            if self.waypoints:
                print("[waypoints]")
                for i, wp in enumerate(self.waypoints):
                    print(
                        f"  {i}: place_id={wp.get('place_id')} "
                        f"x={wp.get('x')} y={wp.get('y')} yaw={wp.get('yaw')} "
                        f"patrol_order={wp.get('patrol_order')}"
                    )
                print()

    def toggle_mode(self):
        self.mode_idx = (self.mode_idx + 1) % len(self.modes)
        print(f"[mode] {self.current_mode()}")

    def command_callback(self, msg: String) -> None:
        cmd = msg.data.strip()
        if not cmd:
            return

        status_changed = None

        with self.lock:
            if cmd == self.last_command:
                return

            self.last_command = cmd

            if cmd == "start":
                self.status = "patrol"
                status_changed = self.status
            elif cmd == "pause":
                self.status = "pause"
                status_changed = self.status
            elif cmd == "resume":
                self.status = "patrol"
                status_changed = self.status
            elif cmd == "stop":
                self.status = "idle"
                status_changed = self.status
            elif cmd == "idle":
                self.status = "idle"
                status_changed = self.status
            elif cmd == "teach":
                self.status = "teach"
                status_changed = self.status

        self.get_logger().info(f"[COMMAND] received -> {cmd}")

        if status_changed is not None:
            self.get_logger().info(f"[ROBOT_STATUS] updated by command -> {status_changed}")

    def waypoints_callback(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            places = data.get("places", [])

            if not isinstance(places, list):
                self.get_logger().warning("waypoints_json format invalid: places is not list")
                return

            with self.lock:
                self.waypoints = places

            self.get_logger().info(f"[WAYPOINTS] received -> n={len(places)}")

            for i, wp in enumerate(places):
                self.get_logger().info(
                    f"[WAYPOINTS] {i}: "
                    f"place_id={wp.get('place_id')} "
                    f"x={wp.get('x')} y={wp.get('y')} yaw={wp.get('yaw')} "
                    f"patrol_order={wp.get('patrol_order')}"
                )

        except Exception as e:
            self.get_logger().error(f"failed to parse /patrol/waypoints_json: {e}")

    def input_loop(self):
        while rclpy.ok():
            try:
                raw = input(f"[{self.current_mode()}] 입력 > ").strip()
                if not raw:
                    continue

                if raw.lower() == "m":
                    self.toggle_mode()
                    continue

                if raw.lower() == "show":
                    self.print_state()
                    continue

                if raw.lower() == "help":
                    self.print_help()
                    continue

                mode = self.current_mode()

                if mode == "pose":
                    parts = raw.split()
                    if len(parts) != 3:
                        print("형식 오류: x y yaw")
                        continue

                    x, y, yaw = map(float, parts)
                    with self.lock:
                        self.current_x = x
                        self.current_y = y
                        self.current_yaw = yaw
                    print(f"updated pose -> x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}")

                elif mode == "goal":
                    parts = raw.split()
                    if len(parts) != 3:
                        print("형식 오류: x y yaw")
                        continue

                    x, y, yaw = map(float, parts)
                    with self.lock:
                        self.goal_x = x
                        self.goal_y = y
                        self.goal_yaw = yaw
                    print(f"updated goal -> x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}")

                elif mode == "next_place_id":
                    value = raw.strip()
                    with self.lock:
                        self.next_place_id = value
                    print(f"updated next_place_id -> {value}")

                elif mode == "robot_status":
                    value = raw.strip()
                    with self.lock:
                        self.status = value if value else "idle"
                    print(f"updated robot_status -> {self.status}")

                elif mode == "reload_waypoints":
                    self.reload_pub.publish(Empty())
                    print("published -> /patrol/reload_waypoints (Empty)")

            except Exception as e:
                print(f"input error: {e}")

    def publish_all(self):
        with self.lock:
            current_x = self.current_x
            current_y = self.current_y
            current_yaw = self.current_yaw

            goal_x = self.goal_x
            goal_y = self.goal_y
            goal_yaw = self.goal_yaw

            next_place_id = self.next_place_id
            status = self.status

        pose_msg = Pose2D()
        pose_msg.x = current_x
        pose_msg.y = current_y
        pose_msg.theta = current_yaw

        goal_msg = Pose2D()
        goal_msg.x = goal_x
        goal_msg.y = goal_y
        goal_msg.theta = goal_yaw

        next_place_msg = String()
        next_place_msg.data = next_place_id

        status_msg = String()
        status_msg.data = status

        self.pose_pub.publish(pose_msg)
        self.goal_pub.publish(goal_msg)
        self.next_place_pub.publish(next_place_msg)
        self.status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DummyPoseInputNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()