#teacher_node.py

import re
from typing import Optional

import requests
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Empty


class RobotTeachSender(Node):
    def __init__(self):
        super().__init__("teacher_node")

        self.declare_parameter("server_url", "http://192.168.0.16:8000")
        self.declare_parameter("robot_pose_topic", "/robot_pose")
        self.declare_parameter("teach_trigger_topic", "/teach_trigger")
        self.declare_parameter("default_patrol_enabled", True)
        self.declare_parameter("place_prefix", "P")

        self.server_url = self.get_parameter("server_url").value.rstrip("/")
        self.robot_pose_topic = self.get_parameter("robot_pose_topic").value
        self.teach_trigger_topic = self.get_parameter("teach_trigger_topic").value
        self.default_patrol_enabled = bool(self.get_parameter("default_patrol_enabled").value)
        self.place_prefix = self.get_parameter("place_prefix").value or "P"

        self.x: Optional[float] = None
        self.y: Optional[float] = None
        self.yaw: Optional[float] = None

        self.teach_in_progress = False

        self.create_subscription(
            Pose2D,
            self.robot_pose_topic,
            self.robot_pose_callback,
            10,
        )

        self.create_subscription(
            Empty,
            self.teach_trigger_topic,
            self.teach_trigger_callback,
            10,
        )

        self.get_logger().info(
            f"RobotTeachSender started | pose_topic={self.robot_pose_topic} | "
            f"teach_trigger_topic={self.teach_trigger_topic} | server={self.server_url}"
        )

    def robot_pose_callback(self, msg: Pose2D) -> None:

        self.x = float(msg.x)
        self.y = float(msg.y)
        self.yaw = float(msg.theta)

    def get_next_place_id(self) -> str:
        response = requests.get(f"{self.server_url}/places", timeout=3.0)
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

    def teach_trigger_callback(self, _msg: Empty) -> None:
        self.get_logger().info(
        f"🔥 Teach trigger received | pose=({self.x}, {self.y}, {self.yaw})"
        )
        if self.teach_in_progress:
            self.get_logger().warning("Teach trigger ignored: already in progress")
            return

        if self.x is None or self.y is None or self.yaw is None:
            self.get_logger().warning("Teach requested but robot pose is not ready")
            return

        self.teach_in_progress = True

        try:
            place_id = self.get_next_place_id()

            payload = {
                "place_id": place_id,
                "x": self.x,
                "y": self.y,
                "yaw": self.yaw,
                "display_name": place_id,
                "patrol_enabled": self.default_patrol_enabled,
            }

            response = requests.post(
                f"{self.server_url}/robot/teach",
                json=payload,
                timeout=3.0,
            )
            response.raise_for_status()

            self.get_logger().info(
                f"Teach success | place_id={place_id} | "
                f"x={self.x:.3f}, y={self.y:.3f}, yaw={self.yaw:.3f}"
            )

        except requests.Timeout:
            self.get_logger().error("Teach timeout")

        except requests.HTTPError as e:
            code = e.response.status_code if e.response is not None else "unknown"
            text = e.response.text if e.response is not None else "no response body"
            self.get_logger().error(f"Teach HTTP error: {code} | {text}")

        except requests.RequestException as e:
            self.get_logger().error(f"Teach request failed: {e}")

        except Exception as e:
            self.get_logger().error(f"Teach failed: {e}")

        finally:
            self.teach_in_progress = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RobotTeachSender()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()