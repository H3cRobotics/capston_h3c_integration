# robot_goal_sender.py

from typing import Optional
from datetime import datetime

import requests
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String


class RobotGoalSender(Node):
    def __init__(self):
        super().__init__("robot_goal_sender")

        self.declare_parameter("server_url", "http://192.168.0.16:8000")
        self.declare_parameter("goal_topic", "/goal_pose_2d")
        self.declare_parameter("next_place_topic", "/next_place_id")

        self.server_url = self.get_parameter("server_url").value.rstrip("/")
        self.goal_topic = self.get_parameter("goal_topic").value
        self.next_place_topic = self.get_parameter("next_place_topic").value

        self.goal_x: Optional[float] = None
        self.goal_y: Optional[float] = None
        self.goal_yaw: Optional[float] = None
        self.next_place_id: Optional[str] = None

        self.last_sent = {
            "x": None,
            "y": None,
            "yaw": None,
            "next_place_id": None,
        }

        self.create_subscription(
            Pose2D,
            self.goal_topic,
            self.goal_callback,
            10,
        )

        self.create_subscription(
            String,
            self.next_place_topic,
            self.next_place_callback,
            10,
        )

        self.get_logger().info(
            f"RobotGoalSender started | "
            f"goal_topic={self.goal_topic} | "
            f"next_place_topic={self.next_place_topic} | "
            f"endpoint={self.server_url}/robot/goal"
        )

    def goal_callback(self, msg: Pose2D) -> None:
        self.goal_x = float(msg.x)
        self.goal_y = float(msg.y)
        self.goal_yaw = float(msg.theta)
        self.try_send_goal()

    def next_place_callback(self, msg: String) -> None:
        value = msg.data.strip()
        self.next_place_id = value if value else None
        self.try_send_goal()

    def try_send_goal(self) -> None:
        if self.goal_x is None or self.goal_y is None or self.goal_yaw is None:
            return

        payload = {
            "x": self.goal_x,
            "y": self.goal_y,
            "yaw": self.goal_yaw,
            "next_place_id": self.next_place_id,
            "timestamp": datetime.now().isoformat(),
        }

        compare_payload = {
            "x": payload["x"],
            "y": payload["y"],
            "yaw": payload["yaw"],
            "next_place_id": payload["next_place_id"],
        }

        if compare_payload == self.last_sent:
            return

        try:
            response = requests.post(
                f"{self.server_url}/robot/goal",
                json=payload,
                timeout=2.0,
            )
            response.raise_for_status()

            self.last_sent = compare_payload.copy()

            self.get_logger().info(
                f"Sent goal | "
                f"x={payload['x']:.3f}, y={payload['y']:.3f}, "
                f"yaw={payload['yaw']:.3f}, next_place_id={payload['next_place_id']}"
            )

        except requests.Timeout:
            self.get_logger().warning("Timeout while sending robot goal")

        except requests.HTTPError as e:
            code = e.response.status_code if e.response is not None else "unknown"
            text = e.response.text if e.response is not None else "no response body"
            self.get_logger().error(
                f"HTTP error while sending robot goal: {code} | {text}"
            )

        except requests.RequestException as e:
            self.get_logger().error(f"Request failed while sending robot goal: {e}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RobotGoalSender()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()