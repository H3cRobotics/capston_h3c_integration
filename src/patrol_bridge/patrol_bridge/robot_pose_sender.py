#robot_pose_sender.py

from typing import Optional

import requests
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String


class RobotPoseSender(Node):
    def __init__(self):
        super().__init__("robot_pose_sender")

        self.declare_parameter("server_url", "http://192.168.0.16:8000")
        self.declare_parameter("pose_topic", "/robot_pose")
        self.declare_parameter("status_topic", "/robot_status")
        self.declare_parameter("post_period_sec", 1.0)

        self.server_url = self.get_parameter("server_url").value.rstrip("/")
        self.pose_topic = self.get_parameter("pose_topic").value
        self.status_topic = self.get_parameter("status_topic").value
        self.post_period_sec = float(self.get_parameter("post_period_sec").value)

        self.x: Optional[float] = None
        self.y: Optional[float] = None
        self.yaw: Optional[float] = None
        self.status: str = "idle"

        self.create_subscription(
            Pose2D,
            self.pose_topic,
            self.pose_callback,
            10,
        )

        self.create_subscription(
            String,
            self.status_topic,
            self.status_callback,
            10,
        )

        self.create_timer(self.post_period_sec, self.timer_callback)

        self.get_logger().info(
            f"RobotPoseSender started | topic={self.pose_topic} | "
            f"status_topic={self.status_topic} | "
            f"endpoint={self.server_url}/robot/pose"
        )

    def pose_callback(self, msg: Pose2D) -> None:
        self.x = float(msg.x)
        self.y = float(msg.y)
        self.yaw = float(msg.theta)

    def status_callback(self, msg: String) -> None:
        value = msg.data.strip()
        self.status = value if value else "idle"

    def timer_callback(self) -> None:
        if self.x is None or self.y is None or self.yaw is None:
            return

        payload = {
            "x": self.x,
            "y": self.y,
            "yaw": self.yaw,
            "status": self.status,
        }

        try:
            response = requests.post(
                f"{self.server_url}/robot/pose",
                json=payload,
                timeout=2.0,
            )
            response.raise_for_status()

        except requests.Timeout:
            self.get_logger().warning("Timeout while sending robot pose")

        except requests.HTTPError as e:
            code = e.response.status_code if e.response is not None else "unknown"
            text = e.response.text if e.response is not None else "no response body"
            self.get_logger().error(
                f"HTTP error while sending robot pose: {code} | {text}"
            )

        except requests.RequestException as e:
            self.get_logger().error(f"Request failed while sending robot pose: {e}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RobotPoseSender()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()