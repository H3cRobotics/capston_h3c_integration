# patrol_command_bridge.py

import json
from typing import Optional

import requests
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Empty


class PatrolCommandBridge(Node):
    def __init__(self):
        super().__init__("patrol_command_bridge")

        self.declare_parameter("server_url", "http://192.168.0.16:8000")
        self.declare_parameter("waypoints_topic", "/patrol/waypoints_json")
        self.declare_parameter("command_topic", "/patrol/command")
        self.declare_parameter("reload_waypoints_topic", "/patrol/reload_waypoints")
        self.declare_parameter("command_poll_period_sec", 1.0)

        self.server_url = self.get_parameter("server_url").value.rstrip("/")
        self.waypoints_topic = self.get_parameter("waypoints_topic").value
        self.command_topic = self.get_parameter("command_topic").value
        self.reload_waypoints_topic = self.get_parameter("reload_waypoints_topic").value
        self.command_poll_period_sec = float(
            self.get_parameter("command_poll_period_sec").value
        )

        self.waypoints_pub = self.create_publisher(String, self.waypoints_topic, 10)
        self.command_pub = self.create_publisher(String, self.command_topic, 10)

        self.last_waypoints_json: Optional[str] = None
        self.last_command: Optional[str] = None

        self.create_subscription(
            Empty,
            self.reload_waypoints_topic,
            self.reload_waypoints_callback,
            10,
        )

        self.create_timer(self.command_poll_period_sec, self.command_timer_callback)

        self.get_logger().info(
            f"PatrolCommandBridge started | server={self.server_url} | "
            f"waypoints_topic={self.waypoints_topic} | "
            f"command_topic={self.command_topic} | "
            f"reload_waypoints_topic={self.reload_waypoints_topic}"
        )

        self.fetch_and_publish_waypoints()

    def reload_waypoints_callback(self, _msg: Empty) -> None:
        self.get_logger().info("Reload waypoint trigger received")
        self.fetch_and_publish_waypoints()

    def command_timer_callback(self) -> None:
        self.fetch_and_publish_command()

    def fetch_and_publish_waypoints(self) -> None:
        try:
            response = requests.get(
                f"{self.server_url}/robot/patrol_points",
                timeout=3.0,
            )
            response.raise_for_status()

            places = response.json().get("places", [])
            payload = {"places": places}
            payload_json = json.dumps(payload, ensure_ascii=False, sort_keys=True)

            if payload_json == self.last_waypoints_json:
                self.get_logger().info("Waypoint unchanged, skip publish")
                return

            msg = String()
            msg.data = payload_json
            self.waypoints_pub.publish(msg)

            self.last_waypoints_json = payload_json

            self.get_logger().info(f"Published patrol waypoints | n={len(places)}")

        except requests.Timeout:
            self.get_logger().warning("Timeout while fetching patrol waypoints")

        except requests.HTTPError as e:
            code = e.response.status_code if e.response is not None else "unknown"
            text = e.response.text if e.response is not None else "no response body"
            self.get_logger().error(f"Waypoint HTTP error: {code} | {text}")

        except requests.RequestException as e:
            self.get_logger().error(f"Waypoint request failed: {e}")

        except Exception as e:
            self.get_logger().error(f"Waypoint bridge failed: {e}")

    def fetch_and_publish_command(self) -> None:
        try:
            response = requests.get(
                f"{self.server_url}/robot/command",
                timeout=2.0,
            )
            response.raise_for_status()

            command = response.json().get("command")
            if command is None:
                return

            command = str(command).strip()
            if not command:
                return

            if command == self.last_command:
                return

            msg = String()
            msg.data = command
            self.command_pub.publish(msg)
            self.last_command = command

            self.get_logger().info(f"Published patrol command | command={command}")

            if command == "reload_waypoints":
                self.fetch_and_publish_waypoints()

        except requests.Timeout:
            self.get_logger().warning("Timeout while fetching patrol command")

        except requests.HTTPError as e:
            code = e.response.status_code if e.response is not None else "unknown"
            text = e.response.text if e.response is not None else "no response body"
            self.get_logger().error(f"Command HTTP error: {code} | {text}")

        except requests.RequestException as e:
            self.get_logger().error(f"Command request failed: {e}")

        except Exception as e:
            self.get_logger().error(f"Command bridge failed: {e}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PatrolCommandBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()