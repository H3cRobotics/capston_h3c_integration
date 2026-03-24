#!/usr/bin/env python3

import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D


class DummyPoseInputNode(Node):
    def __init__(self):
        super().__init__("dummy_pose_input")

        self.declare_parameter("pose_topic", "/robot_pose")
        self.declare_parameter("publish_period_sec", 0.2)

        self.pose_topic = self.get_parameter("pose_topic").value
        self.publish_period_sec = float(self.get_parameter("publish_period_sec").value)

        self.pub = self.create_publisher(Pose2D, self.pose_topic, 10)

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.lock = threading.Lock()

        self.create_timer(self.publish_period_sec, self.publish_pose)

        self.input_thread = threading.Thread(target=self.input_loop, daemon=True)
        self.input_thread.start()

        self.get_logger().info(
            f"dummy_pose_input started | topic={self.pose_topic}\n"
            f"입력 형식: x y yaw   예) 1.2 3.4 0.5"
        )

    def input_loop(self):
        while rclpy.ok():
            try:
                raw = input("pose 입력 (x y yaw): ").strip()
                if not raw:
                    continue

                parts = raw.split()
                if len(parts) != 3:
                    print("형식 오류: x y yaw 로 입력")
                    continue

                x, y, yaw = map(float, parts)

                with self.lock:
                    self.current_x = x
                    self.current_y = y
                    self.current_yaw = yaw

                print(f"updated -> x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}")

            except Exception as e:
                print(f"input error: {e}")

    def publish_pose(self):
        msg = Pose2D()
        with self.lock:
            msg.x = self.current_x
            msg.y = self.current_y
            msg.theta = self.current_yaw

        self.pub.publish(msg)


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