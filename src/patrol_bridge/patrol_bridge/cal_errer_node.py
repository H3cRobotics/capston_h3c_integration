# cal_errer_node.py

import csv
import math
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String


def normalize_angle(rad: float) -> float:
    while rad > math.pi:
        rad -= 2.0 * math.pi
    while rad < -math.pi:
        rad += 2.0 * math.pi
    return rad


class CalErrerNode(Node):
    def __init__(self):
        super().__init__("cal_errer_node")

        self.declare_parameter("goal_topic", "/goal_pose_2d")
        self.declare_parameter("pose_topic", "/robot_pose")
        self.declare_parameter("next_place_topic", "/next_place_id")
        self.declare_parameter("status_topic", "/robot_status")

        self.declare_parameter(
            "csv_dir",
            "/home/choisuhyun/scene_ad_for_patrol_robot/control_eval_logs",
        )

        self.goal_topic = self.get_parameter("goal_topic").value
        self.pose_topic = self.get_parameter("pose_topic").value
        self.next_place_topic = self.get_parameter("next_place_topic").value
        self.status_topic = self.get_parameter("status_topic").value

        csv_dir = Path(self.get_parameter("csv_dir").value)
        run_tag = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = csv_dir / f"control_error_log_{run_tag}.csv"

        self.current_place_id: Optional[str] = None
        self.current_status: str = "unknown"

        self.goal_x: Optional[float] = None
        self.goal_y: Optional[float] = None
        self.goal_yaw: Optional[float] = None

        self.pose_x: Optional[float] = None
        self.pose_y: Optional[float] = None
        self.pose_yaw: Optional[float] = None

        self.active = False
        self.start_time_sec: Optional[float] = None
        self.start_time_str: Optional[str] = None

        self.sample_count = 0
        self.min_xy_error = float("inf")
        self.min_yaw_error = float("inf")
        self.sum_xy_error = 0.0
        self.sum_yaw_error = 0.0

        self.final_xy_error: Optional[float] = None
        self.final_yaw_error: Optional[float] = None

        self.path_length = 0.0
        self.prev_pose_x: Optional[float] = None
        self.prev_pose_y: Optional[float] = None

        self._prepare_csv()

        self.create_subscription(Pose2D, self.goal_topic, self.goal_callback, 10)
        self.create_subscription(Pose2D, self.pose_topic, self.pose_callback, 10)
        self.create_subscription(String, self.next_place_topic, self.next_place_callback, 10)
        self.create_subscription(String, self.status_topic, self.status_callback, 10)

        self.get_logger().info(
            f"cal_errer_node started | "
            f"goal={self.goal_topic}, pose={self.pose_topic}, "
            f"next_place={self.next_place_topic}, csv={self.csv_path}"
        )

    def _prepare_csv(self):
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)

        with open(self.csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    "place_id",
                    "finish_trigger",
                    "start_time",
                    "duration_sec",
                    "goal_x",
                    "goal_y",
                    "goal_yaw_rad",
                    "final_x",
                    "final_y",
                    "final_yaw_rad",
                    "final_xy_error_m",
                    "final_yaw_error_deg",
                    "min_xy_error_m",
                    "min_yaw_error_deg",
                    "mean_xy_error_m",
                    "mean_yaw_error_deg",
                    "path_length_m",
                    "sample_count",
                    "status",
                ]
            )

    def next_place_callback(self, msg: String):
        new_place_id = msg.data.strip() or None

        if self.active and new_place_id != self.current_place_id:
            self.finish_goal(f"NEXT_PLACE_RECEIVED:{new_place_id}")

        self.current_place_id = new_place_id

    def status_callback(self, msg: String):
        self.current_status = msg.data.strip() or "unknown"

    def goal_callback(self, msg: Pose2D):
        if self.active:
            self.finish_goal("NEW_GOAL_RECEIVED")

        self.goal_x = float(msg.x)
        self.goal_y = float(msg.y)
        self.goal_yaw = float(msg.theta)

        self.active = True
        self.start_time_sec = time.time()
        self.start_time_str = time.strftime("%Y-%m-%d %H:%M:%S")

        self.sample_count = 0
        self.min_xy_error = float("inf")
        self.min_yaw_error = float("inf")
        self.sum_xy_error = 0.0
        self.sum_yaw_error = 0.0
        self.final_xy_error = None
        self.final_yaw_error = None

        self.path_length = 0.0
        self.prev_pose_x = self.pose_x
        self.prev_pose_y = self.pose_y

        self.get_logger().info(
            f"[START] place={self.current_place_id} | "
            f"goal=({self.goal_x:.3f}, {self.goal_y:.3f}, {self.goal_yaw:.3f})"
        )

    def pose_callback(self, msg: Pose2D):
        new_x = float(msg.x)
        new_y = float(msg.y)
        new_yaw = float(msg.theta)

        if self.active and self.prev_pose_x is not None and self.prev_pose_y is not None:
            step_dist = math.hypot(
                new_x - self.prev_pose_x,
                new_y - self.prev_pose_y,
            )
            self.path_length += step_dist

        self.pose_x = new_x
        self.pose_y = new_y
        self.pose_yaw = new_yaw

        self.prev_pose_x = new_x
        self.prev_pose_y = new_y

        if not self.active:
            return

        xy_error, yaw_error = self.compute_error()

        self.final_xy_error = xy_error
        self.final_yaw_error = yaw_error

        self.min_xy_error = min(self.min_xy_error, xy_error)
        self.min_yaw_error = min(self.min_yaw_error, yaw_error)

        self.sum_xy_error += xy_error
        self.sum_yaw_error += yaw_error
        self.sample_count += 1

    def compute_error(self):
        if (
            self.goal_x is None
            or self.goal_y is None
            or self.goal_yaw is None
            or self.pose_x is None
            or self.pose_y is None
            or self.pose_yaw is None
        ):
            return None, None

        dx = self.goal_x - self.pose_x
        dy = self.goal_y - self.pose_y
        xy_error = math.hypot(dx, dy)

        yaw_error = abs(normalize_angle(self.goal_yaw - self.pose_yaw))

        return xy_error, yaw_error

    def finish_goal(self, trigger: str):
        if not self.active:
            return

        xy_error, yaw_error = self.compute_error()
        if xy_error is not None and yaw_error is not None:
            self.final_xy_error = xy_error
            self.final_yaw_error = yaw_error

        duration = time.time() - self.start_time_sec if self.start_time_sec else 0.0

        mean_xy_error = (
            self.sum_xy_error / self.sample_count
            if self.sample_count > 0
            else None
        )

        mean_yaw_error = (
            self.sum_yaw_error / self.sample_count
            if self.sample_count > 0
            else None
        )

        row = [
            self.current_place_id,
            trigger,
            self.start_time_str,
            round(duration, 3),
            self.goal_x,
            self.goal_y,
            self.goal_yaw,
            self.pose_x,
            self.pose_y,
            self.pose_yaw,
            self.final_xy_error,
            math.degrees(self.final_yaw_error)
            if self.final_yaw_error is not None
            else None,
            self.min_xy_error if self.min_xy_error != float("inf") else None,
            math.degrees(self.min_yaw_error)
            if self.min_yaw_error != float("inf")
            else None,
            mean_xy_error,
            math.degrees(mean_yaw_error)
            if mean_yaw_error is not None
            else None,
            self.path_length,
            self.sample_count,
            self.current_status,
        ]

        with open(self.csv_path, "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(row)

        final_yaw_deg = (
            math.degrees(self.final_yaw_error)
            if self.final_yaw_error is not None
            else None
        )

        self.get_logger().info(
            f"[FINISH] place={self.current_place_id} | "
            f"trigger={trigger} | "
            f"duration={duration:.2f}s | "
            f"final_xy={self.final_xy_error} | "
            f"final_yaw_deg={final_yaw_deg}"
        )

        self.active = False


def main(args=None):
    rclpy.init(args=args)
    node = CalErrerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.active:
            node.finish_goal("SHUTDOWN")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()