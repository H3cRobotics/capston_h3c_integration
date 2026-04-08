import threading
from typing import List, Dict, Any, Optional, Tuple

import uvicorn
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel, Field

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from geometry_msgs.msg import Pose2D


class YoloRegion(BaseModel):
    region_id: Optional[int] = None
    name: str
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    is_enabled: bool = True


class YoloConfigReq(BaseModel):
    yolo_mode: int = Field(..., description="0: OFF, 1: GLOBAL, 2: REGION")
    run_yolo: bool
    regions: List[YoloRegion] = Field(default_factory=list)
    updated_at: Optional[str] = None


class PersonDetectControlNode(Node):
    def __init__(self):
        super().__init__("person_detect_control_node")

        # ROS topics
        self.declare_parameter("robot_pose_topic", "/robot_pose")
        self.declare_parameter("enable_topic", "/person_tracking/enable")

        # local http server
        self.declare_parameter("notify_host", "0.0.0.0")
        self.declare_parameter("notify_port", 8091)

        # behavior
        self.declare_parameter("start_enabled", False)
        self.declare_parameter("fail_safe_disable", True)
        self.declare_parameter("log_region_match", True)

        self.robot_pose_topic = str(self.get_parameter("robot_pose_topic").value)
        self.enable_topic = str(self.get_parameter("enable_topic").value)

        self.notify_host = str(self.get_parameter("notify_host").value)
        self.notify_port = int(self.get_parameter("notify_port").value)

        self.start_enabled = bool(self.get_parameter("start_enabled").value)
        self.fail_safe_disable = bool(self.get_parameter("fail_safe_disable").value)
        self.log_region_match = bool(self.get_parameter("log_region_match").value)

        self.lock = threading.Lock()

        # current robot pose
        self.current_pose: Optional[Pose2D] = None

        # pushed config cache
        self.yolo_mode = 0                 # 0: OFF, 1: GLOBAL, 2: REGION
        self.run_yolo_hint = False
        self.enabled_regions: List[Dict[str, Any]] = []
        self.last_config_ok = False
        self.last_config_updated_at: Optional[str] = None

        # output state
        self.last_enable: Optional[bool] = None
        self.last_region_name: Optional[str] = None

        self.pose_sub = self.create_subscription(
            Pose2D,
            self.robot_pose_topic,
            self.pose_callback,
            10
        )

        self.enable_pub = self.create_publisher(Bool, self.enable_topic, 10)

        self.eval_timer = self.create_timer(0.2, self.evaluate_enable)

        self.app = FastAPI()
        self._setup_routes()

        self.http_thread = threading.Thread(
            target=self._run_http_server,
            daemon=True
        )
        self.http_thread.start()

        self.publish_enable(self.start_enabled, reason="startup")

        self.get_logger().info(
            f"[INIT] PersonDetectControlNode | "
            f"robot_pose={self.robot_pose_topic} | "
            f"enable_topic={self.enable_topic} | "
            f"notify_http={self.notify_host}:{self.notify_port}"
        )

    # =========================
    # HTTP
    # =========================

    def _setup_routes(self):
        node = self

        @self.app.post("/robot/yolo_config")
        async def post_yolo_config(req: YoloConfigReq):
            if req.yolo_mode not in [0, 1, 2]:
                raise HTTPException(status_code=400, detail="invalid yolo_mode")

            # normalize regions
            normalized_regions: List[Dict[str, Any]] = []
            for r in req.regions:
                x1 = min(r.x_min, r.x_max)
                x2 = max(r.x_min, r.x_max)
                y1 = min(r.y_min, r.y_max)
                y2 = max(r.y_min, r.y_max)

                normalized_regions.append(
                    {
                        "region_id": r.region_id,
                        "name": r.name,
                        "x_min": x1,
                        "x_max": x2,
                        "y_min": y1,
                        "y_max": y2,
                        "is_enabled": bool(r.is_enabled),
                    }
                )

            with node.lock:
                node.yolo_mode = int(req.yolo_mode)
                node.run_yolo_hint = bool(req.run_yolo)
                node.enabled_regions = normalized_regions
                node.last_config_ok = True
                node.last_config_updated_at = req.updated_at

            node.get_logger().info(
                f"[YOLO CONFIG] mode={req.yolo_mode} "
                f"run_yolo={req.run_yolo} "
                f"regions={len(normalized_regions)} "
                f"updated_at={req.updated_at}"
            )

            node.evaluate_enable()

            return {
                "ok": True,
                "applied": True,
                "cached_mode": node.yolo_mode,
                "cached_region_count": len(normalized_regions),
            }

        @self.app.get("/health")
        async def health():
            return {"ok": True}

    def _run_http_server(self):
        uvicorn.run(
            self.app,
            host=self.notify_host,
            port=self.notify_port,
            log_level="info"
        )

    # =========================
    # ROS callbacks
    # =========================

    def pose_callback(self, msg: Pose2D):
        with self.lock:
            self.current_pose = msg

    # =========================
    # Region logic
    # =========================

    def point_in_region(self, x: float, y: float, region: Dict[str, Any]) -> bool:
        try:
            x_min = float(region["x_min"])
            x_max = float(region["x_max"])
            y_min = float(region["y_min"])
            y_max = float(region["y_max"])
        except Exception:
            return False

        if x_min > x_max:
            x_min, x_max = x_max, x_min
        if y_min > y_max:
            y_min, y_max = y_max, y_min

        return (x_min <= x <= x_max) and (y_min <= y <= y_max)

    def find_matching_region(self, x: float, y: float) -> Optional[Dict[str, Any]]:
        with self.lock:
            regions = list(self.enabled_regions)

        for region in regions:
            if int(region.get("is_enabled", 1)) != 1:
                continue
            if self.point_in_region(x, y, region):
                return region
        return None

    # =========================
    # Enable logic
    # =========================

    def compute_enable(self) -> Tuple[bool, Optional[str], str]:
        with self.lock:
            pose = self.current_pose
            yolo_mode = self.yolo_mode
            run_yolo_hint = self.run_yolo_hint
            config_ok = self.last_config_ok

        if not config_ok:
            if self.fail_safe_disable:
                return False, None, "config_not_received"
            return (
                bool(self.last_enable) if self.last_enable is not None else False,
                None,
                "config_not_received_hold",
            )

        if yolo_mode == 0:
            return False, None, "mode_off"

        if yolo_mode == 1:
            return True, None, "mode_global"

        if yolo_mode == 2:
            if not run_yolo_hint:
                return False, None, "mode_region_no_active_region"

            if pose is None:
                return False, None, "no_robot_pose"

            x = float(pose.x)
            y = float(pose.y)

            region = self.find_matching_region(x, y)
            if region is None:
                return False, None, "outside_all_regions"

            region_name = str(
                region.get("name", f"region_{region.get('region_id', 'unknown')}")
            )
            return True, region_name, "inside_enabled_region"

        return False, None, "unknown_mode"

    def evaluate_enable(self):
        enable, region_name, reason = self.compute_enable()

        if self.last_enable is None or self.last_enable != enable:
            self.publish_enable(enable, reason=reason)

        if self.log_region_match and enable and region_name != self.last_region_name:
            self.last_region_name = region_name
            if region_name is not None:
                self.get_logger().info(f"[REGION] matched: {region_name}")

        if not enable:
            self.last_region_name = None

    def publish_enable(self, enable: bool, reason: str = ""):
        msg = Bool()
        msg.data = bool(enable)
        self.enable_pub.publish(msg)
        self.last_enable = bool(enable)

        self.get_logger().info(f"[YOLO ENABLE] {enable} reason={reason}")


def main(args=None):
    rclpy.init(args=args)
    node = PersonDetectControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()