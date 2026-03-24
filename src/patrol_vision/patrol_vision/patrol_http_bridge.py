#!/usr/bin/env python3
from __future__ import annotations

import threading

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Empty
from fastapi import FastAPI
from pydantic import BaseModel
import uvicorn


class PlaceReq(BaseModel):
    place_id: str


class QueryGtReq(BaseModel):
    label: str


class PatrolHttpBridge(Node):

    def __init__(self):
        super().__init__("patrol_http_bridge")

        self.place_pub = self.create_publisher(
            String,
            "/patrol/current_place",
            10
        )

        self.trigger_pub = self.create_publisher(
            Empty,
            "/patrol/capture_trigger",
            10
        )

        self.query_gt_pub = self.create_publisher(
            String,
            "/patrol/query_gt",
            10
        )

        self.app = FastAPI()

        @self.app.get("/health")
        def health():
            return {"ok": True}

        @self.app.post("/patrol/place")
        def set_place(req: PlaceReq):
            msg = String()
            msg.data = str(req.place_id)
            self.place_pub.publish(msg)

            self.get_logger().info(f"place -> {msg.data}")
            return {"ok": True, "place_id": msg.data}

        @self.app.post("/patrol/query_gt")
        def set_query_gt(req: QueryGtReq):
            label = str(req.label).strip().lower()

            if label not in ["normal", "abnormal"]:
                return {"ok": False, "error": "label must be normal or abnormal"}

            msg = String()
            msg.data = label
            self.query_gt_pub.publish(msg)

            self.get_logger().info(f"query_gt -> {msg.data}")
            return {"ok": True, "query_gt": msg.data}

        @self.app.post("/patrol/capture")
        def trigger_capture():
            self.trigger_pub.publish(Empty())

            self.get_logger().info("capture trigger")
            return {"ok": True}

        @self.app.post("/patrol/place_and_capture")
        def place_and_capture(req: PlaceReq):
            msg = String()
            msg.data = str(req.place_id)
            self.place_pub.publish(msg)
            self.trigger_pub.publish(Empty())

            self.get_logger().info(f"place -> {msg.data}, capture trigger")
            return {"ok": True, "place_id": msg.data}

    def run_http(self, host="0.0.0.0", port=8090):
        uvicorn.run(self.app, host=host, port=port, log_level="info")


def main():
    rclpy.init()

    node = PatrolHttpBridge()

    t = threading.Thread(
        target=node.run_http,
        kwargs={"host": "0.0.0.0", "port": 8090},
        daemon=True
    )
    t.start()

    node.get_logger().info("HTTP bridge ready at :8090")
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()