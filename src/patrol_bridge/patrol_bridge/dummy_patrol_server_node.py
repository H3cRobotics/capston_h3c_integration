# dummy_patrol_server_node.py

import json
import sys
import tty
import termios
import select
import threading
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String

from fastapi import FastAPI, File, Form, HTTPException, UploadFile
from fastapi.responses import JSONResponse
from pydantic import BaseModel
import uvicorn

class TeachRequest(BaseModel):
    place_id: str
    x: float
    y: float
    yaw: float
    display_name: Optional[str] = None
    patrol_enabled: bool = True

class DummyPatrolServerNode(Node):
    def __init__(self):
        super().__init__("dummy_patrol_server_node")

        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8000)
        self.declare_parameter("reload_waypoints_topic", "/reload_waypoints")
        self.declare_parameter("waypoints_topic", "/waypoints_json")
        self.declare_parameter("publish_on_teach", False)
        self.declare_parameter("enable_keyboard_clear", True)
        self.declare_parameter("save_root", "./recv_dummy")

        self.host = str(self.get_parameter("host").value)
        self.port = int(self.get_parameter("port").value)
        self.reload_waypoints_topic = str(
            self.get_parameter("reload_waypoints_topic").value
        )
        self.waypoints_topic = str(self.get_parameter("waypoints_topic").value)
        self.publish_on_teach = bool(self.get_parameter("publish_on_teach").value)
        self.enable_keyboard_clear = bool(
            self.get_parameter("enable_keyboard_clear").value
        )
        self.save_root = Path(str(self.get_parameter("save_root").value)).expanduser()

        self._lock = threading.Lock()
        self._places: List[Dict[str, Any]] = []
        self._stop_event = threading.Event()

        self.save_root.mkdir(parents=True, exist_ok=True)

        self.waypoints_pub = self.create_publisher(String, self.waypoints_topic, 10)

        self.create_subscription(
            Empty,
            self.reload_waypoints_topic,
            self.reload_waypoints_callback,
            10,
        )

        self.app = FastAPI()
        self._setup_routes()

        self._server_thread = threading.Thread(
            target=self._run_http_server,
            daemon=True,
        )
        self._server_thread.start()

        self._keyboard_thread = None
        if self.enable_keyboard_clear:
            self._keyboard_thread = threading.Thread(
                target=self._keyboard_loop,
                daemon=True,
            )
            self._keyboard_thread.start()

        self.get_logger().info(
            f"[START] DummyPatrolServerNode | "
            f"http://{self.host}:{self.port} | "
            f"reload_topic={self.reload_waypoints_topic} | "
            f"waypoints_topic={self.waypoints_topic} | "
            f"publish_on_teach={self.publish_on_teach} | "
            f"keyboard_clear={self.enable_keyboard_clear} | "
            f"save_root={self.save_root}"
        )
        if self.enable_keyboard_clear:
            self.get_logger().info(
                "[KEY] Press 'c' or 'C' in this terminal to clear all waypoints"
            )

    # -------------------------
    # ROS callbacks
    # -------------------------
    def reload_waypoints_callback(self, _msg: Empty) -> None:
        self.get_logger().info("[RECV][ROS] reload trigger received")
        self.publish_waypoints()

    # -------------------------
    # Core ops
    # -------------------------
    def clear_all_waypoints(self, reason: str = "unknown") -> int:
        with self._lock:
            removed_ids = [p["place_id"] for p in self._places]
            count = len(self._places)
            self._places.clear()

        self.get_logger().warning(
            f"[CLEAR] reason={reason} | removed={count} | place_ids={removed_ids}"
        )
        return count

    def publish_waypoints(self) -> None:
        with self._lock:
            places = [
                {
                    "place_id": p["place_id"],
                    "display_name": p.get("display_name", p["place_id"]),
                    "x": float(p["x"]),
                    "y": float(p["y"]),
                    "yaw": float(p["yaw"]),
                    "patrol_enabled": bool(p.get("patrol_enabled", True)),
                }
                for p in self._places
                if p.get("patrol_enabled", True)
            ]

        places = sorted(places, key=lambda x: x["place_id"])

        payload = {"places": places}

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.waypoints_pub.publish(msg)

        ids = [p["place_id"] for p in places]
        self.get_logger().info(
            f"[SEND][ROS] published waypoints -> topic={self.waypoints_topic} | "
            f"count={len(places)} | place_ids={ids}"
        )

    # -------------------------
    # HTTP routes
    # -------------------------
    def _setup_routes(self) -> None:
        node = self

        @self.app.get("/places")
        def get_places():
            with node._lock:
                places = [
                    {
                        "place_id": p["place_id"],
                        "display_name": p.get("display_name", p["place_id"]),
                        "patrol_enabled": p.get("patrol_enabled", True),
                    }
                    for p in node._places
                ]

            node.get_logger().info(f"[SEND][HTTP] GET /places | count={len(places)}")
            return {"ok": True, "places": places}

        @self.app.post("/robot/teach")
        def robot_teach(req: TeachRequest):
            node.get_logger().info(
                f"[RECV][HTTP] POST /robot/teach | "
                f"place_id={req.place_id} | x={req.x:.3f}, y={req.y:.3f}, yaw={req.yaw:.3f}"
            )

            place = {
                "place_id": req.place_id,
                "display_name": req.display_name or req.place_id,
                "x": float(req.x),
                "y": float(req.y),
                "yaw": float(req.yaw),
                "patrol_enabled": bool(req.patrol_enabled),
            }

            with node._lock:
                updated = False
                for i, old in enumerate(node._places):
                    if old["place_id"] == req.place_id:
                        node._places[i] = place
                        updated = True
                        break
                if not updated:
                    node._places.append(place)

                total = len(node._places)

            node.get_logger().info(
                f"[SAVE] place_id={req.place_id} | updated={updated} | total={total}"
            )

            if node.publish_on_teach:
                node.publish_waypoints()

            return {
                "ok": True,
                "saved": place,
                "updated": updated,
                "total": total,
            }

        @self.app.get("/robot/patrol_points")
        def get_patrol_points():
            with node._lock:
                places = [
                    {
                        "place_id": p["place_id"],
                        "display_name": p.get("display_name", p["place_id"]),
                        "x": float(p["x"]),
                        "y": float(p["y"]),
                        "yaw": float(p["yaw"]),
                        "patrol_enabled": bool(p.get("patrol_enabled", True)),
                    }
                    for p in node._places
                    if p.get("patrol_enabled", True)
                ]

            places = sorted(places, key=lambda x: x["place_id"])
            ids = [p["place_id"] for p in places]

            node.get_logger().info(
                f"[SEND][HTTP] GET /robot/patrol_points | count={len(places)} | place_ids={ids}"
            )
            return {"ok": True, "places": places}

        @self.app.get("/debug/state")
        def debug_state():
            with node._lock:
                places = list(node._places)

            node.get_logger().info(
                f"[SEND][HTTP] GET /debug/state | count={len(places)}"
            )
            return {
                "ok": True,
                "places": places,
            }

        # ----------------------------------------------------------
        # 이미지를 받은 경우 endpoint
        # 실서버처럼 파일명 규칙만 맞춰서 저장만 수행
        # ----------------------------------------------------------
        @self.app.post("/place_imgs")
        async def place_imgs(
            images: List[UploadFile] = File(...),
            meta: str = Form(...),
        ):
            node.get_logger().info(
                f"[RECV][HTTP] POST /place_imgs | n_images={len(images)}"
            )

            try:
                meta_obj = json.loads(meta)
            except Exception:
                raise HTTPException(status_code=400, detail="meta must be valid json string")

            place_id = str(meta_obj.get("place_id", "unknown"))
            mode = str(meta_obj.get("mode", "unknown"))
            label = meta_obj.get("label", None)
            ts = meta_obj.get("timestamp") or datetime.now().isoformat()

            if mode not in ("bank", "th_calib", "query"):
                raise HTTPException(
                    status_code=400,
                    detail="mode must be one of bank/th_calib/query"
                )

            safe_ts = ts.replace(":", "-")
            out_dir = node.save_root / place_id / mode
            out_dir.mkdir(parents=True, exist_ok=True)

            if label is None:
                prefix = f"{place_id}_{mode}_{safe_ts}"
            else:
                prefix = f"{label}_{place_id}_{mode}_{safe_ts}"

            saved = []

            for i, uf in enumerate(images):
                ext = Path(uf.filename).suffix.lower() or ".jpg"
                out_path = out_dir / f"{prefix}_{i:03d}{ext}"

                data = await uf.read()
                out_path.write_bytes(data)
                saved.append(str(out_path))

            node.get_logger().info(
                f"[SAVE][IMG] place_id={place_id} | mode={mode} | "
                f"saved_dir={out_dir} | n_saved={len(saved)}"
            )

            return JSONResponse(
                {
                    "ok": True,
                    "status": "saved",
                    "n_images": len(saved),
                    "saved_dir": str(out_dir),
                    "meta": meta_obj,
                }
            )

    # -------------------------
    # HTTP server
    # -------------------------
    def _run_http_server(self) -> None:
        self.get_logger().info(
            f"[START][HTTP] uvicorn starting on http://{self.host}:{self.port}"
        )
        config = uvicorn.Config(
            self.app,
            host=self.host,
            port=self.port,
            log_level="warning",
        )
        server = uvicorn.Server(config)
        server.run()
        self.get_logger().info("[STOP][HTTP] uvicorn stopped")

    # -------------------------
    # Keyboard clear
    # -------------------------
    def _keyboard_loop(self) -> None:
        if not sys.stdin.isatty():
            self.get_logger().warning(
                "[KEY] stdin is not a TTY. Keyboard clear disabled."
            )
            return

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)

        try:
            tty.setcbreak(fd)
            while rclpy.ok() and not self._stop_event.is_set():
                readable, _, _ = select.select([sys.stdin], [], [], 0.2)
                if not readable:
                    continue

                ch = sys.stdin.read(1)
                if not ch:
                    continue

                if ch in ("c", "C"):
                    self.get_logger().warning("[KEY] 'C' pressed -> clear requested")
                    self.clear_all_waypoints(reason="keyboard")
        except Exception as e:
            self.get_logger().error(f"[KEY][ERROR] keyboard loop failed: {e}")
        finally:
            try:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            except Exception:
                pass

    # -------------------------
    # Cleanup
    # -------------------------
    def destroy_node(self):
        self._stop_event.set()
        return super().destroy_node()

def main(args=None) -> None:
    rclpy.init(args=args)
    node = DummyPatrolServerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("[STOP] KeyboardInterrupt")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
