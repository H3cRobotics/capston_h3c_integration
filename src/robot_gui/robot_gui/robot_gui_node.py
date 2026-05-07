#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import json
import math
import time
import threading
from typing import Optional

try:
    import pygame
except ImportError:
    pygame = None

import cv2
import yaml
import numpy as np

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool, Int32

try:
    from security_audio_msgs.msg import SoundEvent
except Exception:
    SoundEvent = None

from PyQt5.QtCore import Qt, QTimer, QPropertyAnimation
from PyQt5.QtGui import QImage, QPixmap, QFont
from PyQt5.QtWidgets import (
    QApplication,
    QWidget,
    QLabel,
    QVBoxLayout,
    QHBoxLayout,
    QGroupBox,
    QSizePolicy,
    QGraphicsOpacityEffect,
)


# ==========================================================
# Dark Theme
# ==========================================================
COLORS = {
    "green": {
        "border": "#22c55e",
        "bg": "#052e1b",
        "text": "#86efac",
    },
    "blue": {
        "border": "#3b82f6",
        "bg": "#0b1f44",
        "text": "#93c5fd",
    },
    "orange": {
        "border": "#f97316",
        "bg": "#431407",
        "text": "#fdba74",
    },
    "red": {
        "border": "#ef4444",
        "bg": "#450a0a",
        "text": "#fca5a5",
    },
    "gray": {
        "border": "#64748b",
        "bg": "#1f2937",
        "text": "#d1d5db",
    },
    "purple": {
        "border": "#8b5cf6",
        "bg": "#2e1065",
        "text": "#c4b5fd",
    },
}

APP_BG = "#0b1120"
PANEL_BG = "#111827"
PANEL_BG_2 = "#0f172a"
CARD_BORDER = "#334155"
TEXT_MAIN = "#f8fafc"
TEXT_SUB = "#cbd5e1"
TEXT_MUTED = "#94a3b8"


# ==========================================================
# Shared GUI state
# ==========================================================
class GuiState:
    def __init__(self):
        self.lock = threading.RLock()

        self.latest_frame = None

        self.robot_x: Optional[float] = None
        self.robot_y: Optional[float] = None
        self.robot_yaw: Optional[float] = None
        self.robot_status: str = "unknown"
        self.battery_percent: Optional[int] = None

        self.pending_next_place_id: str = "-"
        self.capture_result_hold_sec: float = 3.0

        self.auth_result_last_time: float = 0.0
        self.auth_result_hold_sec: float = 5.0

        self.goal_x: Optional[float] = None
        self.goal_y: Optional[float] = None
        self.goal_yaw: Optional[float] = None
        self.next_place_id: str = "-"

        self.follow_state: str = "IDLE"

        self.auth_ready: bool = False
        self.auth_result_status: str = "idle"
        self.auth_event_id: str = "-"

        self.patrol_command: str = "unknown"

        # None = 아직 토픽 수신 전
        self.yolo_enable: Optional[bool] = None
        self.audio_upload_enable: Optional[bool] = None
        self.audio_allowed_labels: str = "ALL"

        self.capture_status: str = "IDLE"
        self.capture_place_id: str = "-"
        self.capture_last_msg: str = "-"
        self.capture_last_time: float = 0.0

        self.audio_event_status: str = "IDLE"
        self.audio_event_label: str = "-"
        self.audio_event_doa: Optional[float] = None
        self.audio_event_id: str = "-"
        self.audio_event_last_time: float = 0.0

        self.map_image = None
        self.map_resolution: Optional[float] = None
        self.map_origin_x: Optional[float] = None
        self.map_origin_y: Optional[float] = None
        self.map_origin_yaw: float = 0.0
        self.map_path: str = ""


# ==========================================================
# ROS subscriber node
# ==========================================================
class RobotGuiRosNode(Node):
    def __init__(self, state: GuiState):
        super().__init__("robot_gui_node")
        self.state = state
        self.bridge = CvBridge()

        self.declare_parameter("annotated_topic", "/person_tracking/annotated")
        self.declare_parameter("robot_pose_topic", "/robot_pose")
        self.declare_parameter("robot_status_topic", "/robot_status")
        self.declare_parameter("battery_topic", "/robot/battery")
        self.declare_parameter("goal_pose_topic", "/goal_pose_2d")
        self.declare_parameter("next_place_topic", "/next_place_id")
        self.declare_parameter("follow_state_topic", "/person_tracking/follow_state")

        self.declare_parameter("auth_ready_topic", "/auth_ready")
        self.declare_parameter("auth_result_topic", "/auth/result")

        self.declare_parameter("patrol_command_topic", "/patrol/command")

        self.declare_parameter("yolo_enable_topic", "/person_tracking/enable")
        self.declare_parameter("audio_upload_enable_topic", "/sound/upload_enable")
        self.declare_parameter("audio_allowed_labels_topic", "/sound/allowed_labels")

        self.declare_parameter("capture_trigger_topic", "/patrol/capture_trigger")
        self.declare_parameter("capture_done_topic", "/patrol/capture_done")
        self.declare_parameter("sound_event_topic", "/sound/event")

        self.declare_parameter("map_yaml_path", "")

        annotated_topic = self.get_parameter("annotated_topic").value
        robot_pose_topic = self.get_parameter("robot_pose_topic").value
        robot_status_topic = self.get_parameter("robot_status_topic").value
        battery_topic = self.get_parameter("battery_topic").value
        goal_pose_topic = self.get_parameter("goal_pose_topic").value
        next_place_topic = self.get_parameter("next_place_topic").value
        follow_state_topic = self.get_parameter("follow_state_topic").value

        auth_ready_topic = self.get_parameter("auth_ready_topic").value
        auth_result_topic = self.get_parameter("auth_result_topic").value

        patrol_command_topic = self.get_parameter("patrol_command_topic").value

        yolo_enable_topic = self.get_parameter("yolo_enable_topic").value
        audio_upload_enable_topic = self.get_parameter("audio_upload_enable_topic").value
        audio_allowed_labels_topic = self.get_parameter("audio_allowed_labels_topic").value

        capture_trigger_topic = self.get_parameter("capture_trigger_topic").value
        capture_done_topic = self.get_parameter("capture_done_topic").value
        sound_event_topic = self.get_parameter("sound_event_topic").value

        map_yaml_path = str(self.get_parameter("map_yaml_path").value)
        self.load_map_yaml(map_yaml_path)

        self.create_subscription(Image, annotated_topic, self.annotated_cb, 10)
        self.create_subscription(Pose2D, robot_pose_topic, self.robot_pose_cb, 10)
        self.create_subscription(String, robot_status_topic, self.robot_status_cb, 10)
        self.create_subscription(Int32, battery_topic, self.battery_cb, 10)
        self.create_subscription(Pose2D, goal_pose_topic, self.goal_pose_cb, 10)
        self.create_subscription(String, next_place_topic, self.next_place_cb, 10)
        self.create_subscription(String, follow_state_topic, self.follow_state_cb, 10)

        self.create_subscription(Bool, auth_ready_topic, self.auth_ready_cb, 10)
        self.create_subscription(String, auth_result_topic, self.auth_result_cb, 10)

        self.create_subscription(String, patrol_command_topic, self.patrol_command_cb, 10)

        self.create_subscription(Bool, yolo_enable_topic, self.yolo_enable_cb, 10)
        self.create_subscription(Bool, audio_upload_enable_topic, self.audio_upload_enable_cb, 10)
        self.create_subscription(String, audio_allowed_labels_topic, self.audio_allowed_labels_cb, 10)

        self.create_subscription(String, capture_trigger_topic, self.capture_trigger_cb, 10)
        self.create_subscription(String, capture_done_topic, self.capture_done_cb, 10)

        if SoundEvent is not None:
            try:
                self.create_subscription(SoundEvent, sound_event_topic, self.sound_event_cb, 10)
            except Exception as e:
                self.get_logger().warn(
                    f"[AUDIO] failed to subscribe {sound_event_topic}: {e}. "
                    "Audio event GUI will stay IDLE."
                )
        else:
            self.get_logger().warn(
                "[AUDIO] security_audio_msgs.msg.SoundEvent is not available. "
                "Audio event GUI will stay IDLE."
            )

        self.get_logger().info("Robot GUI ROS node started")
        self.get_logger().info(f"camera={annotated_topic}")
        self.get_logger().info(f"robot_pose={robot_pose_topic}, goal_pose={goal_pose_topic}")
        self.get_logger().info(f"battery={battery_topic}")
        self.get_logger().info(f"auth_ready={auth_ready_topic}, auth_result={auth_result_topic}")
        self.get_logger().info(f"capture_trigger={capture_trigger_topic}, capture_done={capture_done_topic}")
        self.get_logger().info(f"sound_event={sound_event_topic}")
        self.get_logger().info(f"map_yaml_path={map_yaml_path}")

    # --------------------------
    # Map loading
    # --------------------------
    def load_map_yaml(self, map_yaml_path: str):
        if not map_yaml_path:
            self.get_logger().warn("[MAP] map_yaml_path is empty. Map viewer disabled.")
            return

        if not os.path.isfile(map_yaml_path):
            self.get_logger().warn(f"[MAP] yaml not found: {map_yaml_path}")
            return

        try:
            with open(map_yaml_path, "r", encoding="utf-8") as f:
                cfg = yaml.safe_load(f) or {}

            image_rel = cfg.get("image")
            resolution_raw = cfg.get("resolution")
            origin = cfg.get("origin", [0.0, 0.0, 0.0])

            if not image_rel:
                self.get_logger().warn("[MAP] image field is missing in yaml")
                return

            if resolution_raw is None:
                self.get_logger().warn("[MAP] resolution field is missing in yaml")
                return

            try:
                resolution = float(resolution_raw)
            except Exception:
                self.get_logger().warn(f"[MAP] invalid resolution: {resolution_raw}")
                return

            if resolution <= 0:
                self.get_logger().warn(f"[MAP] resolution must be positive: {resolution}")
                return

            if not isinstance(origin, (list, tuple)) or len(origin) < 2:
                self.get_logger().warn(f"[MAP] invalid origin: {origin}")
                return

            try:
                origin_x = float(origin[0])
                origin_y = float(origin[1])
                origin_yaw = float(origin[2]) if len(origin) >= 3 else 0.0
            except Exception:
                self.get_logger().warn(f"[MAP] invalid origin values: {origin}")
                return

            yaml_dir = os.path.dirname(map_yaml_path)
            image_path = str(image_rel)

            if not os.path.isabs(image_path):
                image_path = os.path.join(yaml_dir, image_path)

            if not os.path.isfile(image_path):
                self.get_logger().warn(f"[MAP] image not found: {image_path}")
                return

            img = cv2.imread(image_path, cv2.IMREAD_COLOR)
            if img is None:
                self.get_logger().warn(f"[MAP] failed to load image: {image_path}")
                return

            with self.state.lock:
                self.state.map_image = img
                self.state.map_resolution = resolution
                self.state.map_origin_x = origin_x
                self.state.map_origin_y = origin_y
                self.state.map_origin_yaw = origin_yaw
                self.state.map_path = image_path

            self.get_logger().info(
                f"[MAP] loaded image={image_path}, "
                f"resolution={resolution}, origin=({origin_x}, {origin_y}, {origin_yaw})"
            )

        except Exception as e:
            self.get_logger().warn(f"[MAP] load failed: {e}")

    # --------------------------
    # ROS callbacks
    # --------------------------
    def annotated_cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            with self.state.lock:
                self.state.latest_frame = frame.copy()
        except Exception as e:
            self.get_logger().warn(f"Failed to convert annotated image: {e}")

    def robot_pose_cb(self, msg: Pose2D):
        with self.state.lock:
            self.state.robot_x = float(msg.x)
            self.state.robot_y = float(msg.y)
            self.state.robot_yaw = float(msg.theta)

    def robot_status_cb(self, msg: String):
        value = msg.data.strip()
        with self.state.lock:
            self.state.robot_status = value if value else "idle"

    def battery_cb(self, msg: Int32):
        value = int(msg.data)
        value = max(0, min(100, value))

        with self.state.lock:
            self.state.battery_percent = value

    def goal_pose_cb(self, msg: Pose2D):
        with self.state.lock:
            self.state.goal_x = float(msg.x)
            self.state.goal_y = float(msg.y)
            self.state.goal_yaw = float(msg.theta)

    def next_place_cb(self, msg: String):
        value = msg.data.strip()
        next_place = value if value else "-"

        with self.state.lock:
            self.state.next_place_id = next_place

            if next_place in ["", "-", "None", "none", "NULL", "null"]:
                self.state.pending_next_place_id = "-"
                return

            now = time.time()
            status = (self.state.capture_status or "").strip().upper()
            elapsed = now - float(self.state.capture_last_time or 0.0)

            # DONE/FAILED 직후에는 결과를 잠깐 보여주기 위해 다음 이동 표시를 보류
            if status in ["DONE", "FAILED"] and elapsed < self.state.capture_result_hold_sec:
                self.state.pending_next_place_id = next_place
                return

            # 충분히 시간이 지났거나 일반 이동이면 바로 MOVING 표시로 전환
            self.state.pending_next_place_id = "-"
            self.state.capture_status = "IDLE"
            self.state.capture_place_id = "-"
            self.state.capture_last_msg = f"moving:{next_place}"
            self.state.capture_last_time = now

    def follow_state_cb(self, msg: String):
        value = msg.data.strip()
        with self.state.lock:
            self.state.follow_state = value if value else "unknown"

    def auth_ready_cb(self, msg: Bool):
        ready = bool(msg.data)

        with self.state.lock:
            self.state.auth_ready = ready

            if ready:
                self.state.auth_result_status = "waiting"
                self.state.auth_event_id = "-"

            if not ready and self.state.auth_result_status == "waiting":
                self.state.auth_result_status = "idle"

    def auth_result_cb(self, msg: String):
        try:
            payload = json.loads(msg.data)
            auth_event_id = str(payload.get("auth_event_id", "-")).strip()
            status = str(payload.get("status", "unknown")).strip().lower()
        except Exception:
            auth_event_id = "-"
            status = msg.data.strip().lower()

        if status not in ["success", "fail", "timeout"]:
            status = "unknown"

        with self.state.lock:
            self.state.auth_event_id = auth_event_id if auth_event_id else "-"
            self.state.auth_result_status = status
            self.state.auth_ready = False
            self.state.auth_result_last_time = time.time()

    def patrol_command_cb(self, msg: String):
        value = msg.data.strip()
        with self.state.lock:
            self.state.patrol_command = value if value else "unknown"

    def yolo_enable_cb(self, msg: Bool):
        with self.state.lock:
            self.state.yolo_enable = bool(msg.data)

    def audio_upload_enable_cb(self, msg: Bool):
        with self.state.lock:
            self.state.audio_upload_enable = bool(msg.data)

    def audio_allowed_labels_cb(self, msg: String):
        value = msg.data.strip()

        if not value or value == "[]":
            parsed = "ALL"
        else:
            try:
                labels = json.loads(value)
                if isinstance(labels, list) and len(labels) > 0:
                    parsed = ", ".join(map(str, labels))
                else:
                    parsed = "ALL"
            except Exception:
                parsed = value

        with self.state.lock:
            self.state.audio_allowed_labels = parsed

    # --------------------------
    # Capture / Audio callbacks
    # --------------------------
    def capture_trigger_cb(self, msg: String):
        place_id = msg.data.strip() if msg.data else "-"

        with self.state.lock:
            self.state.capture_status = "CAPTURING"
            self.state.capture_place_id = place_id if place_id else "-"
            self.state.capture_last_msg = f"trigger:{self.state.capture_place_id}"
            self.state.capture_last_time = time.time()

    def capture_done_cb(self, msg: String):
        raw = msg.data.strip() if msg.data else ""

        try:
            status, place_id = raw.split(":", 1)
            status = status.strip().lower()
            place_id = place_id.strip()
        except Exception:
            status = raw.lower()
            place_id = "-"

        if status == "done":
            capture_status = "DONE"
        elif status == "fail":
            capture_status = "FAILED"
        else:
            capture_status = status.upper() if status else "UNKNOWN"

        with self.state.lock:
            self.state.capture_status = capture_status
            self.state.capture_place_id = place_id if place_id else "-"
            self.state.capture_last_msg = raw if raw else "-"
            self.state.capture_last_time = time.time()

    def sound_event_cb(self, msg):
        label = str(msg.label).strip() if msg.label else "unknown"

        if label == "ignore":
            status = "IGNORED"
        else:
            status = "DETECTED"

        with self.state.lock:
            self.state.audio_event_status = status
            self.state.audio_event_label = label.upper()
            self.state.audio_event_doa = float(msg.doa_deg)
            self.state.audio_event_id = str(msg.event_id) if msg.event_id else "-"
            self.state.audio_event_last_time = time.time()


# ==========================================================
# PyQt GUI
# ==========================================================
class SecurityRobotGui(QWidget):
    def __init__(self, state: GuiState):
        super().__init__()
        self.state = state

        self.setWindowTitle("H3C Security Patrol Robot GUI")
        self.resize(1280, 900)

        self.last_popup_auth_status = None
        self.last_follow_state = None
        self.ui_state = {}

        self.last_map_render_time = 0.0
        self.map_render_dt = 0.25  # Map 4Hz

        self.audio_event_hold_sec = 5.0

        self.voice_dir = "/home/chan/capston_h3c_integration/src/robot_gui/audio"
        self.last_voice_key = None

        self.voice_files = {
            "tracking_start": "tracking_start.wav",
            "tracking_lost": "tracking_lost.wav",
        }

        if pygame is None:
            self.voice_enabled = False
            print("[VOICE WARN] pygame is not installed. Voice disabled.")
        else:
            try:
                pygame.mixer.init()
                self.voice_enabled = True
                print("[VOICE] pygame mixer initialized")
            except Exception as e:
                self.voice_enabled = False
                print(f"[VOICE WARN] pygame mixer init failed: {e}")

        self.camera_label = QLabel("Waiting for /person_tracking/annotated ...")
        self.camera_label.setAlignment(Qt.AlignCenter)
        self.camera_label.setMinimumHeight(430)
        self.camera_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.camera_label.setStyleSheet(f"""
            QLabel {{
                background-color: #020617;
                color: {TEXT_SUB};
                border: 2px solid {CARD_BORDER};
                border-radius: 12px;
                padding: 8px;
                font-size: 16px;
                font-weight: 800;
            }}
        """)

        self.map_label = QLabel("Map not loaded")
        self.map_label.setAlignment(Qt.AlignCenter)
        self.map_label.setMinimumHeight(430)
        self.map_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.map_label.setStyleSheet(f"""
            QLabel {{
                background-color: #020617;
                color: {TEXT_SUB};
                border: 2px solid {CARD_BORDER};
                border-radius: 12px;
                padding: 8px;
                font-size: 16px;
                font-weight: 800;
            }}
        """)

        self.robot_goal_label = QLabel()
        self.battery_label = QLabel("BATTERY WAITING")

        self.capture_state_label = QLabel()
        self.audio_event_label = QLabel()
        self.follow_state_label = QLabel()
        self.auth_state_label = QLabel()

        self.yolo_enable_label = QLabel()
        self.audio_upload_label = QLabel()

        self.robot_goal_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self.robot_goal_label.setMinimumHeight(220)
        self.robot_goal_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.robot_goal_label.setTextInteractionFlags(Qt.TextSelectableByMouse)

        self.battery_label.setAlignment(Qt.AlignCenter)
        self.battery_label.setMinimumHeight(52)
        self.battery_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.battery_label.setTextInteractionFlags(Qt.TextSelectableByMouse)

        for label in [
            self.capture_state_label,
            self.audio_event_label,
            self.follow_state_label,
            self.auth_state_label,
        ]:
            label.setAlignment(Qt.AlignCenter)
            label.setMinimumHeight(120)
            label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            label.setTextInteractionFlags(Qt.TextSelectableByMouse)

        for label in [
            self.yolo_enable_label,
            self.audio_upload_label,
        ]:
            label.setAlignment(Qt.AlignCenter)
            label.setMinimumHeight(62)
            label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            label.setTextInteractionFlags(Qt.TextSelectableByMouse)

        self._build_layout()
        self._build_auth_popup()
        self._apply_global_style()

        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self.refresh_ui)
        self.ui_timer.start(66)  # Camera/UI 약 15Hz

    def _apply_global_style(self):
        self.setStyleSheet(f"""
            QWidget {{
                background-color: {APP_BG};
                color: {TEXT_MAIN};
                font-family: DejaVu Sans;
            }}

            QGroupBox {{
                background-color: {PANEL_BG};
                border: 2px solid {CARD_BORDER};
                border-radius: 14px;
                margin-top: 14px;
                padding: 12px;
                font-size: 15px;
                font-weight: 900;
                color: {TEXT_MAIN};
            }}

            QGroupBox::title {{
                subcontrol-origin: margin;
                subcontrol-position: top left;
                padding: 0 8px;
                left: 12px;
                color: {TEXT_SUB};
                background-color: {PANEL_BG};
            }}
        """)

    def _build_layout(self):
        root = QVBoxLayout()
        root.setContentsMargins(12, 12, 12, 10)
        root.setSpacing(8)

        title = QLabel("H3C Security Patrol Robot")
        title.setFont(QFont("DejaVu Sans", 19, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        title.setMinimumHeight(42)
        title.setStyleSheet(f"""
            QLabel {{
                background-color: {PANEL_BG};
                color: {TEXT_MAIN};
                border: 2px solid {CARD_BORDER};
                border-radius: 12px;
                padding: 6px;
            }}
        """)
        root.addWidget(title, stretch=0)

        top = QHBoxLayout()
        top.setSpacing(10)
        top.addWidget(self.camera_label, stretch=3)
        top.addWidget(self.map_label, stretch=2)
        root.addLayout(top, stretch=7)

        middle = QHBoxLayout()
        middle.setSpacing(10)

        robot_box = QGroupBox("Robot / Goal")
        robot_layout = QVBoxLayout()
        robot_layout.setSpacing(8)
        robot_layout.addWidget(self.robot_goal_label)
        robot_layout.addWidget(self.battery_label)
        robot_box.setLayout(robot_layout)

        event_box = QGroupBox("Capture / Audio Event")
        event_layout = QVBoxLayout()
        event_layout.setSpacing(10)
        event_layout.addWidget(self.capture_state_label)
        event_layout.addWidget(self.audio_event_label)
        event_box.setLayout(event_layout)

        security_box = QGroupBox("Tracking / Auth / Mode")
        security_layout = QVBoxLayout()
        security_layout.setSpacing(10)

        security_layout.addWidget(self.follow_state_label)
        security_layout.addWidget(self.auth_state_label)

        mode_row = QHBoxLayout()
        mode_row.setSpacing(8)
        mode_row.addWidget(self.yolo_enable_label)
        mode_row.addWidget(self.audio_upload_label)
        security_layout.addLayout(mode_row)

        security_box.setLayout(security_layout)

        middle.addWidget(robot_box, stretch=2)
        middle.addWidget(event_box, stretch=3)
        middle.addWidget(security_box, stretch=3)

        root.addLayout(middle, stretch=3)

        self.setLayout(root)

    def _build_auth_popup(self):
        self.auth_popup_label = QLabel(self)
        self.auth_popup_label.setAlignment(Qt.AlignCenter)
        self.auth_popup_label.setVisible(False)
        self.auth_popup_label.setAttribute(Qt.WA_TransparentForMouseEvents)

        self.auth_popup_label.setStyleSheet("""
            QLabel {
                background-color: rgba(15, 23, 42, 235);
                color: white;
                border: 4px solid white;
                border-radius: 28px;
                padding: 28px;
                font-size: 34px;
                font-weight: 900;
            }
        """)

        self.auth_popup_effect = QGraphicsOpacityEffect(self.auth_popup_label)
        self.auth_popup_label.setGraphicsEffect(self.auth_popup_effect)

        self.auth_popup_anim = QPropertyAnimation(self.auth_popup_effect, b"opacity")
        self.auth_popup_anim.setDuration(1800)
        self.auth_popup_anim.setStartValue(1.0)
        self.auth_popup_anim.setEndValue(0.0)
        self.auth_popup_anim.finished.connect(self.auth_popup_label.hide)
    
    def apply_auth_result_timeout(self):
        with self.state.lock:
            status = (self.state.auth_result_status or "idle").strip().lower()

            if status not in ["success", "fail", "timeout", "unknown"]:
                return

            elapsed = time.time() - float(self.state.auth_result_last_time or 0.0)

            if elapsed <= self.state.auth_result_hold_sec:
                return

            self.state.auth_result_status = "idle"
            self.state.auth_event_id = "-"
            self.state.auth_ready = False
    
    def apply_audio_event_timeout(self):
        with self.state.lock:
            status = (self.state.audio_event_status or "IDLE").strip().upper()

            if status not in ["DETECTED", "IGNORED"]:
                return

            elapsed = time.time() - float(self.state.audio_event_last_time or 0.0)

            if elapsed <= self.audio_event_hold_sec:
                return

            self.state.audio_event_status = "IDLE"
            self.state.audio_event_label = "-"
            self.state.audio_event_doa = None
            self.state.audio_event_id = "-"
    
    def apply_pending_next_place_after_hold(self):
        with self.state.lock:
            pending_next = self.state.pending_next_place_id

            if pending_next in ["", "-", "None", "none", "NULL", "null"]:
                return

            status = (self.state.capture_status or "").strip().upper()
            elapsed = time.time() - float(self.state.capture_last_time or 0.0)

            # DONE/FAILED 표시 유지 시간이 아직 안 지났으면 그대로 둠
            if status in ["DONE", "FAILED"] and elapsed < self.state.capture_result_hold_sec:
                return

            # 유지 시간이 지났으면 MOVING TO pending_next 로 넘어가게 상태 초기화
            self.state.capture_status = "IDLE"
            self.state.capture_place_id = "-"
            self.state.capture_last_msg = f"moving:{pending_next}"
            self.state.capture_last_time = time.time()
            self.state.pending_next_place_id = "-"

    # --------------------------
    # Thread-safe snapshot
    # --------------------------
    def get_state_snapshot(self) -> dict:
        with self.state.lock:
            return {
                "robot_x": self.state.robot_x,
                "robot_y": self.state.robot_y,
                "robot_yaw": self.state.robot_yaw,
                "robot_status": self.state.robot_status,
                "battery_percent": self.state.battery_percent,

                "goal_x": self.state.goal_x,
                "goal_y": self.state.goal_y,
                "goal_yaw": self.state.goal_yaw,
                "next_place_id": self.state.next_place_id,

                "follow_state": self.state.follow_state,

                "auth_ready": self.state.auth_ready,
                "auth_result_status": self.state.auth_result_status,
                "auth_event_id": self.state.auth_event_id,

                "patrol_command": self.state.patrol_command,

                "yolo_enable": self.state.yolo_enable,
                "audio_upload_enable": self.state.audio_upload_enable,

                "capture_status": self.state.capture_status,
                "capture_place_id": self.state.capture_place_id,

                "audio_event_status": self.state.audio_event_status,
                "audio_event_label": self.state.audio_event_label,
                "audio_event_doa": self.state.audio_event_doa,
                "audio_event_id": self.state.audio_event_id,
                "audio_event_last_time": self.state.audio_event_last_time,
            }

    def get_latest_frame_copy(self):
        with self.state.lock:
            if self.state.latest_frame is None:
                return None
            return self.state.latest_frame.copy()

    def get_map_snapshot(self):
        with self.state.lock:
            if self.state.map_image is None:
                return None

            return {
                "map_image": self.state.map_image.copy(),
                "map_resolution": self.state.map_resolution,
                "map_origin_x": self.state.map_origin_x,
                "map_origin_y": self.state.map_origin_y,
                "map_origin_yaw": self.state.map_origin_yaw,
            }

    # --------------------------
    # Formatting
    # --------------------------
    @staticmethod
    def fmt_pose(x: Optional[float], y: Optional[float], yaw: Optional[float]) -> str:
        if x is None or y is None or yaw is None:
            return "x:-  y:-  yaw:-"
        return f"x:{x:.3f}  y:{y:.3f}  yaw:{yaw:.3f}"

    @staticmethod
    def theme(name: str):
        return COLORS.get(name, COLORS["gray"])

    def set_named_box(
        self,
        label: QLabel,
        title: str,
        value: str,
        theme_name: str,
        subtitle: str = "",
        value_size: int = 30,
        title_size: int = 16,
        subtitle_size: int = 15,
    ):
        c = self.theme(theme_name)
        border = c["border"]
        bg = c["bg"]
        text_color = c["text"]

        label.setStyleSheet(f"""
            QLabel {{
                background-color: {bg};
                border: 3px solid {border};
                border-radius: 16px;
                padding: 10px;
                color: {TEXT_MAIN};
            }}
        """)

        html = (
            f"<div style='font-size:{title_size}px; font-weight:900; color:{TEXT_SUB};'>{title}</div>"
            f"<div style='font-size:{value_size}px; font-weight:950; color:{text_color};'>{value}</div>"
        )

        if subtitle:
            html += (
                f"<div style='font-size:{subtitle_size}px; "
                f"font-weight:800; color:{TEXT_MUTED};'>{subtitle}</div>"
            )

        label.setText(html)

    def set_mode_badge(self, label: QLabel, title: str, enabled: Optional[bool]):
        if enabled is None:
            theme_name = "gray"
            value = "WAITING"
            value_size = 18
        else:
            theme_name = "green" if enabled else "gray"
            value = "ON" if enabled else "OFF"
            value_size = 20

        c = self.theme(theme_name)

        border = c["border"]
        bg = c["bg"]
        text_color = c["text"]
        dot = "●"

        label.setStyleSheet(f"""
            QLabel {{
                background-color: {bg};
                border: 2px solid {border};
                border-radius: 12px;
                padding: 8px 6px;
                color: {TEXT_MAIN};
            }}
        """)

        label.setText(
            f"<div style='line-height:1.15; text-align:center;'>"
            f"<span style='font-size:13px; font-weight:900; color:{TEXT_SUB};'>{title}</span>"
            f"<br>"
            f"<span style='font-size:{value_size}px; font-weight:950; color:{text_color};'>{dot} {value}</span>"
            f"</div>"
        )

    def set_robot_goal_box(self):
        pose = self.fmt_pose(
            self.ui_state.get("robot_x"),
            self.ui_state.get("robot_y"),
            self.ui_state.get("robot_yaw"),
        )
        goal = self.fmt_pose(
            self.ui_state.get("goal_x"),
            self.ui_state.get("goal_y"),
            self.ui_state.get("goal_yaw"),
        )

        robot_status = self.ui_state.get("robot_status") or "-"
        command = self.ui_state.get("patrol_command") or "-"

        html = (
            f"<div style='line-height:1.65;'>"

            f"<div style='font-size:14px; font-weight:950; color:#93c5fd;'>POSE</div>"
            f"<div style='font-size:16px; font-weight:850; color:{TEXT_MAIN}; margin-bottom:16px;'>{pose}</div>"

            f"<div style='font-size:14px; font-weight:950; color:#86efac;'>GOAL</div>"
            f"<div style='font-size:16px; font-weight:850; color:{TEXT_MAIN}; margin-bottom:16px;'>{goal}</div>"

            f"<div style='font-size:14px; font-weight:950; color:#c4b5fd;'>STATUS | CMD</div>"
            f"<div style='font-size:16px; font-weight:950; color:{TEXT_MAIN};'>"
            f"{robot_status} <span style='color:{TEXT_MUTED};'>|</span> {command}"
            f"</div>"

            f"</div>"
        )

        self.robot_goal_label.setStyleSheet(f"""
            QLabel {{
                background-color: {PANEL_BG_2};
                border: 2px solid {CARD_BORDER};
                border-radius: 14px;
                padding: 16px;
                color: {TEXT_MAIN};
            }}
        """)
        self.robot_goal_label.setText(html)

    def set_battery_box(self):
        percent = self.ui_state.get("battery_percent")

        if percent is None:
            value_text = "BATTERY  WAITING"
            border_color = COLORS["gray"]["border"]
            fill_color = COLORS["gray"]["bg"]
            bg_color = "#020617"
            text_color = TEXT_MUTED
            stop_fill = 0.0
            stop_empty = 0.001
        else:
            percent = max(0, min(100, int(percent)))
            value_text = f"BATTERY  {percent}%"
            bg_color = "#020617"
            text_color = TEXT_MAIN

            if percent >= 70:
                border_color = COLORS["green"]["border"]
                fill_color = COLORS["green"]["border"]
            elif percent >= 30:
                border_color = COLORS["orange"]["border"]
                fill_color = COLORS["orange"]["border"]
            else:
                border_color = COLORS["red"]["border"]
                fill_color = COLORS["red"]["border"]

            stop_fill = percent / 100.0
            stop_empty = min(1.0, stop_fill + 0.001)

        self.battery_label.setStyleSheet(f"""
            QLabel {{
                background: qlineargradient(
                    x1:0, y1:0, x2:1, y2:0,
                    stop:0 {fill_color},
                    stop:{stop_fill:.3f} {fill_color},
                    stop:{stop_empty:.3f} {bg_color},
                    stop:1 {bg_color}
                );
                border: 3px solid {border_color};
                border-radius: 14px;
                padding: 6px;
                color: {text_color};
                font-size: 20px;
                font-weight: 950;
            }}
        """)
        self.battery_label.setText(value_text)

    def set_tracking_box(self):
        state = (self.ui_state.get("follow_state") or "").strip().upper()

        if state == "TRACKING":
            theme_name = "blue"
            value_text = "TRACKING"
        elif state == "LOST":
            theme_name = "orange"
            value_text = "LOST"
        elif state == "IDLE":
            theme_name = "gray"
            value_text = "IDLE"
        else:
            theme_name = "gray"
            value_text = state if state else "UNKNOWN"

        self.set_named_box(
            self.follow_state_label,
            "Tracking",
            value_text,
            theme_name,
        )

    def set_auth_box(self):
        status = (self.ui_state.get("auth_result_status") or "").strip().lower()
        auth_ready = bool(self.ui_state.get("auth_ready"))
        auth_event_id = self.ui_state.get("auth_event_id") or "-"

        if status == "success":
            theme_name = "green"
            value_text = "SUCCESS"
        elif status == "fail":
            theme_name = "red"
            value_text = "FAILED"
        elif status == "timeout":
            theme_name = "orange"
            value_text = "TIMEOUT"
        elif auth_ready or status == "waiting":
            theme_name = "blue"
            value_text = "RFID WAITING"
        elif status == "unknown":
            theme_name = "purple"
            value_text = "UNKNOWN"
        else:
            theme_name = "gray"
            value_text = "IDLE"

        self.set_named_box(
            self.auth_state_label,
            "2nd Auth",
            value_text,
            theme_name,
            subtitle=f"event: {auth_event_id}",
            value_size=27,
        )

    def set_capture_box(self):
        status = (self.ui_state.get("capture_status") or "IDLE").strip().upper()
        capture_place = self.ui_state.get("capture_place_id") or "-"
        next_place = self.ui_state.get("next_place_id") or "-"

        has_next = str(next_place).strip() not in ["", "-", "None", "none", "NULL", "null"]

        if status == "CAPTURING":
            theme_name = "blue"
            value_text = "CAPTURING"
            subtitle = f"place: {capture_place}"

        elif status == "DONE":
            theme_name = "green"
            value_text = "DONE"
            subtitle = f"place: {capture_place}"

        elif status == "FAILED":
            theme_name = "red"
            value_text = "FAILED"
            subtitle = f"place: {capture_place}"

        elif has_next:
            theme_name = "blue"
            value_text = f"MOVING TO {next_place}"
            subtitle = "waiting for capture trigger"

        else:
            theme_name = "gray"
            value_text = "IDLE"
            subtitle = "no active place"

        moving = value_text.startswith("MOVING TO")

        self.set_named_box(
            self.capture_state_label,
            "Capture",
            value_text,
            theme_name,
            subtitle=subtitle,
            value_size=24 if moving else 30,
            title_size=16,
            subtitle_size=15,
        )

    def set_audio_event_box(self):
        now = time.time()
        last_time = float(self.ui_state.get("audio_event_last_time") or 0.0)
        elapsed = now - last_time

        status = (self.ui_state.get("audio_event_status") or "IDLE").strip().upper()

        if status in ["DETECTED", "IGNORED"] and elapsed > self.audio_event_hold_sec:
            status = "IDLE"

        label = self.ui_state.get("audio_event_label") or "-"
        doa = self.ui_state.get("audio_event_doa")
        doa_text = "-" if doa is None else f"{float(doa):.1f}°"

        if status == "DETECTED":
            if label in ["ALARM", "SCREAM"]:
                theme_name = "red"
            elif label == "IMPACT":
                theme_name = "orange"
            else:
                theme_name = "blue"

            value_text = "DETECTED"
            subtitle = f"label: {label} | DOA: {doa_text}"

        elif status == "IGNORED":
            theme_name = "gray"
            value_text = "IGNORED"
            subtitle = f"label: {label} | DOA: {doa_text}"

        else:
            theme_name = "gray"
            value_text = "IDLE"
            subtitle = "label: - | DOA: -"

        self.set_named_box(
            self.audio_event_label,
            "Audio Event",
            value_text,
            theme_name,
            subtitle=subtitle,
        )

    # --------------------------
    # Popup / Voice
    # --------------------------
    def show_auth_popup(self, title: str, subtitle: str = "", color: str = "#ffffff"):
        if subtitle:
            text = (
                f"<div style='font-size:38px; font-weight:900; color:{color};'>{title}</div>"
                f"<div style='font-size:22px; font-weight:700; color:#eeeeee; margin-top:8px;'>{subtitle}</div>"
            )
        else:
            text = f"<div style='font-size:38px; font-weight:900; color:{color};'>{title}</div>"

        self.auth_popup_label.setText(text)

        popup_w = min(760, max(520, int(self.width() * 0.58)))
        popup_h = 190

        x = int((self.width() - popup_w) / 2)
        y = int((self.height() - popup_h) / 2)

        self.auth_popup_label.setGeometry(x, y, popup_w, popup_h)
        self.auth_popup_label.raise_()
        self.auth_popup_label.show()

        self.auth_popup_effect.setOpacity(1.0)

        self.auth_popup_anim.stop()
        self.auth_popup_anim.setStartValue(1.0)
        self.auth_popup_anim.setEndValue(0.0)
        self.auth_popup_anim.start()

    def play_voice_event(self, event_key: str):
        if not getattr(self, "voice_enabled", False):
            return

        if event_key == self.last_voice_key:
            return

        self.last_voice_key = event_key

        filename = self.voice_files.get(event_key)
        if not filename:
            print(f"[VOICE WARN] unknown event_key: {event_key}")
            return

        path = os.path.join(self.voice_dir, filename)

        if not os.path.isfile(path):
            print(f"[VOICE WARN] file not found: {path}")
            return

        try:
            pygame.mixer.music.load(path)
            pygame.mixer.music.play()
            print(f"[VOICE] playing: {filename}")
        except Exception as e:
            print(f"[VOICE WARN] play failed: {e}")

    def handle_tracking_popup_event(self):
        current_state = (self.ui_state.get("follow_state") or "").strip().upper()
        prev_state = (self.last_follow_state or "").strip().upper()

        if current_state == prev_state:
            return

        self.last_follow_state = current_state

        if current_state == "TRACKING" and prev_state == "IDLE":
            self.show_auth_popup(
                "추적 시작",
                "대상자를 추적 중입니다",
                "#93c5fd",
            )
            self.play_voice_event("tracking_start")

        elif prev_state == "TRACKING" and current_state == "LOST":
            self.show_auth_popup(
                "추적 대상 상실",
                "대상자를 다시 탐색 중입니다",
                "#fdba74",
            )
            self.play_voice_event("tracking_lost")

    def handle_auth_popup_event(self):
        current_auth_status = (self.ui_state.get("auth_result_status") or "").strip().lower()

        if current_auth_status == self.last_popup_auth_status:
            return

        self.last_popup_auth_status = current_auth_status

        if current_auth_status == "waiting":
            self.show_auth_popup(
                "2차 인증 시작",
                "RFID 카드를 태그하세요",
                "#93c5fd",
            )
        elif current_auth_status == "success":
            self.show_auth_popup(
                "인증 성공",
                "출입 권한이 확인되었습니다",
                "#86efac",
            )
        elif current_auth_status == "fail":
            self.show_auth_popup(
                "인증 실패",
                "등록되지 않은 카드입니다",
                "#fca5a5",
            )
        elif current_auth_status == "timeout":
            self.show_auth_popup(
                "인증 시간 초과",
                "RFID 태그가 감지되지 않았습니다",
                "#fdba74",
            )

    def resizeEvent(self, event):
        super().resizeEvent(event)

        if hasattr(self, "auth_popup_label") and self.auth_popup_label.isVisible():
            popup_w = min(760, max(520, int(self.width() * 0.58)))
            popup_h = 190
            x = int((self.width() - popup_w) / 2)
            y = int((self.height() - popup_h) / 2)
            self.auth_popup_label.setGeometry(x, y, popup_w, popup_h)

    # --------------------------
    # Map transform / drawing
    # --------------------------
    @staticmethod
    def world_to_pixel_from_snapshot(map_snapshot: dict, x: float, y: float):
        if map_snapshot is None:
            return None

        resolution = map_snapshot.get("map_resolution")
        origin_x = map_snapshot.get("map_origin_x")
        origin_y = map_snapshot.get("map_origin_y")
        map_img = map_snapshot.get("map_image")

        if resolution is None or origin_x is None or origin_y is None or map_img is None:
            return None

        h, _ = map_img.shape[:2]

        px = int((x - origin_x) / resolution)
        py = int(h - ((y - origin_y) / resolution))

        return px, py

    def draw_text_with_outline(self, img, text: str, pos, color, scale=0.6, thickness=2):
        outline = (255, 255, 255)
        cv2.putText(
            img,
            text,
            pos,
            cv2.FONT_HERSHEY_SIMPLEX,
            scale,
            outline,
            thickness + 2,
            cv2.LINE_AA,
        )
        cv2.putText(
            img,
            text,
            pos,
            cv2.FONT_HERSHEY_SIMPLEX,
            scale,
            color,
            thickness,
            cv2.LINE_AA,
        )

    def draw_robot_marker(self, img, map_snapshot: dict, x: float, y: float, yaw: float):
        pix = self.world_to_pixel_from_snapshot(map_snapshot, x, y)
        if pix is None:
            return

        px, py = pix
        h, w = img.shape[:2]

        if px < 0 or px >= w or py < 0 or py >= h:
            return

        color = (255, 80, 20)
        outline = (255, 255, 255)

        tip_len = 22
        rear_len = 13
        spread = 2.45

        tip = np.array([
            px + tip_len * math.cos(yaw),
            py - tip_len * math.sin(yaw),
        ])

        left = np.array([
            px + rear_len * math.cos(yaw + spread),
            py - rear_len * math.sin(yaw + spread),
        ])

        right = np.array([
            px + rear_len * math.cos(yaw - spread),
            py - rear_len * math.sin(yaw - spread),
        ])

        pts = np.array([tip, left, right], dtype=np.int32)

        cv2.polylines(img, [pts], True, outline, 5, cv2.LINE_AA)
        cv2.fillPoly(img, [pts], color, lineType=cv2.LINE_AA)
        cv2.polylines(img, [pts], True, outline, 2, cv2.LINE_AA)

        cv2.circle(img, (px, py), 4, outline, -1)
        cv2.circle(img, (px, py), 24, color, 2, cv2.LINE_AA)

        self.draw_text_with_outline(
            img,
            "ROBOT",
            (px + 18, py - 18),
            color,
            scale=0.6,
            thickness=2,
        )

    def draw_goal_marker(self, img, map_snapshot: dict, x: float, y: float, yaw: float):
        pix = self.world_to_pixel_from_snapshot(map_snapshot, x, y)
        if pix is None:
            return

        px, py = pix
        h, w = img.shape[:2]

        if px < 0 or px >= w or py < 0 or py >= h:
            return

        color = (40, 40, 255)
        outline = (255, 255, 255)
        dark = (0, 0, 120)

        cv2.circle(img, (px, py), 18, outline, 5, cv2.LINE_AA)
        cv2.circle(img, (px, py), 18, color, 3, cv2.LINE_AA)
        cv2.circle(img, (px, py), 8, outline, 4, cv2.LINE_AA)
        cv2.circle(img, (px, py), 8, color, 2, cv2.LINE_AA)
        cv2.circle(img, (px, py), 3, color, -1, cv2.LINE_AA)

        arrow_len = 30
        end_x = int(px + arrow_len * math.cos(yaw))
        end_y = int(py - arrow_len * math.sin(yaw))

        cv2.arrowedLine(
            img,
            (px, py),
            (end_x, end_y),
            color,
            2,
            tipLength=0.35,
        )

        label = "GOAL"
        text_pos = (px + 16, py - 16)

        (tw, th), _ = cv2.getTextSize(
            label,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            2,
        )

        box_x1 = text_pos[0] - 5
        box_y1 = text_pos[1] - th - 6
        box_x2 = text_pos[0] + tw + 5
        box_y2 = text_pos[1] + 5

        cv2.rectangle(
            img,
            (box_x1, box_y1),
            (box_x2, box_y2),
            outline,
            -1,
        )
        cv2.rectangle(
            img,
            (box_x1, box_y1),
            (box_x2, box_y2),
            color,
            2,
        )

        cv2.putText(
            img,
            label,
            text_pos,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            dark,
            2,
            cv2.LINE_AA,
        )

    def refresh_map(self):
        map_snapshot = self.get_map_snapshot()
        if map_snapshot is None:
            self.map_label.setText("Map not loaded")
            return

        try:
            vis = map_snapshot["map_image"]

            robot_x = self.ui_state.get("robot_x")
            robot_y = self.ui_state.get("robot_y")
            robot_yaw = self.ui_state.get("robot_yaw")

            goal_x = self.ui_state.get("goal_x")
            goal_y = self.ui_state.get("goal_y")
            goal_yaw = self.ui_state.get("goal_yaw")

            if robot_x is not None and robot_y is not None and robot_yaw is not None:
                self.draw_robot_marker(
                    vis,
                    map_snapshot,
                    robot_x,
                    robot_y,
                    robot_yaw,
                )

            if goal_x is not None and goal_y is not None and goal_yaw is not None:
                self.draw_goal_marker(
                    vis,
                    map_snapshot,
                    goal_x,
                    goal_y,
                    goal_yaw,
                )

            rgb = cv2.cvtColor(vis, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb.shape
            qimg = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888)

            pixmap = QPixmap.fromImage(qimg)
            pixmap = pixmap.scaled(
                self.map_label.width(),
                self.map_label.height(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation,
            )

            self.map_label.setPixmap(pixmap)

        except Exception:
            self.map_label.setText("Map rendering failed")

    # --------------------------
    # Refresh
    # --------------------------
    def refresh_ui(self):
        self.apply_pending_next_place_after_hold()
        self.apply_audio_event_timeout()
        self.apply_auth_result_timeout()
        self.ui_state = self.get_state_snapshot()

        self.refresh_camera()

        now = time.time()
        if now - self.last_map_render_time >= self.map_render_dt:
            self.refresh_map()
            self.last_map_render_time = now

        self.set_robot_goal_box()
        self.set_battery_box()

        self.set_capture_box()
        self.set_audio_event_box()

        self.set_tracking_box()
        self.set_auth_box()

        self.set_mode_badge(
            self.yolo_enable_label,
            "YOLO MODE",
            self.ui_state.get("yolo_enable"),
        )

        self.set_mode_badge(
            self.audio_upload_label,
            "AUDIO MODE",
            self.ui_state.get("audio_upload_enable"),
        )

        self.handle_tracking_popup_event()
        self.handle_auth_popup_event()

    def refresh_camera(self):
        frame = self.get_latest_frame_copy()
        if frame is None:
            return

        try:
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb.shape
            qimg = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888)

            pixmap = QPixmap.fromImage(qimg)
            pixmap = pixmap.scaled(
                self.camera_label.width(),
                self.camera_label.height(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation,
            )
            self.camera_label.setPixmap(pixmap)
        except Exception:
            self.camera_label.setText("Camera frame rendering failed")


# ==========================================================
# Main
# ==========================================================
def main(args=None):
    rclpy.init(args=args)

    app = QApplication(sys.argv)
    state = GuiState()

    node = RobotGuiRosNode(state)
    gui = SecurityRobotGui(state)

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    gui.show()
    exit_code = app.exec_()

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()