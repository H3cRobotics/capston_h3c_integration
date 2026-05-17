import os
from datetime import datetime, timezone
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool

from security_audio_msgs.msg import SoundEvent, TransferStatus
from .upload_client import send_audio


class ClipTransferNode(Node):
    def __init__(self):
        super().__init__('clip_transfer_node')

        self.declare_parameter('server_url', 'http://10.162.213.189:8000')
        self.declare_parameter('pose_topic', '/robot_pose')
        self.declare_parameter('upload_enable_topic', '/sound/upload_enable')
        self.declare_parameter('start_enabled', False)

        self.server_url = self.get_parameter('server_url').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.upload_enable_topic = str(self.get_parameter('upload_enable_topic').value)
        self.audio_enabled = bool(self.get_parameter('start_enabled').value)

        self.x: Optional[float] = None
        self.y: Optional[float] = None
        self.yaw: Optional[float] = None

        self.create_subscription(
            Pose2D,
            self.pose_topic,
            self.pose_callback,
            10
        )

        self.event_sub = self.create_subscription(
            SoundEvent,
            '/sound/event',
            self.event_callback,
            10
        )

        self.create_subscription(
            Bool,
            self.upload_enable_topic,
            self.upload_enable_callback,
            10
        )

        self.status_pub = self.create_publisher(
            TransferStatus,
            '/sound/transfer_status',
            10
        )

        self.get_logger().info(
            f'clip_transfer_node started | server_url={self.server_url} | '
            f'pose_topic={self.pose_topic} | upload_enable_topic={self.upload_enable_topic} | '
            f'audio_enabled={self.audio_enabled}'
        )

    def upload_enable_callback(self, msg: Bool):
        prev_enabled = self.audio_enabled
        self.audio_enabled = bool(msg.data)

        if prev_enabled != self.audio_enabled:
            state_text = 'enabled' if self.audio_enabled else 'disabled'
            self.get_logger().info(f'Clip transfer {state_text} by {self.upload_enable_topic}')

    def pose_callback(self, msg: Pose2D):
        self.x = float(msg.x)
        self.y = float(msg.y)
        self.yaw = float(msg.theta)

    def ros_time_to_iso(self, t) -> str:
        ts = float(t.sec) + float(t.nanosec) / 1e9
        return datetime.fromtimestamp(ts, tz=timezone.utc).isoformat()

    def publish_status(self, event_id: str, success: bool, detail: str):
        status = TransferStatus()
        status.event_id = event_id
        status.stamp = self.get_clock().now().to_msg()
        status.remote_host = self.server_url
        status.method = "http"
        status.success = success
        status.detail = detail
        self.status_pub.publish(status)

    def event_callback(self, msg: SoundEvent):
        try:
            if not self.audio_enabled:
                self.get_logger().info(
                    f'Skip upload: event_id={msg.event_id}, audio system disabled'
                )
                self.publish_status(msg.event_id, False, 'audio system disabled')
                return

            if msg.label == 'ignore':
                self.get_logger().info(
                    f'Skip upload: event_id={msg.event_id}, label=ignore'
                )
                self.publish_status(msg.event_id, False, 'ignored event')
                return

            if self.x is None or self.y is None or self.yaw is None:
                raise RuntimeError('robot pose is not available yet')

            wav_path = msg.clip_wav_path
            if not os.path.isfile(wav_path):
                raise FileNotFoundError(f'WAV file not found: {wav_path}')

            filename = os.path.basename(wav_path)

            with open(wav_path, 'rb') as f:
                audio_bytes = f.read()

            record_start_time = self.ros_time_to_iso(msg.clip_start_time)

            result = send_audio(
                server_url=self.server_url,
                audio_bytes=audio_bytes,
                filename=filename,
                event_label=msg.label,
                doa=msg.doa_deg,
                record_start_time=record_start_time,
                x=self.x,
                y=self.y,
                yaw=self.yaw,
            )

            self.get_logger().info(
                f'Uploaded event_id={msg.event_id}, '
                f'label={msg.label}, doa={msg.doa_deg:.1f}, '
                f'pose=({self.x:.3f}, {self.y:.3f}, {self.yaw:.3f}), '
                f'start={record_start_time}, result={result}'
            )
            self.publish_status(msg.event_id, True, str(result))

        except Exception as e:
            self.get_logger().error(
                f'Upload failed: event_id={msg.event_id}, error={e}'
            )
            self.publish_status(msg.event_id, False, str(e))


def main(args=None):
    rclpy.init(args=args)
    node = ClipTransferNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt received, shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()