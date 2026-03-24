import rclpy
from rclpy.node import Node

from security_audio_msgs.msg import SoundEvent


class SoundEventMonitorNode(Node):
    def __init__(self):
        super().__init__('sound_event_monitor_node')

        self.declare_parameter('ignore_label', 'ignore')
        self.ignore_label = self.get_parameter('ignore_label').value

        self.sub = self.create_subscription(
            SoundEvent,
            '/sound/event',
            self.event_callback,
            10
        )

        self.get_logger().info('sound_event_monitor_node started')

    def event_callback(self, msg: SoundEvent):
        if msg.label == self.ignore_label:
            return

        self.get_logger().info(
            f'[EVENT] label={msg.label} '
            f'conf={msg.confidence:.3f} '
            f'doa={msg.doa_deg:.1f} '
            f'file={msg.clip_wav_path}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = SoundEventMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt received, shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()