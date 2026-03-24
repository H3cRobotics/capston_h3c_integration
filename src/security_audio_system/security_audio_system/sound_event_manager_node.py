import rclpy
from rclpy.node import Node

from security_audio_msgs.msg import AudioClipInfo, SoundClassification, SoundEvent


class SoundEventManagerNode(Node):
    def __init__(self):
        super().__init__('sound_event_manager_node')

        self.clip_cache = {}
        self.cls_cache = {}

        self.clip_sub = self.create_subscription(
            AudioClipInfo,
            '/audio/clip_info',
            self.clip_callback,
            10
        )

        self.cls_sub = self.create_subscription(
            SoundClassification,
            '/sound/classification',
            self.classification_callback,
            10
        )

        self.event_pub = self.create_publisher(
            SoundEvent,
            '/sound/event',
            10
        )

        self.get_logger().info('sound_event_manager_node started')

    def clip_callback(self, msg: AudioClipInfo):
        self.clip_cache[msg.event_id] = msg
        self.try_publish_event(msg.event_id)

    def classification_callback(self, msg: SoundClassification):
        self.cls_cache[msg.event_id] = msg
        self.try_publish_event(msg.event_id)

    def try_publish_event(self, event_id: str):
        if event_id not in self.clip_cache:
            return
        if event_id not in self.cls_cache:
            return

        clip_msg = self.clip_cache.pop(event_id)
        cls_msg = self.cls_cache.pop(event_id)

        out = SoundEvent()
        out.event_id = event_id
        out.stamp = self.get_clock().now().to_msg()
        out.clip_start_time = clip_msg.clip_start_time
        out.label = cls_msg.top1_label
        out.confidence = cls_msg.top1_confidence
        out.doa_deg = clip_msg.doa_deg
        out.level_dbfs = clip_msg.level_dbfs
        out.clip_wav_path = clip_msg.clip_wav_path
        out.clip_flac_path = ''
        out.transfer_requested = False
        out.transfer_success = False

        self.event_pub.publish(out)

        self.get_logger().info(
            f'Published sound event: '
            f'id={out.event_id}, '
            f'label={out.label}, '
            f'conf={out.confidence:.3f}, '
            f'doa={out.doa_deg:.1f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = SoundEventManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt received, shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()