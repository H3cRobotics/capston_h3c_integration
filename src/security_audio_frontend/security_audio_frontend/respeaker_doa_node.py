import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool

import usb.core

sys.path.append('/home/chan/usb_4_mic_array')
from tuning import Tuning


class RespeakerDoaNode(Node):
    def __init__(self):
        super().__init__('respeaker_doa_node')

        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('raw_topic', '/sound/doa_raw_deg')
        self.declare_parameter('mapped_topic', '/sound/doa_deg')
        self.declare_parameter('voice_topic', '/sound/voice_active')
        self.declare_parameter('front_offset_deg', 0.0)
        self.declare_parameter('invert_sign', False)
        self.declare_parameter('use_vad_gate', False)

        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.raw_topic = str(self.get_parameter('raw_topic').value)
        self.mapped_topic = str(self.get_parameter('mapped_topic').value)
        self.voice_topic = str(self.get_parameter('voice_topic').value)
        self.front_offset_deg = float(self.get_parameter('front_offset_deg').value)
        self.invert_sign = bool(self.get_parameter('invert_sign').value)
        self.use_vad_gate = bool(self.get_parameter('use_vad_gate').value)

        self.raw_pub = self.create_publisher(Float32, self.raw_topic, 10)
        self.doa_pub = self.create_publisher(Float32, self.mapped_topic, 10)
        self.voice_pub = self.create_publisher(Bool, self.voice_topic, 10)

        dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)
        if dev is None:
            raise RuntimeError('ReSpeaker control device not found (0x2886:0x0018)')

        self.mic = Tuning(dev)

        period = 1.0 / self.publish_rate_hz
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info('respeaker_doa_node started')

    def wrap_deg_pm180(self, deg: float) -> float:
        while deg > 180.0:
            deg -= 360.0
        while deg <= -180.0:
            deg += 360.0
        return deg

    def map_angle(self, raw_deg: float) -> float:
        mapped = raw_deg - self.front_offset_deg

        if self.invert_sign:
            mapped = -mapped

        return self.wrap_deg_pm180(mapped)

    def timer_callback(self):
        try:
            raw_deg = float(self.mic.direction)
            voice = bool(self.mic.is_voice())

            raw_msg = Float32()
            raw_msg.data = raw_deg
            self.raw_pub.publish(raw_msg)

            voice_msg = Bool()
            voice_msg.data = voice
            self.voice_pub.publish(voice_msg)

            if self.use_vad_gate and not voice:
                return

            mapped_deg = self.map_angle(raw_deg)

            doa_msg = Float32()
            doa_msg.data = mapped_deg
            self.doa_pub.publish(doa_msg)

        except Exception as e:
            self.get_logger().error(f'DOA read failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = RespeakerDoaNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt received, shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()