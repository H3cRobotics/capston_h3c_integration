# can_teach_trigger.py

import can
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty


class CanTeachTriggerNode(Node):
    def __init__(self):
        super().__init__("can_teach_trigger")

        self.declare_parameter("can_interface", "can0")
        self.declare_parameter("teach_can_id", 0x102)
        self.declare_parameter("debounce_sec", 5.0)
        self.declare_parameter("teach_trigger_topic", "/teach_trigger")

        self.can_interface = self.get_parameter("can_interface").value
        self.teach_can_id = int(self.get_parameter("teach_can_id").value)
        self.debounce_sec = float(self.get_parameter("debounce_sec").value)
        self.teach_trigger_topic = self.get_parameter("teach_trigger_topic").value

        self.pub = self.create_publisher(Empty, self.teach_trigger_topic, 10)

        self.bus = can.interface.Bus(channel=self.can_interface, bustype="socketcan")

        self.last_trigger_time = 0.0

        self.create_timer(0.01, self.loop)

        self.get_logger().info(
            f"CAN trigger started | id=0x{self.teach_can_id:X} | debounce={self.debounce_sec}s"
        )

    def loop(self):
        msg = self.bus.recv(timeout=0.0)
        if msg is None:
            return

        if msg.arbitration_id != self.teach_can_id:
            return

        if len(msg.data) == 0:
            return

        value = msg.data[0]
        now = time.time()

        is_after_cooldown = (now - self.last_trigger_time) > self.debounce_sec

        if value == 1 and is_after_cooldown:
            self.pub.publish(Empty())
            self.last_trigger_time = now
            self.get_logger().info("Teach trigger")

    def destroy_node(self):
        try:
            self.bus.shutdown()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CanTeachTriggerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()