import json
import subprocess

import rclpy
from rclpy.node import Node

from security_audio_msgs.msg import AudioClipInfo, SoundClassification


class YamnetClassifierNode(Node):
    def __init__(self):
        super().__init__("yamnet_classifier_node")

        self.declare_parameter("top_k", 5)
        self.declare_parameter("yamnet_python", "/home/chan/security_audio_ws/.venv_yamnet/bin/python")
        self.declare_parameter(
            "worker_script",
            "/home/chan/security_audio_ws/src/security_audio_classifier/security_audio_classifier/yamnet_worker.py"
        )

        self.top_k = self.get_parameter("top_k").value
        self.yamnet_python = self.get_parameter("yamnet_python").value
        self.worker_script = self.get_parameter("worker_script").value

        self.clip_sub = self.create_subscription(
            AudioClipInfo,
            "/audio/clip_info",
            self.clip_callback,
            10
        )

        self.cls_pub = self.create_publisher(
            SoundClassification,
            "/sound/classification",
            10
        )

        self.worker = None
        self.start_worker()

        self.get_logger().info("yamnet_classifier_node started")

    def start_worker(self):
        if self.worker is not None:
            try:
                self.worker.kill()
            except Exception:
                pass

        self.worker = subprocess.Popen(
            [self.yamnet_python, self.worker_script],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1
        )

        ready_line = self.worker.stdout.readline().strip()
        self.get_logger().info(f"worker ready: {ready_line}")

    def ensure_worker(self):
        if self.worker is None or self.worker.poll() is not None:
            self.get_logger().warning("worker is not running, restarting...")
            self.start_worker()

    def clip_callback(self, msg: AudioClipInfo):
        try:
            self.ensure_worker()

            req = {"wav_path": msg.clip_wav_path}
            self.worker.stdin.write(json.dumps(req) + "\n")
            self.worker.stdin.flush()

            line = self.worker.stdout.readline().strip()
            if not line:
                raise RuntimeError("empty response from worker")

            result = json.loads(line)

            if not result.get("ok", False):
                self.get_logger().error(f"classifier worker returned error: {result}")
                return

            out = SoundClassification()
            out.event_id = msg.event_id
            out.stamp = self.get_clock().now().to_msg()
            out.top1_label = result["task_label"]
            out.top1_confidence = float(result["task_confidence"])
            out.top_labels = result["top_labels"]
            out.top_confidences = result["top_scores"]

            self.cls_pub.publish(out)

            self.get_logger().info(
                f"Classified event_id={msg.event_id}, "
                f"label={out.top1_label}, conf={out.top1_confidence:.3f}, "
                f"infer_sec={result.get('timing_sec', -1):.3f}"
            )

        except Exception as e:
            self.get_logger().error(f"Classification failed: {e}")

    def destroy_node(self):
        try:
            if self.worker is not None and self.worker.poll() is None:
                self.worker.kill()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YamnetClassifierNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received, shutting down.")
    finally:
        try:
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
