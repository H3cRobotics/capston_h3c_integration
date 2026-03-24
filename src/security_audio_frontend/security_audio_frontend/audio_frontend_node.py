import os
import time
import uuid
import wave
import threading
from collections import deque

import numpy as np
import sounddevice as sd

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from security_audio_msgs.msg import AudioClipInfo


def compute_dbfs(audio: np.ndarray) -> float:
    if audio.size == 0:
        return -100.0
    rms = np.sqrt(np.mean(np.square(audio)))
    if rms < 1e-12:
        return -100.0
    return float(20.0 * np.log10(rms + 1e-12))


class AudioFrontendNode(Node):
    def __init__(self):
        super().__init__('audio_frontend_node')

        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('pre_trigger_sec', 1.0)
        self.declare_parameter('post_trigger_sec', 1.0)
        self.declare_parameter('trigger_dbfs', -35.0)
        self.declare_parameter('min_trigger_interval_sec', 1.5)
        self.declare_parameter('save_dir', '/tmp/security_audio_clips')
        self.declare_parameter('chunk_size', 1600)

        self.declare_parameter('device_name_contains', 'ReSpeaker')
        self.declare_parameter('device_index', -1)
        self.declare_parameter('input_channels', 1)
        self.declare_parameter('channel_index', 0)

        self.declare_parameter('doa_topic', '/sound/doa_deg')
        self.declare_parameter('enable_doa_sub', True)
        

        self.sample_rate = int(self.get_parameter('sample_rate').value)
        self.pre_trigger_sec = float(self.get_parameter('pre_trigger_sec').value)
        self.post_trigger_sec = float(self.get_parameter('post_trigger_sec').value)
        self.trigger_dbfs = float(self.get_parameter('trigger_dbfs').value)
        self.min_trigger_interval_sec = float(self.get_parameter('min_trigger_interval_sec').value)
        self.save_dir = str(self.get_parameter('save_dir').value)
        self.chunk_size = int(self.get_parameter('chunk_size').value)

        self.device_name_contains = str(self.get_parameter('device_name_contains').value)
        self.device_index = int(self.get_parameter('device_index').value)
        self.input_channels = int(self.get_parameter('input_channels').value)
        self.channel_index = int(self.get_parameter('channel_index').value)

        self.doa_topic = str(self.get_parameter('doa_topic').value)
        self.enable_doa_sub = bool(self.get_parameter('enable_doa_sub').value)

        os.makedirs(self.save_dir, exist_ok=True)

        self.pre_samples = int(self.sample_rate * self.pre_trigger_sec)
        self.post_samples = int(self.sample_rate * self.post_trigger_sec)

        self.ring_buffer = deque(maxlen=self.pre_samples)
        self.post_buffer = []

        self.collecting_post = False
        self.current_event_id = None
        self.current_level_dbfs = -100.0
        self.current_doa_deg = 0.0
        self.last_trigger_time = 0.0
        self.current_trigger_ros_time_ns = 0

        self.latest_doa_deg = 0.0
        self.completed_events = deque()
        self.lock = threading.Lock()

        self.clip_pub = self.create_publisher(AudioClipInfo, '/audio/clip_info', 10)

        if self.enable_doa_sub:
            self.doa_sub = self.create_subscription(
                Float32,
                self.doa_topic,
                self.doa_callback,
                10
            )

        self.timer = self.create_timer(0.05, self.flush_completed_events)

        self.stream = None
        self.start_audio_stream()

        self.get_logger().info('audio_frontend_node started')

    def doa_callback(self, msg: Float32):
        self.latest_doa_deg = float(msg.data)

    def find_input_device(self) -> int:
        if self.device_index >= 0:
            return self.device_index

        devices = sd.query_devices()
        for i, dev in enumerate(devices):
            name = dev.get('name', '')
            max_in = int(dev.get('max_input_channels', 0))
            if self.device_name_contains.lower() in name.lower() and max_in >= self.input_channels:
                return i

        raise RuntimeError(
            f'No input device found containing "{self.device_name_contains}" '
            f'with >= {self.input_channels} input channels'
        )

    def start_audio_stream(self):
        device_idx = self.find_input_device()
        dev = sd.query_devices(device_idx)

        self.get_logger().info(
            f'Using input device index={device_idx}, '
            f'name="{dev["name"]}", input_channels={self.input_channels}, '
            f'channel_index={self.channel_index}, sample_rate={self.sample_rate}'
        )

        self.stream = sd.InputStream(
            samplerate=self.sample_rate,
            blocksize=self.chunk_size,
            device=device_idx,
            channels=self.input_channels,
            dtype='float32',
            callback=self.audio_callback
        )
        self.stream.start()

    def audio_callback(self, indata, frames, time_info, status):
        if status:
            self.get_logger().warning(f'Audio callback status: {status}')

        try:
            audio_chunk = np.asarray(indata[:, self.channel_index], dtype=np.float32).copy()
        except Exception as e:
            self.get_logger().error(f'Failed to extract channel {self.channel_index}: {e}')
            return

        now = time.time()
        level_dbfs = compute_dbfs(audio_chunk)
        #self.get_logger().info(f'level_dbfs={level_dbfs:.2f}')

        with self.lock:
            self.ring_buffer.extend(audio_chunk.tolist())

            if self.collecting_post:
                self.post_buffer.extend(audio_chunk.tolist())

                if len(self.post_buffer) >= self.post_samples:
                    pre_audio = np.array(self.ring_buffer, dtype=np.float32)
                    if pre_audio.size > self.pre_samples:
                        pre_audio = pre_audio[-self.pre_samples:]

                    post_audio = np.array(self.post_buffer[:self.post_samples], dtype=np.float32)
                    full_audio = np.concatenate([pre_audio, post_audio])

                    actual_pre_samples = len(pre_audio)
                    clip_start_time_ns = self.current_trigger_ros_time_ns - int(
                        (actual_pre_samples / self.sample_rate) * 1e9
                    )

                    self.completed_events.append({
                        'event_id': self.current_event_id,
                        'audio': full_audio,
                        'level_dbfs': self.current_level_dbfs,
                        'doa_deg': self.current_doa_deg,
                        'clip_start_time_ns': clip_start_time_ns,
})

                    self.collecting_post = False
                    self.post_buffer = []
                    self.current_event_id = None
                return

            cooldown_ok = (now - self.last_trigger_time) >= self.min_trigger_interval_sec
            buffer_ready = len(self.ring_buffer) >= int(0.5 * self.pre_samples)

            if level_dbfs >= self.trigger_dbfs and cooldown_ok and buffer_ready:
                self.collecting_post = True
                self.current_event_id = str(uuid.uuid4())
                self.current_level_dbfs = level_dbfs
                self.current_doa_deg = self.latest_doa_deg
                self.current_trigger_ros_time_ns = self.get_clock().now().nanoseconds
                self.post_buffer = []
                self.last_trigger_time = now

                self.get_logger().info(
                    f'Trigger detected: event_id={self.current_event_id}, '
                    f'level={level_dbfs:.2f} dBFS, doa={self.current_doa_deg:.1f}'
                )

    def flush_completed_events(self):
        pending = []
        with self.lock:
            while self.completed_events:
                pending.append(self.completed_events.popleft())

        for item in pending:
            wav_path = self.save_wav(item['audio'], item['event_id'])
            self.publish_clip_info(
            event_id=item['event_id'],
            wav_path=wav_path,
            total_samples=len(item['audio']),
            level_dbfs=item['level_dbfs'],
            doa_deg=item['doa_deg'],
            clip_start_time_ns=item['clip_start_time_ns']
        )

    def save_wav(self, audio: np.ndarray, event_id: str) -> str:
        pcm16 = np.clip(audio, -1.0, 1.0)
        pcm16 = (pcm16 * 32767.0).astype(np.int16)

        wav_path = os.path.join(self.save_dir, f'{event_id}.wav')
        with wave.open(wav_path, 'wb') as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(self.sample_rate)
            wf.writeframes(pcm16.tobytes())
        return wav_path

    def publish_clip_info(self, event_id: str, wav_path: str, total_samples: int, level_dbfs: float, doa_deg: float, clip_start_time_ns: int):
        msg = AudioClipInfo()
        msg.event_id = event_id
        msg.stamp = self.get_clock().now().to_msg()
        msg.clip_start_time.sec = int(clip_start_time_ns // 1_000_000_000)
        msg.clip_start_time.nanosec = int(clip_start_time_ns % 1_000_000_000)
        msg.clip_wav_path = wav_path
        msg.doa_deg = float(doa_deg)
        msg.level_dbfs = float(level_dbfs)
        msg.duration_sec = float(total_samples / self.sample_rate)
        msg.sample_rate = int(self.sample_rate)

        self.clip_pub.publish(msg)
        self.get_logger().info(f'Published clip info: {wav_path}')

    def destroy_node(self):
        try:
            if self.stream is not None:
                self.stream.stop()
                self.stream.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AudioFrontendNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt received, shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()