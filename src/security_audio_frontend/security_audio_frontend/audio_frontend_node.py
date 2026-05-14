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
from std_msgs.msg import Float32, Bool, Int32

from security_audio_msgs.msg import AudioClipInfo
#from .doa_wpe_music import WpeMusicDoaEstimator

try:
    from .doa_wpe_music import WpeMusicDoaEstimator
except Exception:
    WpeMusicDoaEstimator = None


def compute_dbfs(x: np.ndarray, eps: float = 1e-12) -> float:
    rms = np.sqrt(np.mean(np.square(x), dtype=np.float64))
    return float(20.0 * np.log10(rms + eps))


def write_wav_mono_16bit(path: str, audio: np.ndarray, sample_rate: int):
    audio = np.asarray(audio, dtype=np.float32)
    audio = np.clip(audio, -1.0, 1.0)
    pcm16 = (audio * 32767.0).astype(np.int16)

    with wave.open(path, 'wb') as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(sample_rate)
        wf.writeframes(pcm16.tobytes())


class AudioFrontendNode(Node):
    def __init__(self):
        super().__init__('audio_frontend_node')

        # ---- basic params ----
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('pre_trigger_sec', 0.2)
        self.declare_parameter('post_trigger_sec', 0.8)
        self.declare_parameter('trigger_dbfs', -35.0)
        self.declare_parameter('min_trigger_interval_sec', 1.0)
        self.declare_parameter('trigger_consecutive_chunks', 1)
        self.declare_parameter('save_dir', '/home/chan/security_audio_clips')
        self.declare_parameter('chunk_size', 400)

        self.declare_parameter('device_name_contains', 'ReSpeaker')
        self.declare_parameter('device_index', 0)
        self.declare_parameter('input_channels', 6)
        self.declare_parameter('channel_index', 0)

        self.declare_parameter('doa_topic', '/sound/doa_deg')
        self.declare_parameter('enable_doa_sub', True)
        self.declare_parameter('upload_enable_topic', '/sound/upload_enable')
        self.declare_parameter('mic_mute_topic', '/sound/mic_mute_sec')
        self.declare_parameter('start_enabled', True)

        # ---- final mapping params (same convention as realtime DOA node) ----
        self.declare_parameter('front_offset_deg', 0.0)
        self.declare_parameter('invert_sign', False)

        # ---- refined DOA params ----
        self.declare_parameter('use_refined_doa', True)
        self.declare_parameter('refined_doa_use_realtime_fallback', True)
        self.declare_parameter('refined_doa_nfft', 256)
        self.declare_parameter('refined_doa_mic_radius', 0.032)
        self.declare_parameter('refined_doa_hardware_offset_deg', -135.0)
        self.declare_parameter('refined_doa_mic_indices_csv', '0,1,2,3')
        self.declare_parameter('refined_doa_wpe_taps', 5)
        self.declare_parameter('refined_doa_wpe_delay', 2)
        self.declare_parameter('refined_doa_wpe_iterations', 3)

        # ---- param values ----
        self.sample_rate = int(self.get_parameter('sample_rate').value)
        self.pre_trigger_sec = float(self.get_parameter('pre_trigger_sec').value)
        self.post_trigger_sec = float(self.get_parameter('post_trigger_sec').value)
        self.trigger_dbfs = float(self.get_parameter('trigger_dbfs').value)
        self.min_trigger_interval_sec = float(self.get_parameter('min_trigger_interval_sec').value)
        self.trigger_consecutive_chunks = int(self.get_parameter('trigger_consecutive_chunks').value)
        self.save_dir = str(self.get_parameter('save_dir').value)
        self.chunk_size = int(self.get_parameter('chunk_size').value)

        self.device_name_contains = str(self.get_parameter('device_name_contains').value)
        self.device_index = int(self.get_parameter('device_index').value)
        self.input_channels = int(self.get_parameter('input_channels').value)
        self.channel_index = int(self.get_parameter('channel_index').value)

        self.doa_topic = str(self.get_parameter('doa_topic').value)
        self.enable_doa_sub = bool(self.get_parameter('enable_doa_sub').value)
        self.upload_enable_topic = str(self.get_parameter('upload_enable_topic').value)
        self.mic_mute_topic = str(self.get_parameter('mic_mute_topic').value)
        self.audio_enabled = bool(self.get_parameter('start_enabled').value)

        self.front_offset_deg = float(self.get_parameter('front_offset_deg').value)
        self.invert_sign = bool(self.get_parameter('invert_sign').value)

        self.use_refined_doa = bool(self.get_parameter('use_refined_doa').value)
        self.refined_doa_use_realtime_fallback = bool(
            self.get_parameter('refined_doa_use_realtime_fallback').value
        )
        self.refined_doa_nfft = int(self.get_parameter('refined_doa_nfft').value)
        self.refined_doa_mic_radius = float(self.get_parameter('refined_doa_mic_radius').value)
        self.refined_doa_hardware_offset_deg = float(
            self.get_parameter('refined_doa_hardware_offset_deg').value
        )
        self.refined_doa_mic_indices = tuple(
            int(x.strip()) for x in
            str(self.get_parameter('refined_doa_mic_indices_csv').value).split(',')
            if x.strip()
        )
        self.refined_doa_wpe_taps = int(self.get_parameter('refined_doa_wpe_taps').value)
        self.refined_doa_wpe_delay = int(self.get_parameter('refined_doa_wpe_delay').value)
        self.refined_doa_wpe_iterations = int(
            self.get_parameter('refined_doa_wpe_iterations').value
        )

        os.makedirs(self.save_dir, exist_ok=True)

        self.pre_samples = int(self.sample_rate * self.pre_trigger_sec)
        self.post_samples = int(self.sample_rate * self.post_trigger_sec)

        self.lock = threading.Lock()

        # mono buffer for trigger
        self.ring_buffer = deque(maxlen=self.pre_samples)

        # multichannel buffer for refined doa
        self.multi_ring_buffer = deque(maxlen=self.pre_samples)

        self.post_buffer = []
        self.multi_post_buffer = []

        self.collecting_post = False
        self.completed_events = []

        self.current_event_id = None
        self.current_level_dbfs = -100.0
        self.current_doa_deg = 0.0
        self.current_trigger_ros_time_ns = 0

        self.current_pre_audio = None
        self.current_pre_multi = None

        self.last_trigger_time = 0.0
        self.consecutive_hit_count = 0
        self.latest_doa_deg = 0.0
        self.mic_mute_until_mono = 0.0

        if self.enable_doa_sub:
            self.create_subscription(Float32, self.doa_topic, self.doa_callback, 10)

        self.create_subscription(
            Bool,
            self.upload_enable_topic,
            self.upload_enable_callback,
            10
        )

        self.create_subscription(
            Int32,
            self.mic_mute_topic,
            self.mic_mute_callback,
            10
        )

        self.clip_pub = self.create_publisher(AudioClipInfo, '/audio/clip_info', 10)

        self.refined_doa_estimator = None
        if self.use_refined_doa:
            self.refined_doa_estimator = WpeMusicDoaEstimator(
                sample_rate=self.sample_rate,
                nfft=self.refined_doa_nfft,
                mic_radius=self.refined_doa_mic_radius,
                mic_channel_indices=self.refined_doa_mic_indices,
                hardware_offset_deg=self.refined_doa_hardware_offset_deg,
                wpe_taps=self.refined_doa_wpe_taps,
                wpe_delay=self.refined_doa_wpe_delay,
                wpe_iterations=self.refined_doa_wpe_iterations,
            )

        self.device_info = self.resolve_input_device()
        self.stream = sd.InputStream(
            samplerate=self.sample_rate,
            blocksize=self.chunk_size,
            device=self.device_index,
            channels=self.input_channels,
            dtype='float32',
            callback=self.audio_callback,
        )
        self.stream.start()

        self.create_timer(0.05, self.flush_completed_events)

        self.get_logger().info(
            f'Using input device index={self.device_index}, '
            f'name="{self.device_info["name"]}", '
            f'input_channels={self.input_channels}, '
            f'channel_index={self.channel_index}, '
            f'sample_rate={self.sample_rate}'
        )
        self.get_logger().info(
            f'audio_frontend_node started | upload_enable_topic={self.upload_enable_topic} | '
            f'mic_mute_topic={self.mic_mute_topic} | '
            f'audio_enabled={self.audio_enabled}'
        )

    def resolve_input_device(self):
        devices = sd.query_devices()

        if 0 <= self.device_index < len(devices):
            d = devices[self.device_index]
            if d['max_input_channels'] >= self.input_channels:
                return d

        for i, d in enumerate(devices):
            if self.device_name_contains.lower() in d['name'].lower():
                if d['max_input_channels'] >= self.input_channels:
                    self.device_index = i
                    return d

        raise RuntimeError(
            f'No suitable input device found for name_contains="{self.device_name_contains}" '
            f'and input_channels={self.input_channels}'
        )

    def doa_callback(self, msg: Float32):
        self.latest_doa_deg = float(msg.data)

    def reset_audio_state_locked(self):
        self.ring_buffer.clear()
        self.multi_ring_buffer.clear()
        self.post_buffer = []
        self.multi_post_buffer = []
        self.collecting_post = False
        self.completed_events.clear()
        self.current_event_id = None
        self.current_level_dbfs = -100.0
        self.current_doa_deg = 0.0
        self.current_trigger_ros_time_ns = 0
        self.current_pre_audio = None
        self.current_pre_multi = None
        self.consecutive_hit_count = 0

    def upload_enable_callback(self, msg: Bool):
        enabled = bool(msg.data)

        with self.lock:
            prev_enabled = self.audio_enabled
            self.audio_enabled = enabled

            if not enabled:
                self.reset_audio_state_locked()

        if prev_enabled != enabled:
            state_text = 'enabled' if enabled else 'disabled'
            self.get_logger().info(f'Audio frontend {state_text} by {self.upload_enable_topic}')

    def mic_mute_callback(self, msg: Int32):
        mute_sec = int(msg.data)

        if mute_sec <= 0:
            return

        mute_until = time.monotonic() + float(mute_sec)

        with self.lock:
            self.mic_mute_until_mono = max(self.mic_mute_until_mono, mute_until)
            self.reset_audio_state_locked()

        self.get_logger().info(
            f'Audio frontend temporary muted for {mute_sec} sec by {self.mic_mute_topic}'
        )

    @staticmethod
    def wrap_deg_pm180(deg: float) -> float:
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

    def audio_callback(self, indata, frames, time_info, status):
        if status:
            self.get_logger().warning(f'Audio callback status: {status}')

        frame_matrix = np.asarray(indata[:, :self.input_channels], dtype=np.float32).copy()
        audio_chunk = frame_matrix[:, self.channel_index].copy()
        level_dbfs = compute_dbfs(audio_chunk)
        now = time.monotonic()

        with self.lock:
            if not self.audio_enabled:
                return

            if now < self.mic_mute_until_mono:
                self.reset_audio_state_locked()
                return

            self.ring_buffer.extend(audio_chunk.tolist())
            self.multi_ring_buffer.extend(frame_matrix.tolist())

            if self.collecting_post:
                self.post_buffer.extend(audio_chunk.tolist())
                self.multi_post_buffer.extend(frame_matrix.tolist())

                if len(self.post_buffer) >= self.post_samples:
                    pre_audio = (
                        self.current_pre_audio
                        if self.current_pre_audio is not None
                        else np.array([], dtype=np.float32)
                    )
                    pre_multi = (
                        self.current_pre_multi
                        if self.current_pre_multi is not None
                        else np.zeros((0, self.input_channels), dtype=np.float32)
                    )

                    post_audio = np.array(
                        self.post_buffer[:self.post_samples],
                        dtype=np.float32
                    )
                    post_multi = np.array(
                        self.multi_post_buffer[:self.post_samples],
                        dtype=np.float32
                    )

                    full_audio = np.concatenate([pre_audio, post_audio], axis=0)
                    full_multi = np.concatenate([pre_multi, post_multi], axis=0)

                    actual_pre_samples = len(pre_audio)
                    clip_start_time_ns = self.current_trigger_ros_time_ns - int(
                        (actual_pre_samples / self.sample_rate) * 1e9
                    )

                    final_doa_deg = self.current_doa_deg

                    if self.use_refined_doa and self.refined_doa_estimator is not None:
                        try:
                            raw_refined_native_deg = self.refined_doa_estimator.estimate(full_multi)
                            mapped_refined_deg = self.map_angle(raw_refined_native_deg)
                            final_doa_deg = mapped_refined_deg

                            self.get_logger().info(
                                f'Refined DOA estimated: event_id={self.current_event_id}, '
                                f'native={raw_refined_native_deg:.1f}, final={mapped_refined_deg:.1f}'
                            )
                        except Exception as e:
                            if self.refined_doa_use_realtime_fallback:
                                self.get_logger().warning(
                                    f'Refined DOA failed, fallback to realtime DOA: {e}'
                                )
                                final_doa_deg = self.current_doa_deg
                            else:
                                self.get_logger().warning(
                                    f'Refined DOA failed and fallback disabled: {e}'
                                )
                                final_doa_deg = 0.0

                    self.completed_events.append({
                        'event_id': self.current_event_id,
                        'audio': full_audio,
                        'level_dbfs': self.current_level_dbfs,
                        'doa_deg': final_doa_deg,
                        'clip_start_time_ns': clip_start_time_ns,
                    })

                    self.collecting_post = False
                    self.post_buffer = []
                    self.multi_post_buffer = []
                    self.current_pre_audio = None
                    self.current_pre_multi = None
                    self.current_event_id = None
                    self.current_level_dbfs = -100.0
                    self.current_trigger_ros_time_ns = 0
                    self.consecutive_hit_count = 0

                return

            # trigger logic
            if level_dbfs >= self.trigger_dbfs:
                self.consecutive_hit_count += 1
            else:
                self.consecutive_hit_count = 0

            can_trigger = (now - self.last_trigger_time) >= self.min_trigger_interval_sec

            if self.consecutive_hit_count >= self.trigger_consecutive_chunks and can_trigger:
                self.collecting_post = True
                self.current_event_id = str(uuid.uuid4())
                self.current_level_dbfs = level_dbfs
                self.current_doa_deg = self.latest_doa_deg  # realtime mapped doa
                self.current_trigger_ros_time_ns = self.get_clock().now().nanoseconds

                self.current_pre_audio = np.array(self.ring_buffer, dtype=np.float32)
                if self.current_pre_audio.size > self.pre_samples:
                    self.current_pre_audio = self.current_pre_audio[-self.pre_samples:]

                self.current_pre_multi = np.array(self.multi_ring_buffer, dtype=np.float32)
                if len(self.current_pre_multi) > self.pre_samples:
                    self.current_pre_multi = self.current_pre_multi[-self.pre_samples:, :]

                self.post_buffer = []
                self.multi_post_buffer = []
                self.last_trigger_time = now

                self.get_logger().info(
                    f'Trigger detected: event_id={self.current_event_id}, '
                    f'level={level_dbfs:.2f} dBFS, doa={self.current_doa_deg:.1f}'
                )

    def flush_completed_events(self):
        with self.lock:
            if not self.audio_enabled:
                self.completed_events.clear()
                return
            if time.monotonic() < self.mic_mute_until_mono:
                self.completed_events.clear()
                return
            if not self.completed_events:
                return
            events = self.completed_events[:]
            self.completed_events.clear()

        for item in events:
            event_id = item['event_id']
            wav_path = os.path.join(self.save_dir, f'{event_id}.wav')

            write_wav_mono_16bit(
                wav_path,
                item['audio'],
                self.sample_rate
            )

            self.publish_clip_info(
                event_id=event_id,
                wav_path=wav_path,
                total_samples=len(item['audio']),
                level_dbfs=item['level_dbfs'],
                doa_deg=item['doa_deg'],
                clip_start_time_ns=item['clip_start_time_ns']
            )

            self.get_logger().info(f'Published clip info: {wav_path}')

    def publish_clip_info(
        self,
        event_id: str,
        wav_path: str,
        total_samples: int,
        level_dbfs: float,
        doa_deg: float,
        clip_start_time_ns: int
    ):
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

    def destroy_node(self):
        try:
            if hasattr(self, 'stream') and self.stream is not None:
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
        try:
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()