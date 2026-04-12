# webrtc_sender.py

"""
필수 패키지

sudo apt-get update

sudo apt-get install -y \
  python3-gi \
  python3-gst-1.0 \
  gstreamer1.0-tools \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly \
  gstreamer1.0-libav \
  python3-requests

확인
gst-inspect-1.0 webrtcbin
gst-inspect-1.0 nvv4l2h264enc
python3 -c "import gi; from gi.repository import Gst; Gst.init(None); print('gst ok')"
"""

# webrtc_sender.py

import sys
import threading
import time
from typing import Optional, Tuple

import cv2
import numpy as np
import requests

sys.stdout.reconfigure(line_buffering=True)

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import gi
gi.require_version("Gst", "1.0")
gi.require_version("GstWebRTC", "1.0")
gi.require_version("GstSdp", "1.0")

from gi.repository import Gst, GstWebRTC, GstSdp, GLib


class _WebRTCImageSubscriber(Node):
    def __init__(self, owner: "WebRTCSender", image_topic: str):
        super().__init__("webrtc_sender_subscriber")
        self.owner = owner
        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10,
        )

        self.timer = self.create_timer(
            1.0 / float(self.owner.fps),
            self.push_timer_callback,
        )

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warning(f"cv_bridge failed: {repr(e)}")
            return

        if frame is None:
            return

        if self.owner.resize_enabled:
            if frame.shape[1] != self.owner.width or frame.shape[0] != self.owner.height:
                frame = cv2.resize(
                    frame,
                    (self.owner.width, self.owner.height),
                    interpolation=cv2.INTER_AREA,
                )

        with self.owner._frame_lock:
            self.owner._latest_bgr = frame.copy()
            self.owner._latest_stamp_ns = time.time_ns()
            self.owner._frame_in_count += 1

    def push_timer_callback(self):
        with self.owner._frame_lock:
            if self.owner._latest_bgr is None or self.owner.appsrc is None:
                return
            frame = self.owner._latest_bgr
            stamp_ns = self.owner._latest_stamp_ns

        try:
            self.owner._push_frame(frame, stamp_ns)
            self.owner._frame_push_count += 1
        except Exception as e:
            self.get_logger().warning(f"push_frame failed: {repr(e)}")

        now = time.monotonic()
        if now - self.owner._last_stat_log_t >= 5.0:
            self.get_logger().info(
                f"frames_in={self.owner._frame_in_count}, "
                f"frames_pushed={self.owner._frame_push_count}"
            )
            self.owner._last_stat_log_t = now


class WebRTCSender:
    def __init__(
        self,
        signaling_base_url: str,
        image_topic: str = "/camera/color/image_raw",
        width: int = 1280,
        height: int = 720,
        fps: int = 30,
        bitrate: int = 4_000_000,
        resize: bool = True,
        use_hw_encoder: bool = True,
        stun_server: str = "stun://stun.l.google.com:19302",
        poll_interval_sec: float = 1.0,
    ):
        self.signaling_base_url = signaling_base_url.rstrip("/")
        self.image_topic = image_topic

        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)
        self.bitrate = int(bitrate)
        self.resize_enabled = bool(resize)
        self.use_hw_encoder = bool(use_hw_encoder)
        self.stun_server = str(stun_server)
        self.poll_interval_sec = float(poll_interval_sec)

        self.running = False

        self._frame_lock = threading.Lock()
        self._latest_bgr: Optional[np.ndarray] = None
        self._latest_stamp_ns: int = 0
        self._frame_in_count = 0
        self._frame_push_count = 0
        self._last_stat_log_t = time.monotonic()

        self._session_lock = threading.Lock()
        self._answer_ready_event = threading.Event()
        self._local_answer: Optional[Tuple[str, str]] = None

        self._glib_thread: Optional[threading.Thread] = None
        self._ros_thread: Optional[threading.Thread] = None
        self._signaling_thread: Optional[threading.Thread] = None

        self._ros_node: Optional[_WebRTCImageSubscriber] = None
        self._executor: Optional[SingleThreadedExecutor] = None

        self.loop: Optional[GLib.MainLoop] = None
        self.pipeline = None
        self.media_bin = None
        self.appsrc = None
        self.pay = None
        self.webrtc = None
        self.webrtc_sink_pad = None

        Gst.init(None)

    # =========================================================
    # Public API
    # =========================================================
    def start(self):
        print("🔥🔥 [WebRTC] start() called", flush=True)
        print("[WebRTC] signaling_base_url =", self.signaling_base_url)
        print("[WebRTC] image_topic =", self.image_topic)
        print("[WebRTC] size =", self.width, "x", self.height, "fps =", self.fps)
        print("[WebRTC] bitrate =", self.bitrate)
        print("[WebRTC] use_hw_encoder =", self.use_hw_encoder)

        if self.running:
            print("[WebRTC] already running")
            return

        self.running = True

        self.loop = GLib.MainLoop()
        self._glib_thread = threading.Thread(target=self._run_glib_loop, daemon=True)
        self._glib_thread.start()

        self._start_ros_subscriber()

        self._signaling_thread = threading.Thread(target=self._signaling_worker, daemon=True)
        self._signaling_thread.start()

        print("[WebRTC] started")

    def stop(self):
        print("[WebRTC] stop() called")
        self.running = False

        try:
            if self.appsrc is not None:
                try:
                    self.appsrc.emit("end-of-stream")
                except Exception:
                    pass

            if self.pipeline is not None:
                self.pipeline.set_state(Gst.State.NULL)
        except Exception as e:
            print("[WebRTC] pipeline stop error:", repr(e))

        try:
            if self._executor is not None:
                self._executor.shutdown()
        except Exception as e:
            print("[WebRTC] executor shutdown error:", repr(e))

        try:
            if self._ros_node is not None:
                self._ros_node.destroy_node()
        except Exception as e:
            print("[WebRTC] ros node destroy error:", repr(e))

        try:
            if self.loop is not None and self.loop.is_running():
                self.loop.quit()
        except Exception as e:
            print("[WebRTC] glib loop quit error:", repr(e))

    # =========================================================
    # ROS
    # =========================================================
    def _start_ros_subscriber(self):
        self._ros_node = _WebRTCImageSubscriber(self, self.image_topic)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._ros_node)

        self._ros_thread = threading.Thread(target=self._spin_ros, daemon=True)
        self._ros_thread.start()

    def _spin_ros(self):
        print("[WebRTC] ROS subscriber spin started")
        try:
            if self._executor is not None:
                self._executor.spin()
        except Exception as e:
            print("[WebRTC] ROS spin error:", repr(e))

    # =========================================================
    # GLib
    # =========================================================
    def _run_glib_loop(self):
        print("[WebRTC] GLib loop started")
        try:
            if self.loop is not None:
                self.loop.run()
        except Exception as e:
            print("[WebRTC] GLib loop error:", repr(e))

    # =========================================================
    # GStreamer
    # =========================================================
    def _media_bin_string(self) -> str:
        if self.use_hw_encoder:
            enc = (
                f"videoconvert ! video/x-raw,format=I420 "
                f"! nvvidconv "
                f"! video/x-raw(memory:NVMM),format=NV12 "
                f"! nvv4l2h264enc bitrate={self.bitrate} "
                f"insert-sps-pps=true iframeinterval={self.fps} idrinterval={self.fps} "
                f"control-rate=1 preset-level=1 maxperf-enable=1 "
            )
        else:
            kbps = max(1, self.bitrate // 1000)
            enc = (
                f"videoconvert "
                f"! x264enc tune=zerolatency speed-preset=ultrafast "
                f"bitrate={kbps} key-int-max={self.fps} "
            )

        return (
            f'appsrc name=src is-live=true block=false format=time do-timestamp=true '
            f'caps=video/x-raw,format=BGR,width={self.width},height={self.height},framerate={self.fps}/1 '
            f'! queue leaky=downstream max-size-buffers=1 '
            f'! {enc} '
            f'! h264parse config-interval=-1 '
            f'! rtph264pay name=pay pt=96 config-interval=1'
        )

    def _cleanup_old_pipeline(self):
        if self.pipeline is not None:
            try:
                self.pipeline.set_state(Gst.State.NULL)
            except Exception as e:
                print("[WebRTC] old pipeline shutdown failed:", repr(e))

        self.pipeline = None
        self.media_bin = None
        self.appsrc = None
        self.pay = None
        self.webrtc = None
        self.webrtc_sink_pad = None

    def _request_webrtc_sink_pad(self):
        pad = None

        if hasattr(self.webrtc, "request_pad_simple"):
            try:
                pad = self.webrtc.request_pad_simple("sink_%u")
            except Exception as e:
                print("[WebRTC] request_pad_simple failed:", repr(e))

        if pad is None and hasattr(self.webrtc, "get_request_pad"):
            try:
                pad = self.webrtc.get_request_pad("sink_%u")
            except Exception as e:
                print("[WebRTC] get_request_pad failed:", repr(e))

        if pad is None:
            try:
                templ = self.webrtc.get_pad_template("sink_%u")
                if templ is not None:
                    pad = self.webrtc.request_pad(templ, None, None)
            except Exception as e:
                print("[WebRTC] request_pad(template) failed:", repr(e))

        return pad

    def _build_pipeline(self):
        with self._session_lock:
            self._answer_ready_event.clear()
            self._local_answer = None

            self._cleanup_old_pipeline()

            media_str = self._media_bin_string()
            print("[WebRTC] creating GStreamer media bin:")
            print(media_str)

            self.pipeline = Gst.Pipeline.new("webrtc-pipeline")
            if self.pipeline is None:
                raise RuntimeError("failed to create Gst.Pipeline")

            self.media_bin = Gst.parse_bin_from_description(media_str, True)
            if self.media_bin is None:
                raise RuntimeError("failed to create media bin from description")

            self.webrtc = Gst.ElementFactory.make("webrtcbin", "webrtc")
            if self.webrtc is None:
                raise RuntimeError("failed to create webrtcbin")

            self.webrtc.set_property("bundle-policy", "max-bundle")
            self.webrtc.set_property("stun-server", self.stun_server)

            if not self.pipeline.add(self.media_bin):
                raise RuntimeError("failed to add media bin to pipeline")

            if not self.pipeline.add(self.webrtc):
                raise RuntimeError("failed to add webrtcbin to pipeline")

            self.appsrc = self.media_bin.get_by_name("src")
            self.pay = self.media_bin.get_by_name("pay")

            if self.appsrc is None:
                raise RuntimeError("failed to get appsrc named 'src'")
            if self.pay is None:
                raise RuntimeError("failed to get rtph264pay named 'pay'")

            pay_src_pad = self.pay.get_static_pad("src")
            if pay_src_pad is None:
                raise RuntimeError("failed to get pay src pad")

            # webrtcbin이 pad 요청 받을 준비를 하도록 먼저 READY/PAUSED로 올림
            ret = self.pipeline.set_state(Gst.State.PAUSED)
            if ret == Gst.StateChangeReturn.FAILURE:
                raise RuntimeError("failed to set pipeline PAUSED before requesting pad")

            time.sleep(0.2)

            self.webrtc_sink_pad = self._request_webrtc_sink_pad()
            if self.webrtc_sink_pad is None:
                raise RuntimeError("failed to request webrtc sink pad")

            sink_caps = self.webrtc_sink_pad.query_caps(None)
            print("[WebRTC] webrtc sink caps =", sink_caps.to_string() if sink_caps else "None")

            pay_caps = pay_src_pad.query_caps(None)
            print("[WebRTC] pay src caps =", pay_caps.to_string() if pay_caps else "None")

            link_ret = pay_src_pad.link(self.webrtc_sink_pad)
            if link_ret != Gst.PadLinkReturn.OK:
                raise RuntimeError(f"failed to link rtph264pay to webrtcbin: {link_ret}")

            caps = Gst.Caps.from_string(
                f"video/x-raw,format=BGR,width={self.width},height={self.height},framerate={self.fps}/1"
            )
            self.appsrc.set_property("caps", caps)
            self.appsrc.set_property("is-live", True)
            self.appsrc.set_property("block", False)
            self.appsrc.set_property("format", Gst.Format.TIME)
            self.appsrc.set_property("do-timestamp", True)

            bus = self.pipeline.get_bus()
            bus.add_signal_watch()
            bus.connect("message", self._on_bus_message)

            self.webrtc.connect("on-ice-candidate", self._on_ice_candidate)
            self.webrtc.connect("notify::ice-gathering-state", self._on_ice_gathering_state_changed)
            self.webrtc.connect("notify::ice-connection-state", self._on_ice_connection_state_changed)
            self.webrtc.connect("notify::connection-state", self._on_connection_state_changed)
            self.webrtc.connect("notify::signaling-state", self._on_signaling_state_changed)

            ret = self.pipeline.set_state(Gst.State.PLAYING)
            if ret == Gst.StateChangeReturn.FAILURE:
                raise RuntimeError("failed to set pipeline PLAYING")

            print("[WebRTC] GStreamer pipeline PLAYING")

    def _push_frame(self, frame_bgr: np.ndarray, stamp_ns: int):
        if self.appsrc is None:
            return

        if not frame_bgr.flags["C_CONTIGUOUS"]:
            frame_bgr = np.ascontiguousarray(frame_bgr)

        data = frame_bgr.tobytes()
        buf = Gst.Buffer.new_allocate(None, len(data), None)
        buf.fill(0, data)

        duration_ns = int(1e9 / float(self.fps))
        pts = stamp_ns if stamp_ns > 0 else time.time_ns()

        buf.pts = pts
        buf.dts = pts
        buf.duration = duration_ns

        ret = self.appsrc.emit("push-buffer", buf)
        if ret != Gst.FlowReturn.OK:
            raise RuntimeError(f"appsrc push-buffer returned {ret}")

    # =========================================================
    # Signaling
    # =========================================================
    def _signaling_worker(self):
        print("🔥🔥 [WebRTC] signaling worker started", flush=True)

        while self.running:
            try:
                poll_url = f"{self.signaling_base_url}/sender_poll"
                resp = requests.get(poll_url, timeout=35.0)

                if resp.status_code == 404:
                    time.sleep(self.poll_interval_sec)
                    continue

                if resp.status_code != 200:
                    print("[WebRTC] sender_poll failed:", resp.status_code, resp.text[:300])
                    time.sleep(self.poll_interval_sec)
                    continue

                offer = resp.json()
                sdp = offer.get("sdp")
                sdp_type = offer.get("type")

                if not sdp or not sdp_type:
                    print("[WebRTC] invalid offer payload:", offer)
                    time.sleep(self.poll_interval_sec)
                    continue

                print(f"🔥🔥 [WebRTC] offer received: type={sdp_type}, sdp_len={len(sdp)}", flush=True)

                self._build_pipeline()
                self._set_remote_description(sdp, sdp_type)
                self._create_answer()

                ok = self._answer_ready_event.wait(timeout=20.0)
                if not ok or self._local_answer is None:
                    raise RuntimeError("timeout waiting for local answer")

                local_sdp, local_type = self._local_answer

                ans_url = f"{self.signaling_base_url}/sender_answer"
                payload = {"sdp": local_sdp, "type": local_type}
                r = requests.post(ans_url, json=payload, timeout=10.0)

                if r.status_code != 200:
                    raise RuntimeError(
                        f"sender_answer failed: status={r.status_code}, body={r.text[:300]}"
                    )

                print(f"[WebRTC] answer posted successfully: type={local_type}, sdp_len={len(local_sdp)}")

            except Exception as e:
                print("[WebRTC] signaling worker error:", repr(e))
                time.sleep(1.0)

    def _set_remote_description(self, sdp_text: str, sdp_type: str):
        if self.webrtc is None:
            raise RuntimeError("webrtcbin is None")

        type_map = {
            "offer": GstWebRTC.WebRTCSDPType.OFFER,
            "answer": GstWebRTC.WebRTCSDPType.ANSWER,
            "pranswer": GstWebRTC.WebRTCSDPType.PRANSWER,
            "rollback": GstWebRTC.WebRTCSDPType.ROLLBACK,
        }

        if sdp_type not in type_map:
            raise RuntimeError(f"unsupported SDP type: {sdp_type}")

        res, sdpmsg = GstSdp.SDPMessage.new()
        if res != GstSdp.SDPResult.OK:
            raise RuntimeError(f"failed to create SDP message: {res}")

        parse_res = GstSdp.sdp_message_parse_buffer(bytes(sdp_text.encode("utf-8")), sdpmsg)
        if parse_res != GstSdp.SDPResult.OK:
            raise RuntimeError(f"failed to parse remote SDP: {parse_res}")

        desc = GstWebRTC.WebRTCSessionDescription.new(type_map[sdp_type], sdpmsg)
        promise = Gst.Promise.new()
        self.webrtc.emit("set-remote-description", desc, promise)
        promise.interrupt()

        print("[WebRTC] remote description set")

    def _create_answer(self):
        if self.webrtc is None:
            raise RuntimeError("webrtcbin is None")

        self._answer_ready_event.clear()
        self._local_answer = None

        promise = Gst.Promise.new_with_change_func(self._on_answer_created, None, None)
        self.webrtc.emit("create-answer", None, promise)
        print("[WebRTC] create-answer emitted")

    def _on_answer_created(self, promise: Gst.Promise, _unused1, _unused2):
        try:
            reply = promise.get_reply()
            answer = reply.get_value("answer")
            if answer is None:
                raise RuntimeError("create-answer returned no answer")

            print("[WebRTC] answer created, setting local description")

            local_promise = Gst.Promise.new()
            self.webrtc.emit("set-local-description", answer, local_promise)
            local_promise.interrupt()

            threading.Thread(
                target=self._wait_for_complete_local_description,
                daemon=True,
            ).start()

        except Exception as e:
            print("[WebRTC] _on_answer_created failed:", repr(e))

    def _wait_for_complete_local_description(self):
        try:
            if self.webrtc is None:
                raise RuntimeError("webrtcbin is None")

            t_end = time.monotonic() + 15.0
            while time.monotonic() < t_end:
                try:
                    state = self.webrtc.get_property("ice-gathering-state")
                    state_name = self._enum_nick(state)
                    if state_name == "complete":
                        break
                except Exception:
                    pass
                time.sleep(0.1)

            local_desc = self.webrtc.get_property("local-description")
            if local_desc is None:
                raise RuntimeError("local-description is None")

            sdp_text = local_desc.sdp.as_text()
            sdp_type = self._sdp_type_to_string(local_desc.type)

            if not sdp_text or not sdp_type:
                raise RuntimeError("invalid local SDP generated")

            self._local_answer = (sdp_text, sdp_type)
            self._answer_ready_event.set()

            print(f"[WebRTC] local answer ready: type={sdp_type}, sdp_len={len(sdp_text)}")

        except Exception as e:
            print("[WebRTC] _wait_for_complete_local_description failed:", repr(e))

    # =========================================================
    # GStreamer callbacks
    # =========================================================
    def _on_ice_candidate(self, element, mlineindex, candidate):
        print(f"[WebRTC] on-ice-candidate mline={mlineindex}, candidate_len={len(candidate)}")

    def _on_ice_gathering_state_changed(self, element, _pspec):
        try:
            state = element.get_property("ice-gathering-state")
            print("[WebRTC] ice-gathering-state =", self._enum_nick(state))
        except Exception as e:
            print("[WebRTC] ice-gathering-state read failed:", repr(e))

    def _on_ice_connection_state_changed(self, element, _pspec):
        try:
            state = element.get_property("ice-connection-state")
            print("[WebRTC] ice-connection-state =", self._enum_nick(state))
        except Exception as e:
            print("[WebRTC] ice-connection-state read failed:", repr(e))

    def _on_connection_state_changed(self, element, _pspec):
        try:
            state = element.get_property("connection-state")
            print("[WebRTC] connection-state =", self._enum_nick(state))
        except Exception as e:
            print("[WebRTC] connection-state read failed:", repr(e))

    def _on_signaling_state_changed(self, element, _pspec):
        try:
            state = element.get_property("signaling-state")
            print("[WebRTC] signaling-state =", self._enum_nick(state))
        except Exception as e:
            print("[WebRTC] signaling-state read failed:", repr(e))

    def _on_bus_message(self, bus, message):
        msg_type = message.type

        if msg_type == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print("[GStreamer][ERROR]", err, "/", debug)

        elif msg_type == Gst.MessageType.WARNING:
            err, debug = message.parse_warning()
            print("[GStreamer][WARNING]", err, "/", debug)

        elif msg_type == Gst.MessageType.EOS:
            print("[GStreamer] EOS received")

        elif msg_type == Gst.MessageType.STATE_CHANGED:
            if message.src == self.pipeline:
                old_state, new_state, pending = message.parse_state_changed()
                print(f"[GStreamer] pipeline state: {old_state.value_nick} -> {new_state.value_nick}")

    # =========================================================
    # Utils
    # =========================================================
    def _enum_nick(self, enum_val) -> str:
        try:
            return enum_val.value_nick
        except Exception:
            return str(enum_val)

    def _sdp_type_to_string(self, sdp_type) -> str:
        if sdp_type == GstWebRTC.WebRTCSDPType.OFFER:
            return "offer"
        if sdp_type == GstWebRTC.WebRTCSDPType.ANSWER:
            return "answer"
        if sdp_type == GstWebRTC.WebRTCSDPType.PRANSWER:
            return "pranswer"
        if sdp_type == GstWebRTC.WebRTCSDPType.ROLLBACK:
            return "rollback"
        return ""