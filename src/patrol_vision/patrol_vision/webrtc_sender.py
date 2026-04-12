# webrtc_sender.py
import asyncio
import threading
from typing import Optional

import cv2
import numpy as np
from av import VideoFrame
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
import requests


class BufferVideoTrack(VideoStreamTrack):
    def __init__(self, buffer):
        super().__init__()
        self.buffer = buffer
        self._last_frame: Optional[np.ndarray] = None

    async def recv(self):
        pts, time_base = await self.next_timestamp()

        try:
            rgb = await asyncio.to_thread(self.buffer.wait_new, 1.0)
            self._last_frame = rgb
        except Exception:
            rgb = self._last_frame

        if rgb is None:
            rgb = np.zeros((720, 1280, 3), dtype=np.uint8)

        rgb = cv2.resize(rgb, (1280, 720), interpolation=cv2.INTER_AREA)

        frame = VideoFrame.from_ndarray(rgb, format="rgb24")
        frame.pts = pts
        frame.time_base = time_base
        return frame


class WebRTCSender:
    def __init__(self, buffer, signaling_base_url: str):
        self.buffer = buffer
        self.signaling_base_url = signaling_base_url.rstrip("/")
        self.thread: Optional[threading.Thread] = None
        self.running = False
        self.pc: Optional[RTCPeerConnection] = None

    def start(self):
        print("[WebRTC] start() called")
        print("[WebRTC] signaling_base_url =", self.signaling_base_url)

        if self.thread is not None and self.thread.is_alive():
            print("[WebRTC] thread already alive")
            return

        self.running = True
        self.thread = threading.Thread(target=self._thread_main, daemon=True)
        self.thread.start()
        print("[WebRTC] thread started")

    def stop(self):
        print("[WebRTC] stop() called")
        self.running = False

    def _thread_main(self):
        print("[WebRTC] _thread_main entered")
        try:
            asyncio.run(self._run())
        except Exception as e:
            print("[WebRTC] _thread_main fatal error:", repr(e))

    async def _http_get(self, url: str, timeout: float):
        return await asyncio.to_thread(requests.get, url, timeout=timeout)

    async def _http_post(self, url: str, json_data: dict, timeout: float):
        return await asyncio.to_thread(
            requests.post,
            url,
            json=json_data,
            timeout=timeout,
        )

    async def _run(self):
        print("[WebRTC] _run loop started")

        while self.running:
            pc = None
            try:
                poll_url = f"{self.signaling_base_url}/sender_poll"
                print("[WebRTC] polling:", poll_url)

                resp = await self._http_get(poll_url, timeout=35.0)
                print("[WebRTC] sender_poll status =", resp.status_code)

                if resp.status_code != 200:
                    try:
                        print("[WebRTC] sender_poll body =", resp.text[:300])
                    except Exception:
                        pass
                    await asyncio.sleep(1.0)
                    continue

                offer_data = resp.json()
                print("[WebRTC] offer received")
                print("[WebRTC] offer keys =", list(offer_data.keys()))

                if "sdp" not in offer_data or "type" not in offer_data:
                    print("[WebRTC] invalid offer_data:", offer_data)
                    await asyncio.sleep(1.0)
                    continue

                pc = RTCPeerConnection()
                self.pc = pc
                print("[WebRTC] RTCPeerConnection created")

                @pc.on("connectionstatechange")
                async def on_connectionstatechange():
                    print("[WebRTC] sender state:", pc.connectionState)

                @pc.on("iceconnectionstatechange")
                async def on_iceconnectionstatechange():
                    print("[WebRTC] ice state:", pc.iceConnectionState)

                @pc.on("icegatheringstatechange")
                async def on_icegatheringstatechange():
                    print("[WebRTC] ice gathering state:", pc.iceGatheringState)

                pc.addTrack(BufferVideoTrack(self.buffer))
                print("[WebRTC] track added")

                offer = RTCSessionDescription(
                    sdp=offer_data["sdp"],
                    type=offer_data["type"],
                )

                print("[WebRTC] about to setRemoteDescription")
                await pc.setRemoteDescription(offer)
                print("[WebRTC] remote description set")

                print("[WebRTC] about to createAnswer")
                answer = await pc.createAnswer()
                print("[WebRTC] answer created")

                print("[WebRTC] about to setLocalDescription")
                await pc.setLocalDescription(answer)
                print("[WebRTC] local description set")

                if pc.localDescription is None:
                    print("[WebRTC] ERROR: localDescription is None")
                    await asyncio.sleep(1.0)
                    continue

                print("[WebRTC] localDescription.type =", pc.localDescription.type)
                print("[WebRTC] localDescription.sdp_len =", len(pc.localDescription.sdp))

                ans_url = f"{self.signaling_base_url}/sender_answer"
                payload = {
                    "sdp": pc.localDescription.sdp,
                    "type": pc.localDescription.type,
                }

                print("[WebRTC] posting answer to:", ans_url)
                r = await self._http_post(ans_url, json_data=payload, timeout=10.0)
                print("[WebRTC] sender_answer post status =", r.status_code)
                try:
                    print("[WebRTC] sender_answer post body =", r.text[:300])
                except Exception:
                    pass

                if r.status_code != 200:
                    print("[WebRTC] sender_answer post failed")
                    await asyncio.sleep(1.0)
                    continue

                print("[WebRTC] answer posted successfully")

                while self.running:
                    state = pc.connectionState
                    if state in ("failed", "closed", "disconnected"):
                        print("[WebRTC] leaving connection loop, state =", state)
                        break
                    await asyncio.sleep(1.0)

            except Exception as e:
                print("[WebRTC] sender error:", repr(e))

            finally:
                if pc is not None:
                    try:
                        await pc.close()
                        print("[WebRTC] pc closed")
                    except Exception as e:
                        print("[WebRTC] pc close error:", repr(e))

                self.pc = None

            await asyncio.sleep(1.0)