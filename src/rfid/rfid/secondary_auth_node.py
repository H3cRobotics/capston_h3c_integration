#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
import subprocess
import json
import wave
import math

from datetime import datetime
from threading import Lock
from typing import Optional

import cv2
import requests
import pygame  # 오디오 재생을 위한 라이브러리
import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool, Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from smartcard.System import readers
from smartcard.Exceptions import CardConnectionException, NoCardException


class SecondaryAuthNode(Node):
    def __init__(self):
        super().__init__('secondary_auth_node')

        # =========================
        # 사용자 설정
        # =========================
        self.server_base_url = 'http://192.168.0.221:8000'

        self.follow_person_id_topic = '/person_tracking/follow_person_id'
        self.annotated_topic = '/person_tracking/annotated'
        self.auth_ready_topic = '/auth_ready'
        self.mic_mute_topic = '/sound/mic_mute_sec'

        self.reader_name_hint = 'ACR122'
        self.auth_timeout_sec = 10.0
        self.poll_period_sec = 0.2
        self.request_timeout_sec = 5.0
        self.jpeg_quality = 90
        self.rfid_read_cooldown_sec = 1.5

        self.mic_mute_fallback_sec = 3
        self.mic_mute_margin_sec = 0.5

        # =========================
        # 오디오(Pygame) 초기화
        # =========================
        self.audio_available = False
        self.audio_channel = None
        self.current_sound = None

        try:
            pygame.mixer.init()
            pygame.mixer.set_num_channels(4)

            # 현재 스크립트가 위치한 폴더의 절대 경로를 스스로 찾아냅니다.
            self.audio_dir = os.path.dirname(os.path.realpath(__file__))

            # 짧은 안내음은 music stream보다 전용 Channel로 재생하는 편이 안정적입니다.
            self.audio_channel = pygame.mixer.Channel(0)
            self.audio_available = True

            self.get_logger().info('오디오 믹서 초기화 성공: 오디오 알림이 활성화되었습니다.')
        except Exception as e:
            self.get_logger().warn(f'오디오 믹서 초기화 실패 (소리가 나지 않을 수 있습니다): {e}')

        # =========================
        # 내부 상태
        # =========================
        self.bridge = CvBridge()
        self.lock = Lock()
        self.http = requests.Session()

        self.latest_follow_person_id: str = ""
        self.latest_auth_frame = None
        self.auth_start_frame = None

        self.auth_in_progress: bool = False
        self.auth_event_id: Optional[str] = None
        self.auth_started_mono: float = 0.0

        # =========================
        # ROS 인터페이스
        # =========================
        self.create_subscription(String, self.follow_person_id_topic, self.follow_person_id_cb, 10)
        self.create_subscription(Image, self.annotated_topic, self.annotated_cb, 10)
        self.create_subscription(Bool, self.auth_ready_topic, self.auth_ready_cb, 10)
        self.auth_result_pub = self.create_publisher(
            String,
            "/auth/result",
            10
        )
        self.mic_mute_pub = self.create_publisher(
            Int32,
            self.mic_mute_topic,
            10
        )
        self.last_auth_ready: bool = False
        self.last_uid: Optional[str] = None
        self.last_uid_read_ts: float = 0.0
        # /auth/result 반복 publish용 상태
        self.pending_auth_result_payload: Optional[str] = None
        self.pending_auth_publish_count: int = 0
        self.max_auth_publish_count: int = 5
        

        # PC/SC 워밍업
        self.warmup_pcsc()

        self.timer = self.create_timer(self.poll_period_sec, self.poll_loop)
        self.auth_result_timer = self.create_timer(
            0.05,
            self.auth_result_publish_loop
        )

        self.get_logger().info(
            f'SecondaryAuthNode started | '
            f'server={self.server_base_url} | '
            f'reader_hint={self.reader_name_hint} | '
            f'mic_mute_topic={self.mic_mute_topic}'
        )

    # =========================
    # 오디오 재생 전담 헬퍼 함수
    # =========================
    def get_wav_duration_sec(self, filepath: str) -> float:
        try:
            with wave.open(filepath, 'rb') as wf:
                frames = wf.getnframes()
                rate = wf.getframerate()

                if rate <= 0:
                    return float(self.mic_mute_fallback_sec)

                return frames / float(rate)
        except Exception as e:
            self.get_logger().warn(f'wav 길이 계산 실패, 기본 mute 시간 사용: {e}')
            return float(self.mic_mute_fallback_sec)

    def calc_mic_mute_sec(self, filepath: str) -> int:
        duration_sec = self.get_wav_duration_sec(filepath)
        mute_sec = int(math.ceil(duration_sec + self.mic_mute_margin_sec))
        return max(1, mute_sec)

    def publish_mic_mute(self, mute_sec: int):
        msg = Int32()
        msg.data = int(mute_sec)
        self.mic_mute_pub.publish(msg)
        self.get_logger().info(f'마이크 임시 비활성화 요청: {mute_sec} sec')

    def play_sound(self, filename: str):
        """지정된 오디오 파일을 백그라운드에서 비동기로 부드럽게 재생합니다."""
        if not self.audio_available or self.audio_channel is None:
            self.get_logger().warn('오디오 믹서가 초기화되지 않아 소리를 재생할 수 없습니다.')
            return

        if not hasattr(self, 'audio_dir'):
            return

        if os.path.isabs(filename):
            filepath = filename
        else:
            filepath = os.path.join(self.audio_dir, filename)

        if os.path.exists(filepath):
            try:
                mute_sec = self.calc_mic_mute_sec(filepath)
                self.publish_mic_mute(mute_sec)

                # 이전 안내음이 아직 재생 중이면 확실히 정지한 뒤 새 안내음을 재생합니다.
                self.audio_channel.stop()

                self.current_sound = pygame.mixer.Sound(filepath)
                self.audio_channel.play(self.current_sound)

                self.get_logger().info(f'오디오 재생 중: {filepath}, mute={mute_sec} sec')
            except Exception as e:
                self.get_logger().error(f'오디오 재생 중 오류 발생: {e}')
        else:
            self.get_logger().error(f'오디오 파일을 찾을 수 없습니다 (해당 경로에 파일을 넣어주세요): {filepath}')

    # =========================
    # PC/SC 워밍업
    # =========================
    def warmup_pcsc(self):
        self.get_logger().info('warming up pcsc with pcsc_scan...')
        try:
            subprocess.run(['bash', '-lc', 'timeout 2s pcsc_scan >/dev/null 2>&1 || true'], check=False)
            self.get_logger().info('pcsc warmup done')
        except Exception as e:
            self.get_logger().warn(f'pcsc warmup failed: {e}')

    # =========================
    # ROS callbacks
    # =========================
    def follow_person_id_cb(self, msg: String):
        new_id = msg.data.strip()
        if new_id == "":
            return
        with self.lock:
            self.latest_follow_person_id = new_id

    def annotated_cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'annotated image convert failed: {e}')
            return
        with self.lock:
            self.latest_auth_frame = frame.copy()

    def auth_ready_cb(self, msg: Bool):
        if msg.data and not self.last_auth_ready:
            self.start_auth()
        self.last_auth_ready = msg.data
    
    def request_publish_auth_result(self, auth_event_id: str, status: str):
        """
        /auth/result를 JSON 문자열로 timer loop에서 짧게 여러 번 publish하도록 요청.
        payload:
        {"auth_event_id": "...", "status": "success|fail|timeout"}
        """
        payload = {
            "auth_event_id": str(auth_event_id),
            "status": status.strip().lower(),
        }

        with self.lock:
            self.pending_auth_result_payload = json.dumps(payload, ensure_ascii=False)
            self.pending_auth_publish_count = 0


    def auth_result_publish_loop(self):
        """
        time.sleep() 없이 timer 기반으로 /auth/result를 0.05초 간격으로 5회 publish.
        """
        with self.lock:
            payload = self.pending_auth_result_payload

            if payload is None:
                return

            if self.pending_auth_publish_count >= self.max_auth_publish_count:
                self.pending_auth_result_payload = None
                self.pending_auth_publish_count = 0
                return

            self.pending_auth_publish_count += 1
            count = self.pending_auth_publish_count

        msg = String()
        msg.data = payload
        self.auth_result_pub.publish(msg)

        self.get_logger().info(
            f"[AUTH] published /auth/result "
            f"({count}/{self.max_auth_publish_count}) {payload}"
        )

    # =========================
    # 인증 시작
    # =========================
    def start_auth(self):
        with self.lock:
            if self.auth_in_progress:
                self.get_logger().warn('auth already in progress')
                return
            tracking_person_id = self.latest_follow_person_id
            frame_snapshot = None if self.latest_auth_frame is None else self.latest_auth_frame.copy()

        if tracking_person_id == "":
            self.get_logger().warn('latest_follow_person_id is empty')
            return

        payload = {"tracking_person_id": tracking_person_id}

        try:
            resp = self.http.post(
                f'{self.server_base_url}/auth/start',
                json=payload,
                timeout=self.request_timeout_sec
            )
            resp.raise_for_status()
            data = resp.json()
        except Exception as e:
            self.get_logger().error(f'/auth/start failed: {e}')
            return

        if not data.get("ok", False):
            self.get_logger().error(f'/auth/start response not ok: {data}')
            return

        auth_event_id = data.get("auth_event_id")
        if not auth_event_id:
            self.get_logger().error(f'no auth_event_id in response: {data}')
            return

        with self.lock:
            self.auth_in_progress = True
            self.auth_event_id = auth_event_id
            self.auth_start_frame = frame_snapshot
            self.auth_started_mono = time.monotonic()

        self.get_logger().info(f'auth started | tracking_person_id={tracking_person_id} | auth_event_id={auth_event_id}')
        
        # 🟢 서버가 인증을 무사히 접수했으므로 시작음을 울립니다.
        self.play_sound('/home/chan/capston_h3c_integration/src/rfid/rfid/start.wav')

    # =========================
    # 주기 루프
    # =========================
    def poll_loop(self):
        with self.lock:
            in_progress = self.auth_in_progress
            started_mono = self.auth_started_mono

        if not in_progress:
            return

        if time.monotonic() - started_mono >= self.auth_timeout_sec:
            self.send_timeout()
            return

        uid = self.read_uid_once()
        if uid is None:
            return

        now = time.monotonic()
        if self.last_uid == uid and (now - self.last_uid_read_ts) < self.rfid_read_cooldown_sec:
            return

        self.last_uid = uid
        self.last_uid_read_ts = now

        self.send_rfid(uid)

    # =========================
    # ACR122U UID 읽기
    # =========================
    def select_reader(self):
        hint = self.reader_name_hint.lower()
        for _ in range(10):
            try:
                available = readers()
                if available:
                    for r in available:
                        if hint in str(r).lower():
                            return r
                    return available[0]
            except Exception as e:
                self.get_logger().warn(f'reader enumeration failed: {e}')
            time.sleep(0.5)
        return None

    def read_uid_once(self) -> Optional[str]:
        reader = self.select_reader()
        if reader is None:
            return None
        conn = None
        try:
            conn = reader.createConnection()
            conn.connect()
            apdu = [0xFF, 0xCA, 0x00, 0x00, 0x00]
            data, sw1, sw2 = conn.transmit(apdu)
            if sw1 == 0x90 and sw2 == 0x00 and len(data) > 0:
                uid = ''.join(f'{b:02X}' for b in data)
                return uid
            return None
        except (NoCardException, CardConnectionException):
            return None
        except Exception as e:
            self.get_logger().warn(f'UID read failed: {e}')
            return None
        finally:
            if conn is not None:
                try:
                    conn.disconnect()
                except Exception:
                    pass

    # =========================
    # 서버 전송 및 응답 확인 (핵심 변경 구역)
    # =========================
    def send_rfid(self, uid: str):
        with self.lock:
            auth_event_id = self.auth_event_id
            frame = None if self.auth_start_frame is None else self.auth_start_frame.copy()

        if not auth_event_id:
            self.get_logger().warn('send_rfid called but auth_event_id is empty')
            return

        data = {"auth_event_id": auth_event_id, "rfid_uid": uid}
        files = self.build_image_file(frame)

        try:
            # 1. 로봇이 서버로 카드를 전송합니다.
            resp = self.http.post(f'{self.server_base_url}/auth/rfid', data=data, files=files, timeout=self.request_timeout_sec)
            resp.raise_for_status()
            
            # 2. 서버가 즉시 돌려준 대답(JSON)을 확인합니다.
            result = resp.json()
            self.get_logger().info(f'/auth/rfid result: {result}')

            # 3. 서버 응답 내용에서 'status' 값을 뽑아냅니다.
            auth_event = result.get("auth_event", {})
            status = auth_event.get("status", "")

            # 4. 판정 결과에 따라 직관적인 소리를 냅니다.
            if status == "success":
                self.play_sound('/home/chan/capston_h3c_integration/src/rfid/rfid/success.wav')
                self.get_logger().info("✅ 인증 성공! (success.wav 재생)")

                self.request_publish_auth_result(auth_event_id, "success")
                   
            elif status == "fail":
                self.play_sound('/home/chan/capston_h3c_integration/src/rfid/rfid/fail.wav')
                self.get_logger().info("❌ 인증 실패! (fail.wav 재생)")

                self.request_publish_auth_result(auth_event_id, "fail")

        except Exception as e:
            self.get_logger().error(f'/auth/rfid 통신 중 치명적 오류 발생: {e}')
            return
        finally:
            # 작업이 끝났으므로 다음 인증을 위해 로봇의 상태를 깨끗이 청소합니다.
            self.clear_auth_state()

    def send_timeout(self):
        with self.lock:
            auth_event_id = self.auth_event_id
            frame = None if self.auth_start_frame is None else self.auth_start_frame.copy()

        if not auth_event_id:
            self.get_logger().warn('send_timeout called but auth_event_id is empty')
            return

        data = {"auth_event_id": auth_event_id, "timestamp": datetime.now().isoformat()}
        files = self.build_image_file(frame)

        try:
            resp = self.http.post(f'{self.server_base_url}/auth/timeout', data=data, files=files, timeout=self.request_timeout_sec)
            resp.raise_for_status()
            result = resp.json()
            self.get_logger().info(f'/auth/timeout result: {result}')
            
            # 🟢 타임아웃 발생 시에도 어쨌든 실패이므로 알림음을 울립니다.
            self.play_sound('/home/chan/capston_h3c_integration/src/rfid/rfid/fail.wav')

            self.request_publish_auth_result(auth_event_id, "timeout")
            
        except Exception as e:
            self.get_logger().error(f'/auth/timeout failed: {e}')
            return
        finally:
            self.clear_auth_state()

    # =========================
    # 이미지 첨부
    # =========================
    def build_image_file(self, frame):
        if frame is None:
            return None
        ok, enc = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not ok:
            self.get_logger().warn('image encoding failed')
            return None
        return {'image': ('auth.jpg', enc.tobytes(), 'image/jpeg')}

    # =========================
    # 상태 초기화
    # =========================
    def clear_auth_state(self):
        with self.lock:
            self.auth_in_progress = False
            self.auth_event_id = None
            self.auth_start_frame = None
            self.auth_started_mono = 0.0
            self.last_auth_ready = False
        self.get_logger().info('auth session cleared (인증 세션이 완전히 초기화되었습니다.)')


def main(args=None):
    rclpy.init(args=args)
    node = SecondaryAuthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("사용자에 의해 안전하게 노드를 종료합니다.")
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()