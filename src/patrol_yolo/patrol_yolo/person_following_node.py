"""
PersonFollowingNode
/person_tracking/tracks_json (String/JSON) 을 구독하여
1. 주행 파트가 바로 사용할 수 있는 geometry_msgs/PointStamped 와
2. 추적 상태(std_msgs/String) 를 pub

퍼블리시 토픽
  /person_tracking/follow_target  (geometry_msgs/PointStamped)
      frame_id : camera_color_optical_frame
      point.x/y/z : X_cam, Y_cam, Z_cam  (m, 카메라 좌표계)
      → TRACKING 상태 + depth 유효할 때만 발행

  /person_tracking/follow_state   (std_msgs/String)
      값 : IDLE | TRACKING | LOST
      → state_publish_hz로 주기적으로 고정 발행

  /person_tracking/follow_person_id   (std_msgs/String)
      값: 현재 추적 중인 대상의 ByteTrack ID
      타겟 없으면 ""
      인증 이벤트와 tracking 대상 매칭용 식별자

구독 토픽
  /person_tracking/tracks_json   (std_msgs/String, JSON)
  /person_tracking/enable        (std_msgs/Bool)

FSM 전이
  IDLE
    └─ stable 감지 + depth 유효 ─────────────► TRACKING

  TRACKING
    ├─ 타겟 ID가 안 보임
    │    ├─ 위치 기반 recovery 성공 ─────────► TRACKING 유지
    │    └─ recovery 실패 ───────────────────► LOST
    ├─ depth 연속 N프레임 없음 ─────────────► LOST
    └─ YOLO 비활성화 ───────────────────────► IDLE

  LOST
    ├─ 기존 target_id 재발견 + depth 유효 ──► TRACKING
    └─ lost_timeout_sec 초과 ───────────────► IDLE
"""

import json
import time
import threading
from typing import Optional, Dict, List

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool
from geometry_msgs.msg import PointStamped


# FSM 상태 상수
IDLE = "IDLE"
TRACKING = "TRACKING"
LOST = "LOST"


def _bbox_area(bbox_xyxy: List[int]) -> float:
    x1, y1, x2, y2 = bbox_xyxy
    return float(max(0, x2 - x1) * max(0, y2 - y1))


def _has_depth(track: dict) -> bool:
    return (
        "X_cam" in track and
        "Y_cam" in track and
        "Z_cam" in track
    )


class PersonFollowingNode(Node):

    def __init__(self):
        super().__init__("person_following_node")

        # ── 파라미터 선언 ────────────────────────────────────────────────
        self.declare_parameter("tracks_topic", "/person_tracking/tracks_json")
        self.declare_parameter("enable_topic", "/person_tracking/enable")
        self.declare_parameter("target_topic", "/person_tracking/follow_target")
        self.declare_parameter("state_topic", "/person_tracking/follow_state")
        self.declare_parameter("person_id_topic", "/person_tracking/follow_person_id")

        # 대상 선택 전략: "largest_bbox" | "center" | "first_seen"
        self.declare_parameter("target_strategy", "largest_bbox")

        # 픽셀 기준 화면 중앙 (center 전략 전용)
        self.declare_parameter("frame_center_u", 640)

        # 트랙 필터
        self.declare_parameter("min_confidence", 0.5)
        self.declare_parameter("min_bbox_w", 40)
        self.declare_parameter("min_bbox_h", 80)

        # 안정화: N프레임 연속 감지 후 TRACKING 전환
        self.declare_parameter("min_stable_hits", 3)

        # LOST → IDLE 타임아웃
        self.declare_parameter("lost_timeout_sec", 3.0)

        # 타겟 ID 스위칭 방지 debounce
        self.declare_parameter("id_switch_debounce_sec", 0.8)

        # depth 없음 허용 연속 프레임 수. 초과 시 TRACKING → LOST
        self.declare_parameter("max_no_depth_streak", 3)

        # depth 없을 시 잠시 이전 좌표를 재사용
        self.declare_parameter("target_hold_sec", 0.5)

        # ByteTrack ID가 바뀌었을 때 bbox 중심 거리 기반 recovery
        self.declare_parameter("recover_max_dist_px", 150.0)

        # 위치 recovery에 사용할 last_seen 유효 시간
        self.declare_parameter("position_recover_max_age_sec", 1.0)

        # stale frame drop: tracks_json header stamp 기준
        self.declare_parameter("max_frame_age_sec", 0.3)

        # 상태 퍼블리시 주기
        self.declare_parameter("state_publish_hz", 10.0)

        # ── 파라미터 읽기 ────────────────────────────────────────────────
        self.tracks_topic = str(self.get_parameter("tracks_topic").value)
        self.enable_topic = str(self.get_parameter("enable_topic").value)
        self.target_topic = str(self.get_parameter("target_topic").value)
        self.state_topic = str(self.get_parameter("state_topic").value)
        self.person_id_topic = str(self.get_parameter("person_id_topic").value)

        self.target_strategy = str(self.get_parameter("target_strategy").value)
        self.frame_center_u = int(self.get_parameter("frame_center_u").value)

        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.min_bbox_w = int(self.get_parameter("min_bbox_w").value)
        self.min_bbox_h = int(self.get_parameter("min_bbox_h").value)

        self.min_stable_hits = int(self.get_parameter("min_stable_hits").value)
        self.lost_timeout_sec = float(self.get_parameter("lost_timeout_sec").value)
        self.id_switch_debounce_sec = float(
            self.get_parameter("id_switch_debounce_sec").value
        )
        self.max_no_depth_streak = int(
            self.get_parameter("max_no_depth_streak").value
        )
        self.target_hold_sec = float(self.get_parameter("target_hold_sec").value)
        self.recover_max_dist_px = float(
            self.get_parameter("recover_max_dist_px").value
        )
        self.position_recover_max_age_sec = float(
            self.get_parameter("position_recover_max_age_sec").value
        )
        self.max_frame_age_sec = float(self.get_parameter("max_frame_age_sec").value)

        state_hz = float(self.get_parameter("state_publish_hz").value)

        # ── FSM 상태 ─────────────────────────────────────────────────────
        self._lock = threading.Lock()

        self.state: str = IDLE
        self.inference_enabled: bool = True

        self.target_id: Optional[int] = None
        self.lost_start_time: Optional[float] = None
        self.last_id_switch_time: float = 0.0

        # 후보 id → 연속 감지 횟수
        self.candidate_hits: Dict[int, int] = {}

        # 타겟 depth 없음 연속 프레임 카운터
        self.no_depth_streak: int = 0

        # 마지막 유효 3D target. depth hold용.
        self.last_valid_target_track: Optional[dict] = None
        self.last_valid_target_time: float = 0.0

        # 마지막으로 화면에서 본 target. ID recovery용.
        self.last_seen_target_track: Optional[dict] = None
        self.last_seen_target_time: float = 0.0

        # ── ROS 통신 ─────────────────────────────────────────────────────
        self.tracks_sub = self.create_subscription(
            String,
            self.tracks_topic,
            self._tracks_callback,
            10,
        )

        self.enable_sub = self.create_subscription(
            Bool,
            self.enable_topic,
            self._enable_callback,
            10,
        )

        self.target_pub = self.create_publisher(
            PointStamped,
            self.target_topic,
            10,
        )

        self.state_pub = self.create_publisher(
            String,
            self.state_topic,
            10,
        )

        self.person_id_pub = self.create_publisher(
            String,
            self.person_id_topic,
            10,
        )

        self.state_timer = self.create_timer(
            1.0 / state_hz,
            self._publish_state,
        )

        self.get_logger().info(
            f"[INIT] PersonFollowingNode | "
            f"strategy={self.target_strategy} | "
            f"min_conf={self.min_confidence} | "
            f"min_bbox=({self.min_bbox_w},{self.min_bbox_h}) | "
            f"min_stable_hits={self.min_stable_hits} | "
            f"lost_timeout={self.lost_timeout_sec} | "
            f"recover_max_dist_px={self.recover_max_dist_px} | "
            f"position_recover_max_age_sec={self.position_recover_max_age_sec} | "
            f"max_no_depth_streak={self.max_no_depth_streak} | "
            f"max_frame_age_sec={self.max_frame_age_sec} | "
            f"tracks={self.tracks_topic} | "
            f"target={self.target_topic} | "
            f"state={self.state_topic}"
        )

    # ────────────────────────────────────────────────────────────────────
    # SUB callbacks

    def _enable_callback(self, msg: Bool):
        with self._lock:
            prev = self.inference_enabled
            self.inference_enabled = bool(msg.data)

        if prev != self.inference_enabled:
            self.get_logger().info(
                f"[CONTROL] inference_enabled → {self.inference_enabled}"
            )
            if not self.inference_enabled:
                self._reset_to_idle("yolo_disabled")

    def _tracks_callback(self, msg: String):
        """
        JSON 파싱 + FSM 업데이트
        """
        try:
            payload = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"[JSON] parse failed: {e}")
            return

        with self._lock:
            enabled = self.inference_enabled

        if not enabled:
            return

        tracks = payload.get("tracks", [])
        header_dict = payload.get("header", {})
        now = time.time()

        stamp_sec = int(header_dict.get("stamp_sec", 0))
        stamp_nsec = int(header_dict.get("stamp_nanosec", 0))
        frame_time = stamp_sec + stamp_nsec * 1e-9
        age_sec = now - frame_time

        if frame_time > 0 and age_sec > self.max_frame_age_sec:
            self.get_logger().debug(
                f"[STALE] frame dropped age={age_sec:.3f}s > {self.max_frame_age_sec}s"
            )
            return

        valid = self._filter_tracks(tracks)

        if not valid:
            self._handle_no_detection(now)
            return

        self._update_candidate_hits(valid)
        chosen = self._select_target(valid)

        if chosen is None:
            self._handle_no_detection(now)
            return

        chosen_id = int(chosen.get("person_id", -1))
        chosen_has_depth = _has_depth(chosen)

        # ── FSM 진입 / 복귀 / 스위칭 ────────────────────────────────────
        with self._lock:
            stable = self.candidate_hits.get(chosen_id, 0) >= self.min_stable_hits
            can_switch = self._can_switch_id_locked(chosen_id, now)

            # LOST 상태에서 기존 target_id가 다시 보이고 depth가 있으면 즉시 복귀
            if (
                self.state == LOST
                and self.target_id == chosen_id
                and chosen_has_depth
            ):
                self.state = TRACKING
                self.lost_start_time = None
                self.no_depth_streak = 0
                self.candidate_hits[chosen_id] = max(
                    self.candidate_hits.get(chosen_id, 0),
                    self.min_stable_hits,
                )
                self.get_logger().info(
                    f"[FSM] LOST → TRACKING (reacquired id={chosen_id})"
                )

            # IDLE 또는 새 타겟 진입/전환
            elif stable and can_switch and chosen_has_depth:
                if self.target_id != chosen_id:
                    self.get_logger().info(
                        f"[FSM] target id: {self.target_id} → {chosen_id}"
                    )
                    self.target_id = chosen_id
                    self.last_id_switch_time = now
                    self.no_depth_streak = 0

                self.state = TRACKING
                self.lost_start_time = None

            current_state = self.state
            current_target = self.target_id

        # TRACKING 상태가 아니면 여기서 종료
        if current_state != TRACKING or current_target is None:
            return

        # ── 현재 target_id 매칭 ─────────────────────────────────────────
        matched = [
            t for t in valid
            if int(t.get("person_id", -1)) == current_target
        ]

        if not matched:
            recovered = self._find_by_position(
                valid,
                self.recover_max_dist_px,
            )

            if recovered is not None:
                recovered_id = int(recovered.get("person_id", -1))

                with self._lock:
                    old_id = self.target_id
                    self.target_id = recovered_id
                    self.last_id_switch_time = now
                    self.state = TRACKING
                    self.lost_start_time = None
                    self.no_depth_streak = 0
                    self.last_seen_target_track = dict(recovered)
                    self.last_seen_target_time = now
                    self.candidate_hits[recovered_id] = self.min_stable_hits

                self.get_logger().info(
                    f"[RECOVER] target id recovered by position: {old_id} → {recovered_id}"
                )

                target_track = recovered

            else:
                with self._lock:
                    self.state = LOST
                    self.lost_start_time = now
                    self.no_depth_streak = 0

                self.get_logger().info(
                    "[FSM] TRACKING → LOST (target missing in frame)"
                )
                return

        else:
            target_track = matched[0]

        has_depth = _has_depth(target_track)

        # 화면에서 본 마지막 타겟은 depth 여부와 관계없이 저장
        with self._lock:
            self.last_seen_target_track = dict(target_track)
            self.last_seen_target_time = now

        if has_depth:
            with self._lock:
                self.no_depth_streak = 0
                self.last_valid_target_track = {
                    "X_cam": float(target_track["X_cam"]),
                    "Y_cam": float(target_track["Y_cam"]),
                    "Z_cam": float(target_track["Z_cam"]),
                    "u": int(target_track.get("u", 0)),
                    "v": int(target_track.get("v", 0)),
                    "person_id": int(target_track.get("person_id", -1)),
                    "bbox_xyxy": target_track.get("bbox_xyxy"),
                    "confidence": float(target_track.get("confidence", 0.0)),
                }
                self.last_valid_target_time = now

            self._publish_target(target_track)

        else:
            with self._lock:
                self.no_depth_streak += 1
                streak = self.no_depth_streak
                max_streak = self.max_no_depth_streak
                last_track = self.last_valid_target_track
                last_time = self.last_valid_target_time

            hold_age = now - last_time

            if last_track is not None and hold_age <= self.target_hold_sec:
                self.get_logger().debug(
                    f"[DEPTH] missing, reuse last target "
                    f"age={hold_age:.2f}s streak={streak}/{max_streak}"
                )
                self._publish_target(last_track)
                return

            self.get_logger().debug(
                f"[DEPTH] no depth streak={streak}/{max_streak} "
                f"target_id={current_target}"
            )

            if streak >= max_streak:
                with self._lock:
                    self.state = LOST
                    self.lost_start_time = now
                    self.no_depth_streak = 0
                    self.last_valid_target_track = None
                    self.last_valid_target_time = 0.0
                    self.last_seen_target_track = None
                    self.last_seen_target_time = 0.0

                self.get_logger().info(
                    f"[FSM] TRACKING → LOST "
                    f"(no depth for {max_streak} consecutive frames)"
                )

    # ────────────────────────────────────────────────────────────────────
    # FSM helpers

    def _handle_no_detection(self, now: float):
        """valid 트랙이 없을 때 FSM 처리."""
        with self._lock:
            state = self.state

        if state == TRACKING:
            with self._lock:
                self.state = LOST
                self.lost_start_time = now
                self.no_depth_streak = 0
            self.get_logger().info("[FSM] TRACKING → LOST (no valid detection)")

        elif state == LOST:
            with self._lock:
                t0 = self.lost_start_time

            if t0 is not None and (now - t0) > self.lost_timeout_sec:
                self._reset_to_idle("lost_timeout")
                return

        # IDLE일 때만 후보 히스토리 제거
        with self._lock:
            if self.state == IDLE:
                self.candidate_hits.clear()

    def _reset_to_idle(self, reason: str):
        with self._lock:
            self.state = IDLE
            self.target_id = None
            self.lost_start_time = None
            self.no_depth_streak = 0
            self.candidate_hits.clear()
            self.last_valid_target_track = None
            self.last_valid_target_time = 0.0
            self.last_seen_target_track = None
            self.last_seen_target_time = 0.0
        self.get_logger().info(f"[FSM] → IDLE reason={reason}")

    def _can_switch_id_locked(self, new_id: int, now: float) -> bool:
        """ID 스위칭 debounce 판별. _lock 보유 상태에서 호출."""
        if self.target_id is None or self.target_id == new_id:
            return True
        return (now - self.last_id_switch_time) >= self.id_switch_debounce_sec

    def _update_candidate_hits(self, valid: list):
        """
        이번 프레임에 보인 id +1, 안 보인 id 제거.
        """
        seen_ids = {int(t.get("person_id", -1)) for t in valid}
        with self._lock:
            new_hits: Dict[int, int] = {}
            for pid in seen_ids:
                if pid < 0:
                    continue
                new_hits[pid] = self.candidate_hits.get(pid, 0) + 1
            self.candidate_hits = new_hits

    # ────────────────────────────────────────────────────────────────────
    # raw값 필터링 + 타겟 선정

    def _filter_tracks(self, tracks: list) -> list:
        """신뢰도, bbox 크기 기준으로 유효 트랙만 반환."""
        result = []

        for t in tracks:
            if float(t.get("confidence", 0.0)) < self.min_confidence:
                continue

            if int(t.get("person_id", -1)) == -1:
                continue

            bbox = t.get("bbox_xyxy")
            if not isinstance(bbox, list) or len(bbox) != 4:
                continue

            x1, y1, x2, y2 = bbox
            if (x2 - x1) < self.min_bbox_w or (y2 - y1) < self.min_bbox_h:
                continue

            result.append(t)

        return result

    def _find_by_position(
        self,
        valid: list,
        max_dist_px: float = 150.0,
    ) -> Optional[dict]:
        """
        ByteTrack ID가 바뀌었더라도 이전 target 위치 근처의 bbox를 같은 사람으로 간주.
        제어 안전성을 위해 depth 있는 후보만 recovery 대상으로 사용.
        """
        with self._lock:
            last = self.last_seen_target_track
            last_time = self.last_seen_target_time

        if last is None:
            return None

        now = time.time()
        if (now - last_time) > self.position_recover_max_age_sec:
            return None

        last_u = last.get("u")
        last_v = last.get("v")
        if last_u is None or last_v is None:
            return None

        best = None
        best_dist = float("inf")

        for t in valid:
            if "u" not in t or "v" not in t:
                continue

            # follow_target 발행 안정성을 위해 depth 없는 후보는 recovery하지 않음
            if not _has_depth(t):
                continue

            du = float(t["u"]) - float(last_u)
            dv = float(t["v"]) - float(last_v)
            dist = (du * du + dv * dv) ** 0.5

            if dist < best_dist:
                best = t
                best_dist = dist

        if best is not None and best_dist <= max_dist_px:
            return best

        return None

    def _select_target(self, valid: list) -> Optional[dict]:
        """
        타겟 선택 우선순위:
          1. 현재 target_id가 살아있으면 그대로 유지
          2. 없으면 target_strategy 기준으로 새 타겟 선택

        전략별 동작:
          largest_bbox : bbox 면적 최대
          center       : 화면 중앙에 픽셀 거리 최소
          first_seen   : valid 리스트 첫 번째
        """
        if not valid:
            return None

        with self._lock:
            cur = self.target_id

        if cur is not None:
            existing = [
                t for t in valid
                if int(t.get("person_id", -1)) == cur
            ]
            if existing:
                return existing[0]

        if self.target_strategy == "largest_bbox":
            return max(valid, key=lambda t: _bbox_area(t["bbox_xyxy"]))

        if self.target_strategy == "center":
            return min(
                valid,
                key=lambda t: abs(int(t.get("u", 0)) - self.frame_center_u),
            )

        return valid[0]

    # ────────────────────────────────────────────────────────────────────
    # PUB

    def _publish_target(self, track: dict):
        """
        geometry_msgs/PointStamped 퍼블리시.
        X_cam/Y_cam/Z_cam 존재해야 함.
        """
        if not _has_depth(track):
            return

        pt = PointStamped()
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.header.frame_id = "camera_color_optical_frame"
        pt.point.x = float(track["X_cam"])
        pt.point.y = float(track["Y_cam"])
        pt.point.z = float(track["Z_cam"])
        self.target_pub.publish(pt)

    def _publish_state(self):
        """state_publish_hz 주기로 현재 FSM 상태 퍼블리시."""
        with self._lock:
            current_state = self.state
            current_target = self.target_id

        state_msg = String()
        state_msg.data = current_state
        self.state_pub.publish(state_msg)

        pid_msg = String()
        pid_msg.data = "" if current_target is None else str(current_target)
        self.person_id_pub.publish(pid_msg)

    # ────────────────────────────────────────────────────────────────────

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PersonFollowingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()