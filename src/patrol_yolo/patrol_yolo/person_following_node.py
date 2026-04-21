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
    값: 현재 추적 중인 대상의 ByteTrack ID (예: "7")
    타겟 없으면 ""
    인증 이벤트와 tracking 대상 매칭용 식별자
    state_publish_hz 주기로 follow_state와 함께 발행

구독 토픽
  /person_tracking/tracks_json   (std_msgs/String, JSON)
  /person_tracking/enable        (std_msgs/Bool)

FSM 전이
  IDLE
    └─ stable 감지 + depth 유효 (동시 충족) >  TRACKING
  TRACKING
    ├─ 타겟 소실 (프레임 내 미검출) >  LOST
    ├─ depth 연속 N프레임 없음(멀어짐) > LOST
    └─ YOLO 비활성화 > IDLE
  LOST
    ├─ 타겟 재발견 + depth 유효 ──────────────► TRACKING
    └─ lost_timeout_sec 초과 ────────────────► IDLE


타겟 선택 전략 (target_strategy 파라미터)
  largest_bbox  : bbox 면적 최대 = 가장 가까운(크게 보이는) 사람 

  확정 후에는 해당 ID가 사라지기 전까지 교체하지 않음
  ID가 사라지면 LOST → 타임아웃 후 IDLE에서 재선택
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
IDLE     = "IDLE"
TRACKING = "TRACKING"
LOST     = "LOST"


def _bbox_area(bbox_xyxy: List[int]) -> float:
    x1, y1, x2, y2 = bbox_xyxy
    return float(max(0, x2 - x1) * max(0, y2 - y1))


class PersonFollowingNode(Node):

    def __init__(self):
        super().__init__("person_following_node")

        # ── 파라미터 선언 ────────────────────────────────────────────────
        self.declare_parameter("tracks_topic",            "/person_tracking/tracks_json")
        self.declare_parameter("enable_topic",            "/person_tracking/enable")
        self.declare_parameter("target_topic",            "/person_tracking/follow_target")
        self.declare_parameter("state_topic",             "/person_tracking/follow_state")
        self.declare_parameter("person_id_topic", "/person_tracking/follow_person_id")

        # 대상 선택 전략: "largest_bbox" | "center" | "first_seen"
        self.declare_parameter("target_strategy",         "largest_bbox")

        # 픽셀 기준 화면 중앙 (center 전략 전용)
        self.declare_parameter("frame_center_u",          640) # 1280x720 기준

        # 트랙 필터
        self.declare_parameter("min_confidence",          0.5)
        self.declare_parameter("min_bbox_w",              40)
        self.declare_parameter("min_bbox_h",              80)

        # 안정화: N프레임 연속 감지 후 TRACKING 전환
        self.declare_parameter("min_stable_hits",         3)

        # LOST → IDLE 타임아웃
        self.declare_parameter("lost_timeout_sec",        3.0)

        # 타겟 ID 스위칭 방지 debounce
        self.declare_parameter("id_switch_debounce_sec",  0.8)

        # depth 없음 허용 연속 프레임 수. 초과 시 TRACKING → LOST
        self.declare_parameter("max_no_depth_streak",     3)

        # stale frame drop: tracks_json header stamp 기준
        # 현재 시각과의 차이가 이 값을 초과하면 해당 프레임을 무시
        self.declare_parameter("max_frame_age_sec",       0.3)

        # 상태 퍼블리시 주기
        self.declare_parameter("state_publish_hz",        10.0)

        # ── 파라미터 읽기 ────────────────────────────────────────────────
        self.tracks_topic           = str(self.get_parameter("tracks_topic").value)
        self.enable_topic           = str(self.get_parameter("enable_topic").value)
        self.target_topic           = str(self.get_parameter("target_topic").value)
        self.state_topic            = str(self.get_parameter("state_topic").value)
        self.person_id_topic = str(self.get_parameter("person_id_topic").value)

        self.target_strategy        = str(self.get_parameter("target_strategy").value)
        self.frame_center_u         = int(self.get_parameter("frame_center_u").value)

        self.min_confidence         = float(self.get_parameter("min_confidence").value)
        self.min_bbox_w             = int(self.get_parameter("min_bbox_w").value)
        self.min_bbox_h             = int(self.get_parameter("min_bbox_h").value)

        self.min_stable_hits        = int(self.get_parameter("min_stable_hits").value)
        self.lost_timeout_sec       = float(self.get_parameter("lost_timeout_sec").value)
        self.id_switch_debounce_sec = float(self.get_parameter("id_switch_debounce_sec").value)
        self.max_no_depth_streak    = int(self.get_parameter("max_no_depth_streak").value)
        self.max_frame_age_sec      = float(self.get_parameter("max_frame_age_sec").value)

        state_hz                    = float(self.get_parameter("state_publish_hz").value)

        # ── FSM 상태 ─────────────────────────────────────────────────────
        self._lock = threading.Lock()

        self.state:            str            = IDLE
        self.inference_enabled: bool          = True

        self.target_id:          Optional[int]   = None
        self.lost_start_time:    Optional[float] = None
        self.last_id_switch_time: float          = 0.0

        # 후보 id → 연속 감지 횟수 (안정화용)  ← _lock 으로 보호
        self.candidate_hits: Dict[int, int] = {}

        # 타겟의 depth 없음 연속 프레임 카운터  ← _lock 으로 보호
        self.no_depth_streak: int = 0

        # ── ROS 통신 ─────────────────────────────────────────────────────
        self.tracks_sub = self.create_subscription(
            String, self.tracks_topic, self._tracks_callback, 10)

        self.enable_sub = self.create_subscription(
            Bool, self.enable_topic, self._enable_callback, 10)

        self.target_pub = self.create_publisher(
            PointStamped, self.target_topic, 10)

        self.state_pub = self.create_publisher(
            String, self.state_topic, 10)
        
        self.person_id_pub = self.create_publisher(
            String, self.person_id_topic, 10
        )

        self.state_timer = self.create_timer(
            1.0 / state_hz, self._publish_state)

        self.get_logger().info(
            f"[INIT] PersonFollowingNode | "
            f"strategy={self.target_strategy} | "
            f"max_no_depth_streak={self.max_no_depth_streak} | "
            f"max_frame_age_sec={self.max_frame_age_sec} | "
            f"tracks={self.tracks_topic} | "
            f"target={self.target_topic} | "
            f"state={self.state_topic}"
        )

    # ────────────────────────────────────────────────────────────────────
    # SUB callback

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

        tracks      = payload.get("tracks", [])
        header_dict = payload.get("header", {})
        now         = time.time()

        stamp_sec  = int(header_dict.get("stamp_sec",  0))
        stamp_nsec = int(header_dict.get("stamp_nanosec", 0))
        frame_time = stamp_sec + stamp_nsec * 1e-9
        age_sec    = now - frame_time

        if frame_time > 0 and age_sec > self.max_frame_age_sec:
            self.get_logger().debug(
                f"[STALE] frame dropped  age={age_sec:.3f}s > {self.max_frame_age_sec}s"
            )
            return

        valid = self._filter_tracks(tracks)

        if not valid:
            self._handle_no_detection(now)
            return

        # ── 후보 hits 갱신 (lock 보호) ───────────────────────────────────
        self._update_candidate_hits(valid)
        chosen = self._select_target(valid)

        if chosen is None:
            self._handle_no_detection(now)
            return

        chosen_id = int(chosen.get("person_id", -1))
        chosen_has_depth = ("X_cam" in chosen and
                            "Y_cam" in chosen and
                            "Z_cam" in chosen)

        # TRACKING 진입 조건: stable + depth valid 동시 충족 ──
        with self._lock:
            stable     = self.candidate_hits.get(chosen_id, 0) >= self.min_stable_hits
            can_switch = self._can_switch_id_locked(chosen_id, now)

            if stable and can_switch and chosen_has_depth:
                if self.target_id != chosen_id:
                    self.get_logger().info(
                        f"[FSM] target id: {self.target_id} → {chosen_id}"
                    )
                    self.target_id           = chosen_id
                    self.last_id_switch_time = now
                    self.no_depth_streak     = 0  

                self.state           = TRACKING
                self.lost_start_time = None

            current_state  = self.state
            current_target = self.target_id

        # ── TRACKING 상태: 타겟 depth 확인 후 퍼블리시 ──────────────────
        if current_state != TRACKING or current_target is None:
            return

        matched = [t for t in valid if int(t.get("person_id", -1)) == current_target]

        if not matched:
            # 타겟이 이 프레임에서 사라짐 → LOST
            with self._lock:
                self.state           = LOST
                self.lost_start_time = now
                self.no_depth_streak = 0   # ← lock 안에서 리셋
            self.get_logger().info("[FSM] TRACKING → LOST (target missing in frame)")
            return

        target_track = matched[0]
        has_depth    = ("X_cam" in target_track and
                        "Y_cam" in target_track and
                        "Z_cam" in target_track)

        if has_depth:
            # depth 정상 → streak 리셋 후 퍼블리시
            with self._lock:
                self.no_depth_streak = 0   # ← lock 안에서 리셋
            self._publish_target(target_track)

        else:
            # depth 없음 → streak 증가 (lock 보호)
            with self._lock:
                self.no_depth_streak += 1
                streak = self.no_depth_streak
                max_streak = self.max_no_depth_streak

            self.get_logger().debug(
                f"[DEPTH] no depth streak={streak}/{max_streak} "
                f"target_id={current_target}"
            )

            if streak >= max_streak:
                # 연속 N프레임 depth 없음 → LOST
                with self._lock:
                    self.state           = LOST
                    self.lost_start_time = now
                    self.no_depth_streak = 0
                self.get_logger().info(
                    f"[FSM] TRACKING → LOST "
                    f"(no depth for {max_streak} consecutive frames)"
                )

    # ────────────────────────────────────────────────────────────────────
    # FSM 헬퍼

    def _handle_no_detection(self, now: float):
        """valid 트랙이 없을 때 FSM 처리."""
        with self._lock:
            state = self.state

        if state == TRACKING:
            with self._lock:
                self.state           = LOST
                self.lost_start_time = now
                self.no_depth_streak = 0  
            self.get_logger().info("[FSM] TRACKING → LOST (no valid detection)")

        elif state == LOST:
            with self._lock:
                t0 = self.lost_start_time
            if t0 is not None and (now - t0) > self.lost_timeout_sec:
                self._reset_to_idle("lost_timeout")

        with self._lock:
            self.candidate_hits.clear()   

    def _reset_to_idle(self, reason: str):
        with self._lock:
            self.state           = IDLE
            self.target_id       = None
            self.lost_start_time = None
            self.no_depth_streak = 0       
            self.candidate_hits.clear()    
        self.get_logger().info(f"[FSM] → IDLE  reason={reason}")

    def _can_switch_id_locked(self, new_id: int, now: float) -> bool:
        """ID 스위칭 debounce 판별. _lock 보유 상태에서 호출."""
        if self.target_id is None or self.target_id == new_id:
            return True
        return (now - self.last_id_switch_time) >= self.id_switch_debounce_sec

    def _update_candidate_hits(self, valid: list):
        """이번 프레임에 보인 id +1, 안 보인 id 제거. _lock 으로 보호."""
        seen_ids = {int(t.get("person_id", -1)) for t in valid}
        with self._lock:
            new_hits: Dict[int, int] = {}
            for pid in seen_ids:
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

    def _select_target(self, valid: list) -> Optional[dict]:
        """
        타겟 선택 우선순위:
          1. 현재 target_id가 살아있으면 그대로 유지 (교체 안 함)
          2. 없으면 target_strategy 기준으로 새 타겟 선택

        전략별 동작:
          largest_bbox : bbox 면적 최대 → 카메라에 가장 크게 잡힌(가까운) 사람
                         여러 명일 때 제일 가까운 사람을 follow
          center       : 화면 중앙에 픽셀 거리 최소 → 정면에 있는 사람
        """
        if not valid:
            return None

        # 기존 타겟 유지 우선
        with self._lock:
            cur = self.target_id

        if cur is not None:
            existing = [t for t in valid if int(t.get("person_id", -1)) == cur]
            if existing:
                return existing[0]

        # 새 타겟 선택
        if self.target_strategy == "largest_bbox":
            return max(valid, key=lambda t: _bbox_area(t["bbox_xyxy"]))
        elif self.target_strategy == "center":
            return min(valid, key=lambda t: abs(int(t.get("u", 0)) - self.frame_center_u))
        else:  # first_seen
            return valid[0]

    # ────────────────────────────────────────────────────────────────────
    # PUB

    def _publish_target(self, track: dict):
        """
        geometry_msgs/PointStamped 퍼블리시.
        has_depth 확인 후 호출되므로 X_cam/Y_cam/Z_cam 존재 보장.
        """
        pt = PointStamped()
        pt.header.stamp    = self.get_clock().now().to_msg()
        pt.header.frame_id = "camera_color_optical_frame"
        pt.point.x         = float(track["X_cam"])
        pt.point.y         = float(track["Y_cam"])
        pt.point.z         = float(track["Z_cam"])
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