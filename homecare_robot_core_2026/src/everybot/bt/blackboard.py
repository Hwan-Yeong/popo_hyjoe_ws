"""
RobotBlackboard — BT 노드 간 공유 상태 컨테이너.

Bridge 가 매 tick 갱신하고, 노드들은 읽기/쓰기(ctx 등)로 사용한다.
py_trees 내장 Blackboard 대신 단순 dataclass 를 사용해
타입 힌팅과 IDE 지원을 최대화한다.
"""
from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class ScheduleEntry:
    """스케줄 기반 자동 시나리오 엔트리."""

    scenario_id: str = ""
    enabled: bool = True
    trigger_time: str = ""
    operating_start: str = ""
    operating_end: str = ""
    zone_id: str = ""
    days: list[str] = field(
        default_factory=lambda: ["mon", "tue", "wed", "thu", "fri", "sat", "sun"]
    )


@dataclass
class RobotBlackboard:
    """
    서비스 상태와 시나리오 컨텍스트를 보관하는 공유 데이터 구조.

    Bridge.update() 가 매 tick 아래 필드를 최신 상태로 갱신한다.
    BT 노드는 이 객체를 참조(reference)로 받아 직접 읽는다.
    """

    # ── AMR 상태 (Bridge 가 매 tick 갱신) ───────────────────────
    amr_moving_state: int   = 0          # MovingState.IDLE(0) or MOVING(1)
    amr_robot_state: int = 0
    amr_action_state: int = 0
    amr_position:     dict  = field(default_factory=dict)
    amr_arrived:      bool  = False      # pop_arrived_event() 결과 캐시
    amr_arrived_state: int | None = None # 도착 이벤트 발생 시점의 movingState
    battery_percent:  float = 100.0

    # ── AI 이벤트 (매 tick drain → 교체) ──────────────────────
    ai_events: list[dict] = field(default_factory=list)

    # ── 시나리오 제어 ──────────────────────────────────────────
    active_scenario:  str  | None = None
    scenario_params:  dict        = field(default_factory=dict)

    # ── step 간 공유 컨텍스트 ──────────────────────────────────
    ctx: dict = field(default_factory=dict)
    # 예: ctx["detected_person"] = {"name": "홍길동", "unit": "101"}
    #     ctx["photo_path"]      = "/tmp/photo.jpg"
    #     ctx["recognized_name"] = "홍길동"

    # ── 글로벌 플래그 ────────────────────────────────────────────
    initialized:    bool = False   # Init Sequence 완료 여부 (부팅 후 1회)
    emergency_stop: bool = False   # UI 긴급정지 버튼 활성 상태
    move_stop_requested: bool = False  # UI 이동 정지 요청(robot/cmd/stop)
    idle_status:    bool = False   # Idle 진입 상태

    # ── Init 상태 (v2) ───────────────────────────────────────────
    map_ready:         bool = False  # 맵 파일 존재 확인 결과 (부팅 시 wiring에서 설정)
    wifi_registered:   bool = False  # WiFi 자동연결 프로파일 존재 여부 (state.json)

    # ── 맵 생성 (v2) ─────────────────────────────────────────────
    map_creation_done: bool = False  # MapCreationServer /api/done 수신 시 True
    latest_map_data:   dict = field(default_factory=dict)  # AMR MapData 수신 시 임시 저장

    # ── 공장초기화 (v2) ──────────────────────────────────────────
    factory_reset:     bool = False  # UI request_factory_reset 수신 시 True

    # ── Force SoftAP (v2) ─────────────────────────────────────
    force_softap:      bool = False  # cmd_force_softap 수신 시 True, 해제 시 False

    # ── Map Edit (v2) ────────────────────────────────────────
    map_edit:          bool = False  # cmd_map_edit enabled=true/false → 맵서버 편집 모드

    # ── 설정 변경 (v2) — MainService에서 처리 후 초기화 ──────────
    settings_changed:  bool = False  # MQTT settings/change_value 수신 시 True
    settings_pending:  dict = field(default_factory=dict)  # 수신된 변경값 델타

    # ── 상태 게시 (v2.1) ────────────────────────────────────────
    current_event_code: str = "Normal"

    # ── 스케줄 기반 시나리오 (v2.2) ─────────────────────────────
    schedule_table: list[ScheduleEntry] = field(default_factory=list)
    schedule_trigger_history: set[str] = field(default_factory=set)
    emergency_sensor_triggered: bool = False

    # ── 긴급상황감지 — 낙상/배회 (5/21 demo) ───────────────────
    fall_detected: bool = False
    fall_candidate: bool = False
    fall_status: str = ""
    fall_confidence: float = 0.0
    fall_image_path: str = ""
    wander_detected: bool = False
    wander_person_id: str = ""
    wander_count: int = 0
    wander_image_path: str = ""
    detection_type: str = ""

    # ── 순찰 상태 보존 (이동/대기 중 감지 복귀용) ────────────────
    patrol_waypoints: list = field(default_factory=list)
    patrol_current_idx: int = 0
    patrol_dwell_sec: int = 30
    patrol_interrupted: bool = False
    saved_nav_target: dict = field(default_factory=dict)
    saved_dwell_remaining: float = 0.0

    # ── 모닝콜 컨텍스트 ─────────────────────────────────────────
    morning_call_visits: list = field(default_factory=list)
    current_visit_index: int = 0
    door_retry_count: int = 0
    door_opened: bool = False
    morning_call_active: bool = False
    morning_call_detected_person: str = ""

    # ── ROI 기반 현재 위치 ─────────────────────────────────────
    current_location_id: str = ""
    current_location_name: str = ""

    # ── 운영시간 / 상태 발행 ───────────────────────────────────
    operating_hours: dict = field(default_factory=dict)
    last_status_publish_time: float = 0.0
    last_location_publish_time: float = 0.0
    last_config_publish_time: float = 0.0

    # ── 음성 에이전트 연동 (v2.2) ───────────────────────────────
    voice_agent_intent: str = ""
    voice_agent_response: str = ""
    voice_agent_action: dict = field(default_factory=dict)

    # ── 헬퍼 ────────────────────────────────────────────────────

    def has_ai_event(self, t: str) -> bool:
        """ai_events 에 특정 type 이벤트가 있는지 확인."""
        return any(e.get("type") == t for e in self.ai_events)

    def get_ai_event(self, t: str) -> dict | None:
        """ai_events 에서 특정 type 이벤트 첫 번째 반환. 없으면 None."""
        return next((e for e in self.ai_events if e.get("type") == t), None)

    def clear_ctx(self) -> None:
        """시나리오 컨텍스트 초기화."""
        self.ctx.clear()
