"""
서비스 인터페이스 정의 (Protocol / structural subtyping).

Mock 서비스와 Real 서비스가 모두 이 프로토콜을 만족하도록 구현한다.
BT 노드는 이 인터페이스만 의존하므로 런타임에 Mock ↔ Real 교체가 가능하다.
"""
from __future__ import annotations

from typing import Protocol, runtime_checkable


@runtime_checkable
class AmrServiceProtocol(Protocol):
    """AMR 이동 제어 및 상태 조회 인터페이스."""

    @property
    def cached_moving_state(self) -> int:
        """현재 이동 상태. MovingState.IDLE(0) or MovingState.MOVING(1)."""
        ...

    @property
    def cached_position(self) -> dict:
        """현재 위치 스냅샷. {"x": float, "y": float, "theta": float}"""
        ...

    def pop_arrived_event(self) -> bool:
        """
        엣지 트리거: MOVING → IDLE 전환 시 1회만 True 반환.
        Bridge.update() 에서 호출하므로 BT 노드는 bb.amr_arrived 를 읽는다.
        """
        ...

    @property
    def cached_valid_target_position(self) -> bool:
        """AllMovingInfo.validTargetPosition — 목적지 주행 명령 활성 여부."""
        ...

    def send_target_position(self, coord: dict) -> None:
        """
        cmd=60 목적지 설정만 전송 (cmd=61 없음, type 필드 없음).
        Phase 2 nav_test 전용. send_nav_cmd 와 달리 즉시 주행을 시작하지 않는다.
        """
        ...

    def send_nav_cmd(self, coord: dict) -> None:
        """
        목적지 이동 명령 전송.
        coord: {"x": float, "y": float, "theta": float}
        AMR 실기 기준 cmd=60(TargetPosition)만 전송한다.
        cmd=61(START)를 즉시 연속 전송하면 주행이 시작되지 않는 경우가 있다.
        """
        ...

    def send_stop(self) -> None:
        """AMR 즉시 정지. cmd=61, DrivingSet.STOP."""
        ...

    def send_raw_cmd(self, cmd: int, dtype: int, args: dict) -> None:
        """저수준 직접 전송. 테스트 / 특수 용도."""
        ...

    @property
    def battery_percent(self) -> float:
        """배터리 잔량 (%). BatteryStatus 수신 시 갱신."""
        ...

    def send_manual_vw(self, ms: float, rads: float) -> None:
        """
        cmd=59 수동 조종.
        ms   : 선속도 (m/s)
        rads : 각속도 (rad/s)
        MapCreationServer 키보드 조종 시 사용.
        """
        ...

    def send_mapping_start(self, manual: bool = True) -> None:
        """
        cmd=62 맵 작성 시작.
        manual=True  → set=1 (수동)
        manual=False → set=2 (자동)
        """
        ...

    def send_mapping_stop(self) -> None:
        """cmd=62 맵 작성 정지 (set=4)."""
        ...

    def send_save_map(self) -> None:
        """cmd=87 맵 저장 명령."""
        ...

    def request_map_data(self) -> None:
        """
        cmd=15 맵 데이터 요청.
        응답은 수신 루프에서 latest_map_data 에 캐시된다.
        """
        ...

    @property
    def latest_map_data(self) -> "dict | None":
        """
        최신 MapData 캐시.
        AMR 으로부터 cmd=15 응답 수신 시 갱신.
        ActionSaveMap / BlackboardBridge 가 읽는다.
        """
        ...

    def pop_drive_failed(self) -> bool:
        """
        주행 불가 응답(Response.Set.Driving.result=false) 수신 시 1회만 True 반환.
        ActionNavigateTo 재시도 로직에서 사용.
        """
        ...

    @property
    def cached_robot_status(self) -> int:
        """마지막 수신된 RobotStatus.status (0=IDLE, 1=MOVING, 7=충전스테이션 위치)."""
        ...

    def send_driving_start(self) -> None:
        """cmd=61 Driving set=1 (주행 시작). send_nav_cmd 없이 주행만 별도로 시작할 때 사용."""
        ...

    def send_software_reset(self) -> None:
        """AMR 소프트웨어 리셋. 맵 저장 후 AMR 재초기화 시 사용."""
        ...
       
    def request_moving_info(self) -> None:
        """AMR AllMovingInfo 요청."""
        ...
        
    def send_return_charging_station(self) -> None:
        """AMR 충전 스테이션 복귀 명령."""
        ...
        
    def send_station_repositioning(self,  x: float =0.0, y: float =0.0, theta: float =0.0) -> None:
        """AMR 충전 스테이션 위치 재설정."""
        ...

    def send_bypass(
        self,
        block_areas: list[dict] | None = None,
        block_walls: list[dict] | None = None,
        charging_station: dict | None = None,
    ) -> None:
        """AMR ByPass 설정 전송."""
        ...

    def send_rotation(self, rot_type: int, radian: float) -> None:
        """AMR 회전 명령 전송."""
        ...

    def is_in_zone(self, key: str, zone_mgr: "Any") -> bool:
        """현재 위치가 특정 zone 내부인지 판단."""
        ...

    def estimate_object_world_pos(self, distance_m: float, angle_deg: float = 0.0) -> dict:
        """현재 로봇 pose 기준으로 사물의 월드 좌표를 추정."""
        ...
        
        


@runtime_checkable
class AiServiceProtocol(Protocol):
    """AI 서비스(Flask API) 인터페이스."""

    def drain_events(self) -> list[dict]:
        """
        누적된 AI 이벤트를 반환하고 내부 큐를 비운다.
        이벤트 스키마 예시:
          {"type": "person_detected",   "confidence": float, "bbox": dict}
          {"type": "person_lying_down", "position": dict}
          {"type": "face_recognized",   "name": str, "unit": str}
          {"type": "speech_done"}
          {"type": "stt_result",        "text": str, "confidence": float}
        """
        ...

    def call(self, endpoint: str, method: str = "GET",
             payload: dict | None = None,
             timeout: float = 5.0) -> dict | None:
        """
        온디맨드 Flask API 호출.

        지원 엔드포인트:
          TTS 발화는 BT ActionSpeak에서 local AI-TTS API /v1/tts/speak 로 처리
          POST /api/face/recognize  → {"name": str | None, "unit": str, "confidence": float}
          POST /api/camera/capture  → {"image_path": str}
          POST /api/stt/listen      → {"text": str | None, "confidence": float}
          GET  /api/detections/latest → {"person_count": int, "lying_down": bool}
        """
        ...

    def detect_fall(self) -> dict:
        """낙상 감지 결과 반환. STUB/실 API 모두 동일 스키마를 사용한다."""
        ...

    def detect_wander(self, person_id: str = "") -> dict:
        """15분 내 반복 감지 기반 배회 판정 결과 반환."""
        ...

    def detect_door_open(self) -> dict:
        """문열림 감지 결과 반환."""
        ...

    def recognize_face(self) -> dict:
        """얼굴 인식 결과 반환."""
        ...

    def start_conversation(self, conversation_type: str = "", context: dict | None = None) -> dict:
        """자유대화/상태확인 응답 텍스트 반환."""
        ...

    def begin_conversation_session(self) -> int:
        """자유대화 세션 시작 기준 이벤트 ID를 반환."""
        ...

    def wait_conversation_event(self, *, wait_timeout: float = 1.0) -> dict | None:
        """자유대화 중 새 Agent Events 1건을 짧게 polling."""
        ...


@runtime_checkable
class WiredServiceProtocol(Protocol):
    """OrangePi ↔ Jetson TCP JSONL 통신 인터페이스."""

    @property
    def has_client(self) -> bool:
        """현재 클라이언트(OrangePi)가 연결되어 있는지 여부."""
        ...

    def try_recv(self) -> dict | None:
        """
        수신 큐에서 메시지 1건 꺼내기. 없으면 None.
        비블로킹(즉시 반환).
        """
        ...

    def send(self, msg: dict) -> None:
        """OrangePi로 메시지 전송."""
        ...
