"""
BlackboardBridge — 서비스 큐 → RobotBlackboard 동기화 계층.

MainService.tick() 최상단에서 호출하여 모든 서비스의 최신 상태를
Blackboard 에 반영한다.

시나리오 제어 명령(request_scenario_start/stop)은 Bridge 가 처리하고,
레거시 명령은 passthrough 리스트로 반환하여 MainService 가 처리한다.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Callable

from .blackboard import RobotBlackboard
from ..services.interfaces import AmrServiceProtocol, AiServiceProtocol, WiredServiceProtocol
from ..utils.waypoint_manager import WaypointManager
from ..utils.robot_settings import RobotSettingsManager
from ..utils.event_code import EventCode, SCENARIO_EVENT_MAP, resolve_scenario_id
from ..utils.settings_schema import validate_settings_delta
from ..drivers.softap_manager import SoftApManager
from ..drivers.wifi_manager import WifiManager

log = logging.getLogger(__name__)


@dataclass
class ServiceBundle:
    """BT 노드가 접근하는 서비스 참조 묶음."""
    amr:       AmrServiceProtocol
    ai:        AiServiceProtocol
    wired:     WiredServiceProtocol          # TCP (기존 병행 유지)
    waypoints: dict[str, dict]
    # waypoints: "entrance" → {"x": 1.0, "y": 2.0, "theta": 0.0}

    # OrangePi ↔ Jetson 로컬 MQTT (None이면 비활성)
    ui_mqtt:      WiredServiceProtocol | None = None

    # v2 신규: 목적지/설정 매니저
    waypoint_mgr: WaypointManager | None      = None
    settings_mgr: RobotSettingsManager | None = None

    # v2 신규: WiFi 프로비저닝 + Force SoftAP
    softap:      SoftApManager | None          = None  # Force SoftAP 제어
    wifi:        WifiManager   | None          = None  # home WiFi 재연결
    wifi_reg_fn: Callable[[], bool] | None     = None  # 등록 상태 콜백 (Bridge 동기화용)

    # v2 신규: RF 트랜시버
    rf_base_url: str = "http://localhost:8084"  # rf_manager HTTP API base URL

    # speaker-refactoring 신규: BT 스피커 매니저
    speaker_base_url: str = "http://localhost:8083"  # speaker_manager HTTP API base URL
    camera_base_url:  str = "http://localhost:8081"  # camera_manager HTTP API base URL
    mic_base_url:     str = "http://localhost:8082"  # mic_manager HTTP API base URL

    def send_to_ui(self, msg: dict) -> None:
        """wired(TCP) + ui_mqtt(MQTT) 두 채널에 동시 전송.

        각 채널의 has_client를 확인하여 연결된 채널에만 전송한다.
        """
        if self.wired.has_client:
            self.wired.send(msg)
        if self.ui_mqtt is not None and self.ui_mqtt.has_client:
            self.ui_mqtt.send(msg)


class BlackboardBridge:
    """
    매 tick MainService.tick() 최상단에서 update() 를 호출한다.

    역할:
      1. AMR 상태(이동 상태, 위치, 도착 이벤트)를 BB 에 복사
      2. AI 이벤트를 drain 하여 BB 에 교체
      3. Wired 수신 메시지를 파싱:
           - 시나리오 명령 → BB 갱신 (처리 완료)
           - 레거시 명령  → passthrough 리스트로 반환
    """

    def __init__(self, bb: RobotBlackboard, bundle: ServiceBundle) -> None:
        self._bb     = bb
        self._bundle = bundle

    def update(self) -> list[dict]:
        """
        모든 서비스 큐를 drain 하여 Blackboard 를 최신 상태로 갱신한다.

        Returns:
            MainService._handle_wired_legacy() 로 전달할 메시지 목록.
        """
        bb  = self._bb
        svc = self._bundle

        # 1. AMR 상태 갱신 ──────────────────────────────────────────
        bb.amr_moving_state = svc.amr.cached_moving_state
        bb.amr_position     = svc.amr.cached_position
        bb.amr_robot_state  = svc.amr.cached_robot_status
        bb.amr_arrived      = svc.amr.pop_arrived_event()

        # MockAmrService / Real AmrService 모두 battery_percent 프로퍼티 보유.
        if hasattr(svc.amr, "battery_percent"):
            bb.battery_percent = svc.amr.battery_percent  # type: ignore[attr-defined]

        # Real AmrService 는 latest_map_data 속성을 갖는다 (MapData 수신 시 갱신).
        # ActionSaveMap 이 bb.latest_map_data 를 읽으므로 bridge 에서 동기화.
        if hasattr(svc.amr, "latest_map_data"):
            new_map = svc.amr.latest_map_data  # type: ignore[attr-defined]
            if new_map and new_map is not bb.latest_map_data:
                bb.latest_map_data = new_map

        # 2. AI 이벤트 drain ────────────────────────────────────────
        bb.ai_events = svc.ai.drain_events()

        # 3. Wired 메시지 파싱 (TCP) ────────────────────────────────
        passthrough: list[dict] = []
        while True:
            msg = svc.wired.try_recv()
            if msg is None:
                break
            if not self._parse_wired(msg):
                passthrough.append(msg)

        # 4. WiFi 등록 상태 런타임 동기화 ────────────────────────────
        if svc.wifi_reg_fn is not None:
            new_val = svc.wifi_reg_fn()
            if new_val != bb.wifi_registered:
                log.info("[Bridge] wifi_registered: %s → %s", bb.wifi_registered, new_val)
                bb.wifi_registered = new_val

        # 5. UiMqtt 메시지 파싱 (MQTT) ──────────────────────────────
        if svc.ui_mqtt is not None:
            while True:
                msg = svc.ui_mqtt.try_recv()
                if msg is None:
                    break
                log.info("[Bridge] UiMqtt msg received: type=%s", msg.get("type"))
                if not self._parse_wired(msg):   # 동일 파서 재사용
                    passthrough.append(msg)

        return passthrough

    # ── Private ─────────────────────────────────────────────────────

    def _parse_wired(self, msg: dict) -> bool:
        """
        True  → Bridge 가 처리 완료 (BB 업데이트)
        False → passthrough (MainService 가 처리)
        """
        msg_type = msg.get("type", "")
        payload  = msg.get("payload", {})
        
        if msg_type == "status":
            return True

        if msg_type == "request_scenario_start":
            raw_scenario = payload.get("scenario_id")
            new_scenario = resolve_scenario_id(raw_scenario)
            self._bb.active_scenario = new_scenario
            params = dict(payload.get("params", {}) or {})
            if raw_scenario == "facility_guidance" and "target_pos" not in params:
                params["target_pos"] = params.get("target_facility", "")
            self._bb.scenario_params = params
            self._bb.clear_ctx()
            self._bb.current_event_code = SCENARIO_EVENT_MAP.get(
                str(new_scenario), EventCode.NORMAL
            ) if new_scenario else EventCode.NORMAL
            if raw_scenario and raw_scenario != new_scenario:
                log.info("[Bridge] legacy scenario_id %s -> %s", raw_scenario, new_scenario)
            log.info("[Bridge] scenario_start → %s params=%s",
                     self._bb.active_scenario, self._bb.scenario_params)
            return True

        if msg_type == "request_scenario_stop":
            log.info("[Bridge] scenario_stop (was %s)", self._bb.active_scenario)
            self._bb.active_scenario = None
            self._bb.scenario_params = {}
            self._bb.clear_ctx()
            self._bb.current_event_code = EventCode.NORMAL
            self._bb.voice_agent_intent = ""
            self._bb.voice_agent_response = ""
            self._bb.voice_agent_action = {}
            return True

        if msg_type == "request_emergency_stop":
            self._bb.emergency_stop = True
            self._bb.current_event_code = EventCode.STOP
            log.warning("[Bridge] EMERGENCY STOP activated")
            return True

        if msg_type == "request_emergency_release":
            self._bb.emergency_stop = False
            self._bb.current_event_code = EventCode.NORMAL
            log.info("[Bridge] emergency released")
            return True

        # ── v2 신규 파싱 ──────────────────────────────────────────

        if msg_type == "request_call":
            if not isinstance(payload, dict):
                payload = {}
            call_type = str(payload.get("call_type", "general") or "general").strip() or "general"
            self._bb.active_scenario = "concierge"
            self._bb.scenario_params = {
                "call_type": call_type,
                "message": str(payload.get("message", "") or ""),
                "room_id": str(payload.get("room_id", "") or ""),
            }
            self._bb.clear_ctx()
            self._bb.current_event_code = EventCode.CONCIERGE
            if call_type == "emergency":
                log.warning("[Bridge] emergency concierge call requested: %s", self._bb.scenario_params)
            else:
                log.info("[Bridge] concierge call requested: %s", self._bb.scenario_params)
            return True

        if msg_type == "voice_action_result":
            if not isinstance(payload, dict):
                payload = {}
            self._bb.voice_agent_intent = str(payload.get("intent", "") or "")
            self._bb.voice_agent_response = str(
                payload.get("tts_text") or payload.get("response") or ""
            )
            action = payload.get("action", {}) or {}
            params_raw = payload.get("parameters", {}) or {}

            params = dict(params_raw) if isinstance(params_raw, dict) else {}
            if isinstance(action, dict) and action:
                self._bb.voice_agent_action = dict(action)
            else:
                self._bb.voice_agent_action = dict(params)
            if payload.get("action_id"):
                self._bb.voice_agent_action["action_id"] = str(payload.get("action_id"))
            raw_scenario = (
                params.get("scenario_id")
                or payload.get("scenario_id")
                or self._bb.voice_agent_action.get("scenario_id")
            )
            scenario_id = resolve_scenario_id(raw_scenario)
            if scenario_id:
                self._bb.active_scenario = scenario_id
                self._bb.scenario_params = params
                self._bb.clear_ctx()
                self._bb.current_event_code = SCENARIO_EVENT_MAP.get(
                    scenario_id, EventCode.NORMAL
                )
                log.info(
                    "[Bridge] voice action scenario_start → %s params=%s",
                    scenario_id,
                    self._bb.scenario_params,
                )
            else:
                log.info("[Bridge] voice action result stored intent=%s", self._bb.voice_agent_intent)
            return True

        if msg_type == "request_picture_capture":
            self._bb.active_scenario = "photo_service"
            self._bb.scenario_params = dict(payload if isinstance(payload, dict) else {})
            self._bb.clear_ctx()
            self._bb.current_event_code = EventCode.PHOTO
            log.info("[Bridge] picture capture requested params=%s", self._bb.scenario_params)
            return True

        if msg_type == "request_factory_reset":
            self._bb.factory_reset = True
            log.warning("[Bridge] FACTORY RESET requested")
            return True

        if msg_type == "settings_change_value":
            if not isinstance(payload, dict):
                log.warning("[Bridge] invalid settings payload type: %s", type(payload).__name__)
                return True
            valid_delta = validate_settings_delta(payload)
            if not valid_delta:
                log.warning("[Bridge] settings change ignored: no valid keys")
                return True
            self._bb.settings_pending = valid_delta
            self._bb.settings_changed = True
            log.info("[Bridge] settings change: %s", valid_delta)
            return True

        if msg_type == "cmd_force_softap":
            self._bb.force_softap = bool(payload.get("enabled", False))
            log.info("[Bridge] force_softap → %s", self._bb.force_softap)
            return True

        return False
