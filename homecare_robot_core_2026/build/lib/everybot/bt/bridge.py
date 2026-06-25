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
    tts_api_base_url: str = "http://127.0.0.1:8085"  # local AI-TTS queue API base URL

    # 5/21 demo: ROI/기능 설정
    zone_mgr: object | None = None
    feature_cfg: dict = field(default_factory=dict)
    program_schedule: dict = field(default_factory=dict)

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
        if bb.amr_arrived:
            bb.amr_arrived_state = getattr(
                svc.amr,
                "cached_arrived_state",
                bb.amr_moving_state,
            )
        else:
            bb.amr_arrived_state = None

        # AmrService battery_percent 프로퍼티 동기화.
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

        # 6. ROI 위치 갱신 ─────────────────────────────────────────
        if svc.zone_mgr is not None and bb.amr_position:
            try:
                x = float(bb.amr_position.get("x", 0.0))
                y = float(bb.amr_position.get("y", 0.0))
                zone = svc.zone_mgr.get_zone(x, y)  # type: ignore[attr-defined]
                if zone:
                    bb.current_location_id = str(zone.get("zone_id", "") or "")
                    bb.current_location_name = str(zone.get("zone_name", "") or "")
                else:
                    # GAP-03 fix: ROI 미매핑 시 nearest waypoint fallback
                    nearest = self._find_nearest_waypoint(x, y, svc)
                    if nearest:
                        bb.current_location_id = nearest["key"]
                        bb.current_location_name = nearest.get("label", nearest["key"])
                    else:
                        bb.current_location_id = ""
                        bb.current_location_name = ""
            except Exception as exc:
                log.debug("[Bridge] ROI update skipped: %s", exc)

        return passthrough

    # ── Private ─────────────────────────────────────────────────────

    @staticmethod
    def _find_nearest_waypoint(x: float, y: float, svc: ServiceBundle) -> dict | None:
        """ROI 미매핑 좌표에 대해 가장 가까운 waypoint 을 반환 (fallback)."""
        import math
        best, best_dist = None, float("inf")
        wp_dict = svc.waypoints or {}
        for key, info in wp_dict.items():
            if not isinstance(info, dict):
                continue
            wx = float(info.get("x", 0.0))
            wy = float(info.get("y", 0.0))
            d = math.hypot(x - wx, y - wy)
            if d < best_dist:
                # waypoints dict 에는 label 없음 → waypoint_mgr 에서 조회
                label = key
                if svc.waypoint_mgr is not None:
                    wp_obj = svc.waypoint_mgr.get(key)
                    if wp_obj and hasattr(wp_obj, "label"):
                        label = wp_obj.label or key
                best_dist = d
                best = {"key": key, "label": label, "dist": d}
        # 3m 이내만 유효
        if best and best["dist"] <= 3.0:
            return best
        return None

    def _parse_wired(self, msg: dict) -> bool:
        """
        True  → Bridge 가 처리 완료 (BB 업데이트)
        False → passthrough (MainService 가 처리)
        """
        msg_type = msg.get("type", "")
        payload  = msg.get("payload", {})
        if not isinstance(payload, dict):
            payload = {}
        
        if msg_type == "status":
            return True

        if msg_type == "debug_fall_detected":
            detected = self._payload_bool(
                payload,
                "fall_detected",
                "detected",
                default=True,
            )
            candidate = self._payload_bool(
                payload,
                "fall_candidate",
                "candidate",
                default=False,
            )
            self._bb.fall_detected = detected
            self._bb.fall_candidate = candidate
            self._bb.fall_status = str(
                payload.get("fall_status", payload.get("status", "debug" if detected else "clear")) or ""
            )
            self._bb.fall_confidence = self._payload_float(
                payload,
                "confidence",
                default=1.0 if detected or candidate else 0.0,
            )
            self._bb.fall_image_path = str(payload.get("image_path", "") or "")
            self._bb.ctx["debug_fall_detected"] = detected
            self._bb.ctx["debug_fall_candidate"] = candidate
            if detected or candidate:
                self._bb.detection_type = "fall"
                position = payload.get("position", payload.get("coordinates", {})) or {}
                if isinstance(position, dict):
                    self._bb.ctx["detected_person_position"] = position
            elif self._bb.detection_type == "fall":
                self._bb.detection_type = ""
                self._bb.ctx.pop("detected_person_position", None)
                self._bb.ctx.pop("debug_fall_detected", None)
                self._bb.ctx.pop("debug_fall_candidate", None)
            log.warning(
                "[Bridge] debug fall detected=%s candidate=%s status=%s",
                self._bb.fall_detected,
                self._bb.fall_candidate,
                self._bb.fall_status,
            )
            return True

        if msg_type == "debug_wander_detected":
            detected = self._payload_bool(
                payload,
                "wander_detected",
                "detected",
                "repeated",
                default=True,
            )
            self._bb.wander_detected = detected
            self._bb.wander_person_id = str(
                payload.get("person_id", self._bb.wander_person_id or "debug_person") or ""
            )
            self._bb.wander_count = int(
                self._payload_float(payload, "count", default=2.0 if detected else 0.0)
            )
            self._bb.wander_image_path = str(payload.get("image_path", "") or "")
            self._bb.ctx["debug_wander_detected"] = detected
            if detected:
                self._bb.detection_type = "wander"
            elif self._bb.detection_type == "wander":
                self._bb.detection_type = ""
                self._bb.ctx.pop("debug_wander_detected", None)
            log.warning(
                "[Bridge] debug wander detected=%s person=%s count=%d",
                self._bb.wander_detected,
                self._bb.wander_person_id,
                self._bb.wander_count,
            )
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
            self._bb.move_stop_requested = True
            self._bb.ctx["stop_reason"] = str(payload.get("reason", "scenario_stop") or "scenario_stop")
            self._bb.ctx["stop_command_id"] = str(payload.get("command_id", "") or "")
            return True

        if msg_type == "cmd_move":
            target_id = str(payload.get("target_location_id", "") or "")
            target_name = str(payload.get("target_location_name", "") or "")
            self._bb.active_scenario = "direct_move"
            self._bb.scenario_params = {"target_pos": target_id}
            self._bb.clear_ctx()
            self._bb.ctx["cmd_move_target"] = target_id
            self._bb.ctx["cmd_move_name"] = target_name
            self._bb.ctx["cmd_move_command_id"] = str(payload.get("command_id", "") or "")
            self._bb.ctx["cmd_move_requested_by"] = str(payload.get("requested_by", "") or "")
            log.info("[Bridge] cmd_move → scenario=direct_move target=%s name=%s",
                     target_id, target_name)
            return True

        if msg_type == "cmd_pause":
            self._bb.ctx["paused"] = True
            self._bb.ctx["pause_reason"] = str(payload.get("reason", "") or "")
            self._bb.ctx["pause_command_id"] = str(payload.get("command_id", "") or "")
            log.info("[Bridge] cmd_pause reason=%s", self._bb.ctx["pause_reason"])
            return True

        if msg_type == "cmd_resume":
            self._bb.ctx.pop("paused", None)
            self._bb.ctx.pop("pause_reason", None)
            self._bb.ctx["resume_command_id"] = str(payload.get("command_id", "") or "")
            log.info("[Bridge] cmd_resume")
            return True

        if msg_type == "cmd_stop":
            self._bb.move_stop_requested = True
            self._bb.ctx["stop_reason"] = str(payload.get("reason", "move_stop") or "move_stop")
            self._bb.ctx["stop_command_id"] = str(payload.get("command_id", "") or "")
            log.info("[Bridge] cmd_stop as move_stop reason=%s", self._bb.ctx["stop_reason"])
            return True

        if msg_type == "cmd_release":
            self._bb.emergency_stop = False
            self._bb.current_event_code = EventCode.NORMAL
            log.info("[Bridge] emergency released")
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

        if msg_type == "cmd_map_edit":
            self._bb.map_edit = bool(payload.get("enabled", False))
            log.info("[Bridge] map_edit → %s", self._bb.map_edit)
            return True

        return False

    @staticmethod
    def _payload_bool(payload: dict, *keys: str, default: bool = False) -> bool:
        for key in keys:
            if key not in payload:
                continue
            value = payload.get(key)
            if isinstance(value, bool):
                return value
            if isinstance(value, (int, float)):
                return value != 0
            text = str(value).strip().lower()
            if text in ("1", "true", "yes", "y", "on", "detected", "success"):
                return True
            if text in ("0", "false", "no", "n", "off", "clear", "none"):
                return False
        return default

    @staticmethod
    def _payload_float(payload: dict, key: str, default: float = 0.0) -> float:
        try:
            return float(payload.get(key, default) or default)
        except (TypeError, ValueError):
            return default
