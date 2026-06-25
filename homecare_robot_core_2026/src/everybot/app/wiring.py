"""
wiring.py — 런타임 의존성 조립.

build_runtime(cfg) 가 유일한 공개 진입점.
모든 서비스/BT 컴포넌트를 생성하고 Runtime 으로 묶어 반환한다.

BT 통합 흐름 (v2):
  Runtime.tick()
    → MainService.tick()       ← 등록 FSM + wired 레거시 명령 처리
    → BtLayer.tick()           ← Bridge.update() + BT tree.tick()
    → settings_changed 처리   ← BB.settings_changed → RobotSettingsManager.update()
"""
from __future__ import annotations

import logging
import os
import json
from dataclasses import dataclass, field
from pathlib import Path

from ..utils.beam_projector_util import BeamProjectorUtil
from ..utils.robot_settings import RobotSettings
from ..utils.waypoint_manager import WaypointManager
from ..utils.robot_settings import RobotSettingsManager
from ..utils.state_store import StateStore
from ..utils.zone_manager import ZoneManager

from ..config.schema import RobotConfig
from ..services.base import Service
from ..services.wired_control_service import WiredControlService
from ..services.mobile_provision_service import MobileProvisionService
from ..services.main_service import MainService, RegState
from ..services.mqtt_service import MqttService
from ..services.amr_service import AmrService
from ..services.amr_constants import MovingState, RobotStatus
from ..services.ui_mqtt_service import UiMqttService
from ..services.jetson_ai_service import JetsonAiService
from ..services.network_monitor_service import NetworkMonitorService

from ..bt.blackboard import RobotBlackboard, ScheduleEntry
from ..bt.bridge import BlackboardBridge, ServiceBundle
from ..bt.tree import build_robot_tree
from ..bt.debug import DebugMode, RobotBTDebugger

log = logging.getLogger(__name__)


def _sync_speaker_volume(speaker_url: str, settings: RobotSettings) -> None:
    import threading

    def _call() -> None:
        try:
            import requests

            requests.post(
                f"{speaker_url}/volume",
                json={"type": "tts", "level": settings.tts_volume},
                timeout=3.0,
            )
            requests.post(
                f"{speaker_url}/volume",
                json={"type": "bgm", "level": settings.bgm_volume},
                timeout=3.0,
            )
        except Exception as exc:
            log.warning("[settings] speaker_volume sync failed: %s", exc)

    threading.Thread(target=_call, daemon=True).start()


def _make_status_fn(main_svc: MainService) -> callable:
    def fn() -> dict:
        if main_svc._softap and main_svc._softap.enabled:
            return {"netstat": 0, "net_ssid": main_svc._softap._cfg.ssid}
        ssid = (main_svc._wifi.current_ssid() or "") if main_svc._wifi else ""
        return {"netstat": 1, "net_ssid": ssid}

    return fn


def _load_json_config(path: str) -> dict:
    src = Path(path)
    if not src.exists():
        log.warning("[wiring] optional config not found: %s", path)
        return {}
    try:
        with src.open("r", encoding="utf-8") as fh:
            data = json.load(fh)
        return data if isinstance(data, dict) else {}
    except Exception as exc:
        log.warning("[wiring] config load failed %s: %s", path, exc)
        return {}


# ─────────────────────────────────────────────────────────────────────────────
# BT 계층 래퍼
# ─────────────────────────────────────────────────────────────────────────────

class BtLayer:
    """
    BT 트리 + BlackboardBridge 를 감싼 tick 단위 실행기.

    Runtime.tick() 에서 MainService.tick() 직후 호출한다.
    settings_changed 처리도 여기서 담당한다.
    """

    def __init__(
        self,
        bb:             RobotBlackboard,
        bridge:         BlackboardBridge,
        debugger:       RobotBTDebugger,
        settings_mgr:   RobotSettingsManager | None,
        settings_path:  str,
        speaker_base_url: str,
        mqtt_svc:       MqttService | None,
        ui_mqtt_svc:    UiMqttService | None,
        ui_status_interval: float,
        tick_log_interval: int = 100,
        bundle:         ServiceBundle | None = None,
    ) -> None:
        self._bb            = bb
        self._bridge        = bridge
        self._dbg           = debugger
        self._bundle        = bundle
        self._settings_mgr  = settings_mgr
        self._settings_path = settings_path
        self._speaker_base_url = speaker_base_url
        self._mqtt_svc = mqtt_svc
        self._ui_mqtt_svc = ui_mqtt_svc
        self._ui_status_interval = ui_status_interval
        self._tick_log_interval = max(tick_log_interval, 1)  # 0 방지
        self._last_ext_status: tuple[str, str] | None = None
        self._last_move_status_publish_time = 0.0
        self._last_move_status = ""
        self._last_move_target_id = ""
        self._move_motion_seen = False
        self._tick_count    = 0   # 주기적 BB 로그용

    def setup(self) -> None:
        """py_trees setup() 호출 (트리 시작 전 1회)."""
        self._dbg.setup()

    def tick(self) -> None:
        """1 tick: Bridge.update() → BT tick → settings 처리."""
        # 1. 서비스 → Blackboard 동기화
        passthrough = self._bridge.update()

        # 2. 레거시 wired 메시지 로그 (MainService 가 이미 처리)
        for msg in passthrough:
            log.debug("[BtLayer] passthrough: %s", msg.get("type"))

        # 3. BT tick
        self._dbg.tick()

        if self._mqtt_svc is not None:
            from ..utils.event_code import derive_status

            current = (self._bb.current_event_code, derive_status(self._bb))
            if current != self._last_ext_status:
                self._mqtt_svc.publish_external_status(
                    event_code=current[0],
                    status=current[1],
                    battery=self._bb.battery_percent,
                )
                self._last_ext_status = current

        self._publish_ui_status_if_due()
        self._publish_move_status_if_due()
        self._publish_config_if_due()

        # 4. 주기적 BB 상태 로그 (tick_log_interval ticks, YAML 설정)
        self._tick_count += 1
        if self._tick_count % self._tick_log_interval == 0:
            bb = self._bb
            log.info(
                "[BB] init=%s scenario=%s emg=%s map=%s wifi=%s battery=%.0f%%",
                bb.initialized,
                bb.active_scenario or "idle",
                bb.emergency_stop,
                bb.map_ready,
                bb.wifi_registered,
                bb.battery_percent,
            )

        # 4. settings_changed 처리 (BT 트리 외부)
        if self._bb.settings_changed and self._settings_mgr is not None:
            delta = self._bb.settings_pending
            try:
                self._settings_mgr.update(delta)
                if any(key in delta for key in ("speaker_volume", "tts_volume", "bgm_volume")):
                    _sync_speaker_volume(self._speaker_base_url, self._settings_mgr.get())
                log.info("[BtLayer] settings updated: %s", delta)
            except Exception as e:
                log.error("[BtLayer] settings update failed: %s", e)
            finally:
                self._bb.settings_changed = False
                self._bb.settings_pending = {}

    def _publish_ui_status_if_due(self) -> None:
        if self._ui_mqtt_svc is None or not self._ui_mqtt_svc.has_client:
            return
        import time

        now = time.time()
        if now - self._bb.last_status_publish_time >= self._ui_status_interval:
            self._bb.last_status_publish_time = now
            self._ui_mqtt_svc.send({
                "type": "robot_status",
                "payload": {
                    "uuid": "Everybot-robot",
                    "mode": self._ui_status_mode(),
                    "battery": self._bb.battery_percent,
                    "connected": True,
                    "event_code": self._bb.current_event_code,
                    "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                },
            })
        if (
            self._bb.current_location_id
            and now - self._bb.last_location_publish_time >= self._ui_status_interval
        ):
            self._bb.last_location_publish_time = now
            pos = self._bb.amr_position or {}
            loc_payload = {
                "location_id": self._bb.current_location_id,
                "location_name": self._bb.current_location_name,
                "x": pos.get("x", 0.0),
                "y": pos.get("y", 0.0),
                "theta": pos.get("theta", 0.0),
                "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            }
            self._ui_mqtt_svc.send({
                "type": "location_update",
                "payload": loc_payload,
            })
            log.info(
                "[BtLayer] robot/location published: id=%s name=%s (%.2f, %.2f)",
                loc_payload["location_id"],
                loc_payload["location_name"],
                loc_payload["x"],
                loc_payload["y"],
            )

    def _publish_move_status_if_due(self) -> None:
        if self._ui_mqtt_svc is None or not self._ui_mqtt_svc.has_client:
            return

        current_target_id = self._current_move_target_id()
        if current_target_id:
            if current_target_id != self._last_move_target_id:
                self._last_move_status = ""
                self._move_motion_seen = False
            self._last_move_target_id = current_target_id
        if self._bb.amr_moving_state in {
            MovingState.MOVING,
            MovingState.PAUSE,
            MovingState.ALTERNATIVE_GOAL,
        }:
            self._move_motion_seen = True

        target_id = current_target_id or self._last_move_target_id
        status = self._current_move_status_for_ui()
        if not target_id or not status:
            return
        if status == "moving" and not current_target_id:
            return

        import time

        now = time.time()
        status_changed = status != self._last_move_status
        due_for_moving = status == "moving" and (
            now - self._last_move_status_publish_time >= self._ui_status_interval
        )
        if not status_changed and not due_for_moving:
            return

        self._last_move_status_publish_time = now
        self._last_move_status = status
        self._ui_mqtt_svc.send({
            "type": "move_status",
            "payload": {
                "target_id": target_id,
                "status": status,
            },
        })
        if status in {"arrived", "cancelled", "failed"}:
            self._last_move_target_id = ""
            self._move_motion_seen = False

    def _current_move_status_for_ui(self) -> str:
        if self._bb.emergency_stop:
            return "cancelled"
        if self._bb.amr_arrived or self._bb.amr_moving_state == MovingState.ARRIVED:
            if not self._move_motion_seen:
                if str(self._bb.active_scenario or "") == "direct_move":
                    return "moving"
                return ""
            return "arrived"
        if (
            self._bb.amr_moving_state == MovingState.FAIL
            or self._bb.amr_robot_state == RobotStatus.ERROR
        ):
            return "failed"
        if self._bb.amr_moving_state in {
            MovingState.MOVING,
            MovingState.PAUSE,
            MovingState.READY,
            MovingState.ALTERNATIVE_GOAL,
        }:
            return "moving"

        # Direct move can enter scenario state before AMR movement state updates.
        if str(self._bb.active_scenario or "") == "direct_move":
            return "moving"
        return ""

    def _current_move_target_id(self) -> str:
        ctx_target = (
            self._bb.ctx.get("cmd_move_target")
            or self._bb.ctx.get("cmd_move_resolved_target")
            or self._bb.ctx.get("nav_waypoint_target")
            or self._bb.ctx.get("nav_waypoint_resolved_target")
        )
        if ctx_target:
            return str(ctx_target)

        params = self._bb.scenario_params if isinstance(self._bb.scenario_params, dict) else {}
        target_pos = params.get("target_pos")
        if target_pos:
            return str(target_pos)

        if self._bb.active_scenario == "morning_call" and self._bb.morning_call_visits:
            try:
                visit = self._bb.morning_call_visits[self._bb.current_visit_index]
            except (IndexError, TypeError):
                visit = {}
            if isinstance(visit, dict):
                visit_target = visit.get("waypoint_id") or visit.get("room_id")
                if visit_target:
                    return str(visit_target)

        waypoints = params.get("waypoints")
        if isinstance(waypoints, list) and waypoints:
            try:
                index = int(self._bb.ctx.get("wp_index", 0))
            except (TypeError, ValueError):
                index = 0
            if 0 <= index < len(waypoints):
                return str(waypoints[index])

        if self._bb.patrol_waypoints:
            try:
                patrol_index = int(self._bb.patrol_current_idx)
            except (TypeError, ValueError):
                patrol_index = 0
            if 0 <= patrol_index < len(self._bb.patrol_waypoints):
                return str(self._bb.patrol_waypoints[patrol_index])

        return ""

    def _ui_status_mode(self) -> str:
        """Return one of the UI-defined robot/status mode values."""
        scenario = str(self._bb.active_scenario or "")
        if self._bb.emergency_stop or self._bb.fall_detected or self._bb.wander_detected:
            return "emergency"
        if self._bb.amr_robot_state == 7 or self._bb.battery_percent <= 15.0:
            return "charging"
        if scenario == "morning_call":
            return "morning-call"
        if scenario in {"patrol_situation_check", "emergency_patrol", "patrol"}:
            return "patrol"
        if scenario in {"direct_move", "move_waypoints", "rotation"}:
            return "moving"
        return "idle"

    def _publish_config_if_due(self) -> None:
        """목적지 목록 스케줄을 N초 주기로 UI에 발행."""
        if self._ui_mqtt_svc is None or not self._ui_mqtt_svc.has_client:
            return
        if self._bundle is None:
            return

        import time
        now = time.time()
        _CONFIG_INTERVAL = 2.0  # 설정 데이터는 변경 빈도 낮음

        if now - self._bb.last_config_publish_time < _CONFIG_INTERVAL:
            return
        self._bb.last_config_publish_time = now

        date = time.strftime("%Y-%m-%d", time.gmtime())
        dest_count = 0
        visit_count = 0

        # 1) robot/config/destinations — bell_id 제외 (C6)
        if self._bundle.waypoint_mgr is not None:
            wps = self._bundle.waypoint_mgr.list()
            destinations = [
                {
                    "id": wp.key,
                    "name": wp.label,
                    "group": wp.type,
                    "desc": wp.comment,
                }
                for wp in wps
            ]
            dest_count = len(destinations)
            self._ui_mqtt_svc.send({
                "type": "config_destinations",
                "payload": {
                    "destinations": destinations,
                    "count": dest_count,
                    "date": date,
                },
            })
        else:
            log.warning("[BtLayer] config skip: waypoint_mgr is None")
        """
        # 2) robot/config/morning-call-schedule
        mc_cfg = self._bundle.feature_cfg.get("morning_call", {})
        if mc_cfg:
            visits = mc_cfg.get("visits", [])
            visit_count = len(visits)
            self._ui_mqtt_svc.send({
                "type": "config_schedule",
                "payload": {
                    "schedule_date" : date,
                    "operating_hours": mc_cfg.get("operating_hours", {}),
                    "visits": visits,
                    "max_door_retry": mc_cfg.get("max_door_retry", -1),
                    "door_wait_sec": mc_cfg.get("door_wait_sec", -1.0),
                },
            })
        else:
            log.warning("[BtLayer] config skip: morning_call config empty")
        """
        log.info(
            "[BtLayer] config published: %d destinations, %d visits",
            dest_count, visit_count,
        )


# ─────────────────────────────────────────────────────────────────────────────
# Runtime
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class Runtime:
    services: list[Service]
    main:     MainService
    bt:       BtLayer

    def start(self) -> None:
        for s in self.services:
            s.start()
        self.main.start()
        self.bt.setup()

    def tick(self) -> None:
        for s in self.services:
            s.tick()
        self.main.tick()
        self.bt.tick()

    def stop(self) -> None:
        try:
            self.main.stop()
        finally:
            for s in reversed(self.services):
                s.stop()


# ─────────────────────────────────────────────────────────────────────────────
# 팩토리
# ─────────────────────────────────────────────────────────────────────────────

def build_runtime(cfg: RobotConfig) -> Runtime:
    """
    전체 런타임을 조립하여 반환한다.

    순서:
      1. 기본 서비스 생성 (AMR, Wired, UiMqtt, AI)
      2. WaypointManager / RobotSettingsManager 초기화
      3. BB 초기 상태 설정 (wifi_registered, map_ready)
      4. BT 컴포넌트 조립 (ServiceBundle, Bridge, Tree, Debugger)
      5. MainService 생성
      6. Runtime 반환
    """

    # ── 1. 기본 서비스 ──────────────────────────────────────────────
    wired     = WiredControlService(cfg.wired_server)
    mobile    = MobileProvisionService(cfg.mobile_server)
    amr       = AmrService(
        amr_ip=cfg.amr.ip,
        amr_port=cfg.amr.port,
        recv_buf_size=cfg.amr.recv_buf_size,
    )
    projector = BeamProjectorUtil()
    ui_mqtt   = UiMqttService(cfg.ui_mqtt)
    net_mon   = NetworkMonitorService(
        wifi_ifname=cfg.network.wifi_ifname,
        auto_softap=cfg.network.auto_softap_on_disconnect,
        disconnect_grace=cfg.network.disconnect_grace_sec,
    )

    # ── 2. WaypointManager 초기화 ───────────────────────────────────
    waypoint_mgr = WaypointManager()
    wp_path = cfg.waypoints.file_path
    try:
        waypoint_mgr.load(wp_path)
        log.info("[wiring] WaypointManager ready: %s (%d waypoints)",
                 wp_path, len(waypoint_mgr.list()))
    except Exception as e:
        log.warning("[wiring] waypoints load failed (%s): %s", wp_path, e)

    # ── 3. RobotSettingsManager 초기화 ──────────────────────────────
    settings_mgr  = RobotSettingsManager()
    settings_path = cfg.robot_settings.file_path
    if os.path.exists(settings_path):
        try:
            settings_mgr.load(settings_path)
            log.info("[wiring] RobotSettingsManager loaded: %s", settings_path)
        except Exception as e:
            log.warning("[wiring] settings load failed (%s): %s", settings_path, e)
    else:
        log.info("[wiring] settings file not found (%s) — defaults", settings_path)

    # ── 4. 5/21 데모 설정 로드 ─────────────────────────────────
    feature_cfg = _load_json_config(cfg.feature_settings.file_path)
    program_schedule = _load_json_config(cfg.program_schedule.file_path)
    ai_cfg = feature_cfg.get("ai_integration", {}) if isinstance(feature_cfg, dict) else {}
    ai_svc = JetsonAiService(
        fall_status_url=cfg.hw_api.fall_status_url,
        fall_status_timeout=cfg.hw_api.fall_status_timeout,
        agent_events_base_url=cfg.hw_api.agent_events_url,
        agent_events_timeout=cfg.hw_api.agent_events_timeout,
        conversation_wait_timeout=cfg.hw_api.conversation_wait_timeout,
        face_api_base_url=cfg.hw_api.face_api_url,
        face_api_timeout=cfg.hw_api.face_api_timeout,
        tts_api_base_url=cfg.hw_api.tts_url,
        tts_poll_interval=float(ai_cfg.get("tts_poll_interval_sec", 0.25) or 0.25),
        tts_done_timeout=float(ai_cfg.get("tts_done_timeout_sec", 20.0) or 20.0),
        agent_events_baseline_on_start=bool(ai_cfg.get("agent_events_baseline_on_start", True)),
        agent_events_baseline_on_conversation_start=bool(ai_cfg.get("agent_events_baseline_on_conversation_start", True)),
        conversation_followup_grace_sec=float(ai_cfg.get("conversation_followup_grace_sec", 15.0) or 15.0),
        fall_candidate_min_hits=int(ai_cfg.get("fall_candidate_min_hits", 2) or 2),
        fall_candidate_window_sec=float(ai_cfg.get("fall_candidate_window_sec", 2.0) or 2.0),
        fall_candidate_requires_clear=bool(ai_cfg.get("fall_candidate_requires_clear", True)),
        fall_cooldown_sec=float(ai_cfg.get("fall_cooldown_sec", 30.0) or 30.0),
        wander_min_count=int(ai_cfg.get("wander_min_count", 3) or 3),
        wander_window_sec=float(ai_cfg.get("wander_window_sec", 60.0) or 60.0),
        wander_cooldown_sec=float(ai_cfg.get("wander_cooldown_sec", 30.0) or 30.0),
        wander_unknown_only=bool(ai_cfg.get("wander_unknown_only", True)),
    )
    zone_mgr = ZoneManager()
    zone_mgr.load(cfg.map.roi_zones_path)

    # ── 4. Blackboard 초기 상태 설정 ────────────────────────────────
    bb = RobotBlackboard()
    morning_cfg = feature_cfg.get("morning_call", {}) if isinstance(feature_cfg, dict) else {}
    patrol_cfg = feature_cfg.get("patrol", {}) if isinstance(feature_cfg, dict) else {}
    morning_hours = morning_cfg.get("operating_hours", {}) if isinstance(morning_cfg, dict) else {}
    patrol_hours = patrol_cfg.get("operating_hours", {}) if isinstance(patrol_cfg, dict) else {}
    bb.operating_hours["morning_call"] = morning_hours
    bb.operating_hours["emergency"] = patrol_hours
    bb.morning_call_visits = list(morning_cfg.get("visits", [])) if isinstance(morning_cfg, dict) else []
    bb.patrol_waypoints = list(patrol_cfg.get("waypoint_sequence", [])) if isinstance(patrol_cfg, dict) else []
    if isinstance(patrol_cfg, dict):
        bb.patrol_dwell_sec = int(patrol_cfg.get("dwell_time_sec", bb.patrol_dwell_sec) or bb.patrol_dwell_sec)
    bb.schedule_table = [
        ScheduleEntry(
            scenario_id="morning_call",
            enabled=True,
            trigger_time=str(morning_cfg.get("trigger_time", "") or ""),
            operating_start=str(morning_hours.get("start", "") or ""),
            operating_end=str(morning_hours.get("end", "") or ""),
        ),
        ScheduleEntry(
            scenario_id="emergency",
            enabled=True,
            operating_start=str(patrol_hours.get("start", "") or ""),
            operating_end=str(patrol_hours.get("end", "") or ""),
        ),
    ]

    # WiFi 등록 확인: state.json 의 home_ssid + home_password 존재 여부
    state_store = StateStore(cfg.state.path)
    try:
        state_data = state_store.load()
        home_ssid  = state_data.get("home_ssid", "")
        home_psk   = state_data.get("home_password", "")
        bb.wifi_registered = bool(home_ssid) and bool(home_psk)
        log.info("[wiring] wifi_registered=%s (ssid=%s)",
                 bb.wifi_registered, home_ssid or "(none)")
    except Exception as e:
        log.warning("[wiring] state.json read failed: %s — wifi_registered=False", e)
        bb.wifi_registered = False

    # 맵 파일 존재 확인 (PGM 기준)
    pgm_path  = cfg.map.pgm_path
    bb.map_ready = os.path.exists(pgm_path)
    log.info("[wiring] map_ready=%s (%s)", bb.map_ready, pgm_path)

    # ── 5. MainService 먼저 생성 (bundle이 main._softap, main._wifi 참조 필요) ──
    main = MainService(
        robot_name    = cfg.name,
        wired         = wired,
        mobile        = mobile,
        amr           = amr,
        softap_cfg    = cfg.softap,
        home_wifi_cfg = cfg.home_wifi,
        mqtt_cfg      = cfg.mqtt,
        ui_mqtt       = ui_mqtt,
        state_path    = cfg.state.path,
        projector     = projector,
        ha_enabled    = cfg.ha.enabled,
        test          = 0,
    )
    main._mqtt.set_bb(bb)
    main._mqtt.set_ui_mqtt(ui_mqtt)
    main._mqtt.set_status_fn(_make_status_fn(main))
    net_mon.set_wifi(main._wifi)
    net_mon.set_softap(main._softap)
    net_mon.set_bb(bb)

    # ── 6. BT 컴포넌트 조립 ─────────────────────────────────────────
    bundle = ServiceBundle(
        amr          = amr,
        ai           = ai_svc,
        wired        = wired,
        ui_mqtt      = ui_mqtt,
        waypoints    = waypoint_mgr.as_dict(),    # 기존 dict 호환
        waypoint_mgr = waypoint_mgr,
        settings_mgr = settings_mgr,
        # v2 신규: WiFi 프로비저닝 + Force SoftAP
        softap       = main._softap,              # SoftApManager 인스턴스
        wifi         = main._wifi,                # WifiManager 인스턴스
        wifi_reg_fn  = main.is_wifi_registered,   # 등록 상태 콜백
        speaker_base_url = cfg.hw_api.speaker_url,
        rf_base_url      = cfg.hw_api.rf_url,
        camera_base_url  = cfg.hw_api.camera_url,
        mic_base_url     = cfg.hw_api.mic_url,
        tts_api_base_url = cfg.hw_api.tts_url,
        zone_mgr         = zone_mgr,
        feature_cfg      = feature_cfg,
        program_schedule = program_schedule,
    )

    bridge = BlackboardBridge(bb, bundle)

    root = build_robot_tree(
        bb, bundle,
        map_pgm_path       = cfg.map.pgm_path,
        map_yaml_path      = cfg.map.yaml_path,
        waypoints_path     = cfg.waypoints.file_path,
        forbidden_zones_path = cfg.map.forbidden_zones_path,
        roi_zones_path       = cfg.map.roi_zones_path,
        map_creation_port  = cfg.map_creation.server_port,
        tts_root           = cfg.tts.file_root,
        bgm_root           = cfg.bgm.file_root,
    )

    # BT 디버그 로그 활성화(SNAPSHOT) or 비활성화(SILENT)
    try:
        debug_mode = DebugMode[cfg.bt.debug_mode.upper()]
    except KeyError:
        log.warning("[wiring] unknown bt.debug_mode '%s' -> SNAPSHOT", cfg.bt.debug_mode)
        debug_mode = DebugMode.SNAPSHOT
    debugger = RobotBTDebugger(root, bb, debug_mode, snapshot_interval=cfg.bt.tick_log_interval)
    #debugger = RobotBTDebugger(root, bb, DebugMode.SILENT)

    bt_layer = BtLayer(
        bb            = bb,
        bridge        = bridge,
        debugger      = debugger,
        settings_mgr  = settings_mgr,
        settings_path = settings_path,
        speaker_base_url = cfg.hw_api.speaker_url,
        mqtt_svc      = main._mqtt,
        ui_mqtt_svc   = ui_mqtt,
        ui_status_interval = cfg.ui_mqtt.status_publish_interval,
        tick_log_interval  = cfg.bt.tick_log_interval,
        bundle        = bundle,
    )

    # ── 7. Runtime 반환 ─────────────────────────────────────────────
    services: list[Service] = [
        wired,
        amr,
        ai_svc,
        ui_mqtt,
        net_mon,
    ]

    return Runtime(services=services, main=main, bt=bt_layer)
