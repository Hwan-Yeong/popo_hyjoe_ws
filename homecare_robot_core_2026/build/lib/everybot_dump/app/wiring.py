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
from dataclasses import dataclass, field

from ..utils.beam_projector_util import BeamProjectorUtil
from ..utils.robot_settings import RobotSettings
from ..utils.waypoint_manager import WaypointManager
from ..utils.robot_settings import RobotSettingsManager
from ..utils.state_store import StateStore

from ..config.schema import RobotConfig
from ..services.base import Service
from ..services.wired_control_service import WiredControlService
from ..services.mobile_provision_service import MobileProvisionService
from ..services.main_service import MainService, RegState
from ..services.mqtt_service import MqttService
from ..services.amr_service import AmrService
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
    ) -> None:
        self._bb            = bb
        self._bridge        = bridge
        self._dbg           = debugger
        self._settings_mgr  = settings_mgr
        self._settings_path = settings_path
        self._speaker_base_url = speaker_base_url
        self._mqtt_svc = mqtt_svc
        self._last_ext_status: tuple[str, str] | None = None
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

        # 4. 주기적 BB 상태 로그 (5초 = 100 ticks @ 20Hz)
        self._tick_count += 1
        if self._tick_count % 100 == 0:
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
    ai_svc    = JetsonAiService()    # STUB: 실제 AI 연동은 추후 구현
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

    # ── 4. Blackboard 초기 상태 설정 ────────────────────────────────
    bb = RobotBlackboard()
    bb.schedule_table = [
        ScheduleEntry(
            scenario_id="morning_call",
            enabled=True,
            trigger_time="07:30",
            days=["mon", "tue", "wed", "thu", "fri"],
        ),
        ScheduleEntry(
            scenario_id="music_play",
            enabled=True,
            trigger_time="18:00",
            zone_id="corridor_1f",
            days=["mon", "tue", "wed", "thu", "fri"],
        ),
        ScheduleEntry(
            scenario_id="emergency",
            enabled=True,
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
    debugger = RobotBTDebugger(root, bb, debug_mode)
    #debugger = RobotBTDebugger(root, bb, DebugMode.SILENT)

    bt_layer = BtLayer(
        bb            = bb,
        bridge        = bridge,
        debugger      = debugger,
        settings_mgr  = settings_mgr,
        settings_path = settings_path,
        speaker_base_url = cfg.hw_api.speaker_url,
        mqtt_svc      = main._mqtt,
    )

    # ── 7. Runtime 반환 ─────────────────────────────────────────────
    services: list[Service] = [
        wired,
        amr,
        ui_mqtt,
        net_mon,
    ]

    return Runtime(services=services, main=main, bt=bt_layer)
