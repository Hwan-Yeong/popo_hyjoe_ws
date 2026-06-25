"""
MockAmrService — AMR 서비스 Mock 구현.

nav_duration 초 후 자동 IDLE 전환.
inject_arrived() 로 즉시 도착 이벤트 주입 가능.
전송된 명령은 _sent_cmds 에 누적되어 테스트 검증에 사용한다.
"""
from __future__ import annotations

import time
import logging
from dataclasses import dataclass, field

from ..amr_constants import (
    AmrCmd,
    AmrDtype,
    DrivingSet,
    MovingState,
    build_bypass_request,
    build_driving_request,
    build_nav_request,
    build_rotation_request,
)

log = logging.getLogger(__name__)


@dataclass
class MockAmrService:
    """
    AmrServiceProtocol 을 만족하는 Mock 구현.

    실제 UDP 통신 없이 nav_duration 경과 시 자동 IDLE 전환을
    시뮬레이션한다.
    """
    nav_duration: float = 3.0       # 이동 소요 시간(초)
    initial_battery: float = 85.0

    # ── Private state ────────────────────────────────────────────
    _moving_state:    int           = field(default=MovingState.IDLE, init=False)
    _position:        dict          = field(default_factory=lambda: {"x": 0.0, "y": 0.0, "theta": 0.0}, init=False)
    _arrived_flag:    bool          = field(default=False, init=False)
    _nav_start:       float         = field(default=0.0, init=False)
    _nav_target:      dict | None   = field(default=None, init=False)
    _battery:         float         = field(default=0.0, init=False)
    _sent_cmds:       list[dict]    = field(default_factory=list, init=False)
    _prev_state:      int           = field(default=MovingState.IDLE, init=False)
    _latest_map_data: dict | None   = field(default=None, init=False)

    def __post_init__(self) -> None:
        self._battery = self.initial_battery

    # ── AmrServiceProtocol ───────────────────────────────────────

    @property
    def cached_moving_state(self) -> int:
        """nav_start 이후 nav_duration 경과 시 자동 IDLE 전환."""
        self._update_state()
        return self._moving_state

    @property
    def cached_position(self) -> dict:
        return dict(self._position)

    def pop_arrived_event(self) -> bool:
        """엣지 트리거: IDLE 전환 순간 1회만 True."""
        self._update_state()
        if self._arrived_flag:
            self._arrived_flag = False
            return True
        return False

    def send_target_position(self, coord: dict) -> None:
        """cmd=60 목적지 설정만 기록하고 이동 상태로 전환."""
        self._moving_state = MovingState.MOVING
        self._nav_start    = time.monotonic()
        self._nav_target   = dict(coord)
        self._arrived_flag = False

        # 로그용 cmd 기록
        self._sent_cmds.append({
            "cmd":  AmrCmd.TARGET_POSITION,
            "dtype": AmrDtype.SET,
            "body": build_nav_request(**{k: coord.get(k, 0.0)
                                         for k in ("x", "y", "theta")}),
        })
        log.debug("[MockAMR] send_target_position target=%s", coord)

    def send_stop(self) -> None:
        """즉시 정지."""
        self._moving_state = MovingState.IDLE
        self._nav_target   = None
        self._sent_cmds.append({
            "cmd":  AmrCmd.DRIVING,
            "dtype": AmrDtype.SET,
            "body": build_driving_request(DrivingSet.STOP),
        })
        log.debug("[MockAMR] send_stop")

    def send_raw_cmd(self, cmd: int, dtype: int, args: dict) -> None:
        """저수준 직접 전송 — 기록만."""
        self._sent_cmds.append({"cmd": cmd, "dtype": dtype, "body": args})
        log.debug("[MockAMR] send_raw_cmd cmd=%d dtype=%d", cmd, dtype)

    def send_nav_cmd(self, coord: dict) -> None:
        """실제 구현과 동일하게 목적지 설정 후 Driving START 기록."""
        self.send_target_position(coord)
        self._sent_cmds.append({
            "cmd": AmrCmd.DRIVING,
            "dtype": AmrDtype.SET,
            "body": build_driving_request(DrivingSet.START),
        })

    @property
    def cached_valid_target_position(self) -> bool:
        return self._nav_target is not None

    @property
    def cached_robot_status(self) -> int:
        return 0 if self._moving_state == MovingState.IDLE else 3

    def send_driving_start(self) -> None:
        self._sent_cmds.append({
            "cmd": AmrCmd.DRIVING,
            "dtype": AmrDtype.SET,
            "body": build_driving_request(DrivingSet.START),
        })
        log.debug("[MockAMR] send_driving_start")

    def request_moving_info(self) -> None:
        self._sent_cmds.append({"cmd": AmrCmd.ALL_MOVING_INFO, "dtype": AmrDtype.GET, "body": {}})
        log.debug("[MockAMR] request_moving_info")

    def send_return_charging_station(self) -> None:
        self._sent_cmds.append({"cmd": AmrCmd.RETURN_CHARGING_STATION, "dtype": AmrDtype.SET, "body": {}})
        log.debug("[MockAMR] send_return_charging_station")

    def send_station_repositioning(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0) -> None:
        self._sent_cmds.append({
            "cmd": AmrCmd.STATION_REPOSITIONING,
            "dtype": AmrDtype.SET,
            "body": {"x": x, "y": y, "theta": theta},
        })
        log.debug("[MockAMR] send_station_repositioning")

    def send_bypass(
        self,
        block_areas: list[dict] | None = None,
        block_walls: list[dict] | None = None,
        charging_station: dict | None = None,
    ) -> None:
        self._sent_cmds.append({
            "cmd": AmrCmd.BYPASS,
            "dtype": AmrDtype.SET,
            "body": build_bypass_request(block_areas, block_walls, charging_station),
        })
        log.debug("[MockAMR] send_bypass")

    def send_rotation(self, rot_type: int, radian: float) -> None:
        self._sent_cmds.append({
            "cmd": AmrCmd.ROTATION,
            "dtype": AmrDtype.SET,
            "body": build_rotation_request(rot_type, radian),
        })
        log.debug("[MockAMR] send_rotation")

    def is_in_zone(self, key: str, zone_mgr: object) -> bool:
        pos = self.cached_position
        return bool(zone_mgr.is_point_in_zone(key, pos.get("x", 0.0), pos.get("y", 0.0)))

    def estimate_object_world_pos(self, distance_m: float, angle_deg: float = 0.0) -> dict:
        import math
        pos = self.cached_position
        theta = float(pos.get("theta", 0.0))
        obj_angle = theta + math.radians(angle_deg)
        return {
            "x": float(pos.get("x", 0.0)) + distance_m * math.cos(obj_angle),
            "y": float(pos.get("y", 0.0)) + distance_m * math.sin(obj_angle),
        }

    def send_manual_vw(self, ms: float, rads: float) -> None:
        """cmd=59 수동 조종 — 기록만."""
        self._sent_cmds.append({"cmd": 59, "dtype": 2, "body": {"mS": ms, "radS": rads}})
        log.debug("[MockAMR] send_manual_vw ms=%.2f rads=%.2f", ms, rads)

    def send_mapping_start(self, manual: bool = True) -> None:
        """cmd=62 맵 작성 시작 — 기록만."""
        self._sent_cmds.append({"cmd": 62, "dtype": 2, "body": {"manual": manual}})
        log.debug("[MockAMR] send_mapping_start manual=%s", manual)

    def send_mapping_stop(self) -> None:
        """cmd=62 맵 작성 정지 (set=4) — 기록만."""
        self._sent_cmds.append({"cmd": 62, "dtype": 2, "body": {"set": 4}})
        log.debug("[MockAMR] send_mapping_stop")

    def send_save_map(self) -> None:
        """cmd=87 맵 저장 — 기록만."""
        self._sent_cmds.append({"cmd": 87, "dtype": 2, "body": {}})
        log.debug("[MockAMR] send_save_map")

    def send_software_reset(self) -> None:
        """cmd=56 소프트웨어 리셋 — 기록만."""
        self._sent_cmds.append({"cmd": 56, "dtype": 2, "body": {}})
        log.debug("[MockAMR] send_software_reset")

    def request_map_data(self) -> None:
        """cmd=15 맵 데이터 요청 — 기록만. latest_map_data 는 inject_map_data() 로 주입."""
        self._sent_cmds.append({"cmd": 15, "dtype": 1, "body": {}})
        log.debug("[MockAMR] request_map_data")

    @property
    def latest_map_data(self) -> "dict | None":
        """최신 MapData 캐시. inject_map_data() 로 주입한 값을 반환."""
        return self._latest_map_data

    def pop_drive_failed(self) -> bool:
        return False

    # ── 테스트 헬퍼 ────────────────────────────────────────────────

    def inject_map_data(self, map_data: dict) -> None:
        """최신 MapData 주입 (ActionSaveMap 테스트용)."""
        self._latest_map_data = dict(map_data)
        log.debug("[MockAMR] inject_map_data: %d keys", len(map_data))

    def inject_arrived(self) -> None:
        """즉시 도착 이벤트 주입 (nav_duration 무시)."""
        self._moving_state = MovingState.IDLE
        self._arrived_flag = True
        if self._nav_target:
            self._position = dict(self._nav_target)
        log.debug("[MockAMR] inject_arrived")

    def inject_battery(self, pct: float) -> None:
        """배터리 퍼센트 강제 설정."""
        self._battery = float(pct)

    @property
    def battery_percent(self) -> float:
        return self._battery

    def get_sent_cmds(self) -> list[dict]:
        """전송 기록 복사본 반환."""
        return list(self._sent_cmds)

    def reset(self) -> None:
        """전체 상태 초기화."""
        self._moving_state = MovingState.IDLE
        self._position     = {"x": 0.0, "y": 0.0, "theta": 0.0}
        self._arrived_flag = False
        self._nav_start    = 0.0
        self._nav_target   = None
        self._battery          = self.initial_battery
        self._latest_map_data  = None
        self._sent_cmds.clear()

    # ── Private ──────────────────────────────────────────────────

    def _update_state(self) -> None:
        """MOVING 중 nav_duration 경과 시 IDLE 전환 + 도착 플래그 세팅."""
        if self._moving_state == MovingState.MOVING:
            elapsed = time.monotonic() - self._nav_start
            if elapsed >= self.nav_duration:
                self._moving_state = MovingState.IDLE
                self._arrived_flag = True
                if self._nav_target:
                    self._position = dict(self._nav_target)
                log.debug("[MockAMR] auto-arrived after %.1fs", elapsed)
