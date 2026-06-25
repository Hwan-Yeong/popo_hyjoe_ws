"""
WaypointManager — 목적지(waypoint) JSON 파일 기반 관리.

부팅 시 load() 한 번 호출 후 get(key)/list()로 조회한다.
타입별 조회(list_by_type)와 UI 직렬화(as_ui_payload)를 지원한다.
"""
from __future__ import annotations

import json
import logging
from dataclasses import dataclass, field
from pathlib import Path

log = logging.getLogger(__name__)


@dataclass
class Waypoint:
    """단일 목적지 데이터."""
    key:     str
    x:       float
    y:       float
    theta:   float
    type:    str = "normal"   # "normal" | "dock" | "home"
    label:   str = ""         # UI 표시명 (예: "101호실")
    comment: str = ""         # 위치에 대한 설명
    bell_id: str = ""         # RF 알림벨 ID (8자리 hex, 예: "3FA17B19")
                              # BASE(3FA17B18) 이하 또는 빈 문자열 → RF 송신 skip


class WaypointManager:
    """
    JSON 파일 기반 목적지 관리자.

    사용 예:
        mgr = WaypointManager()
        mgr.load("configs/waypoints.json")
        wp = mgr.get("entrance")
    """

    def __init__(self) -> None:
        self._waypoints: dict[str, Waypoint] = {}
        self._path: str = ""

    # ── 로드 ────────────────────────────────────────────────────

    def load(self, path: str) -> None:
        """JSON 파일에서 waypoint 목록을 로드한다. 파일이 없으면 빈 파일을 생성한다."""
        self._path = path
        p = Path(path)
        if not p.exists():
            log.info("[WaypointManager] file not found: %s — creating default file", path)
            p.parent.mkdir(parents=True, exist_ok=True)
            # 'home' 기본 waypoint 포함: 모든 시나리오 완료 후 복귀 목적지.
            # 실제 충전 스테이션 좌표로 수정 후 사용할 것.
            _default = {
                "waypoints": [
                    {
                        "key":     "home",
                        "x":       0.0,
                        "y":       0.0,
                        "theta":   0.0,
                        "type":    "home",
                        "label":   "홈(충전 스테이션)",
                        "comment": "이곳은 로봇이 대기하는 위치입니다.",
                        "bell_id": "",
                    }
                ]
            }
            p.write_text(json.dumps(_default, indent=2, ensure_ascii=False),
                         encoding="utf-8")
            # home waypoint 로드
            self._waypoints = {
                "home": Waypoint(key="home", x=0.0, y=0.0, theta=0.0,
                                 type="home", label="홈(충전 스테이션)",
                                 comment="이곳은 로봇이 대기하는 위치입니다.",
                                 bell_id="")
            }
            log.info("[WaypointManager] created default waypoints (home) at %s", path)
            return
        try:
            data = json.loads(p.read_text(encoding="utf-8"))
            items = data.get("waypoints", [])
            self._waypoints = {}
            for item in items:
                key = str(item.get("key", "")).strip()
                if not key:
                    continue
                self._waypoints[key] = Waypoint(
                    key=key,
                    x=float(item.get("x", 0.0)),
                    y=float(item.get("y", 0.0)),
                    theta=float(item.get("theta", 0.0)),
                    type=str(item.get("type", "normal")),
                    label=str(item.get("label", "")),
                    comment=str(item.get("comment", "")),
                    bell_id=str(item.get("bell_id", "")),
                )
            log.info("[WaypointManager] loaded %d waypoints from %s", len(self._waypoints), path)
        except Exception as e:
            log.error("[WaypointManager] load failed: %s", e)

    # ── 조회 ────────────────────────────────────────────────────

    def get(self, key: str) -> Waypoint | None:
        """키로 waypoint 조회. 없으면 None."""
        return self._waypoints.get(key)

    def list(self) -> list[Waypoint]:
        """전체 waypoint 리스트."""
        return list(self._waypoints.values())

    def list_by_type(self, wp_type: str) -> list[Waypoint]:
        """타입별 waypoint 필터링."""
        return [w for w in self._waypoints.values() if w.type == wp_type]

    def as_dict(self) -> dict[str, dict]:
        """
        기존 ServiceBundle.waypoints 호환 dict 변환.
        key → {"x": float, "y": float, "theta": float}
        """
        return {k: {"x": w.x, "y": w.y, "theta": w.theta}
                for k, w in self._waypoints.items()}

    def as_ui_payload(self) -> list[dict]:
        """UI 전달용 직렬화 리스트."""
        return [
            {"key": w.key, "x": w.x, "y": w.y, "theta": w.theta,
             "type": w.type, "label": w.label}
            for w in self._waypoints.values()
        ]

    def file_path(self) -> str:
        return self._path

    def reload(self) -> None:
        """JSON 파일 재로드. BT bundle.waypoints가 as_dict()를 통해 즉시 반영된다."""
        if self._path:
            self.load(self._path)

    def save(self, waypoints: list[dict]) -> None:
        """
        waypoint 목록을 JSON 파일에 저장 후 reload().
        Args:
            waypoints: [{"key": str, "x": float, "y": float,
                         "theta": float, "type": str, "label": str}]
        """
        if not self._path:
            log.error("[WaypointManager] save failed: path not set")
            return
        data = {"waypoints": waypoints}
        p = Path(self._path)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
        log.info("[WaypointManager] saved %d waypoints to %s", len(waypoints), self._path)
        self.reload()
