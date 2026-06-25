"""
ZoneManager - 금지영역/관심영역 JSON 파일 관리.
"""
from __future__ import annotations

import json
import logging
from pathlib import Path

log = logging.getLogger(__name__)


def _ray_cast(px: float, py: float, polygon: list[dict]) -> bool:
    """Ray Casting 알고리즘. 점이 polygon 내부에 있으면 True."""
    n = len(polygon)
    if n < 3:
        return False
    inside = False
    j = n - 1
    for i in range(n):
        xi = float(polygon[i].get("x", 0.0))
        yi = float(polygon[i].get("y", 0.0))
        xj = float(polygon[j].get("x", 0.0))
        yj = float(polygon[j].get("y", 0.0))
        if ((yi > py) != (yj > py)) and (px < (xj - xi) * (py - yi) / ((yj - yi) + 1e-12) + xi):
            inside = not inside
        j = i
    return inside


class ZoneManager:
    """금지영역 또는 관심영역 JSON을 로드, 저장, 조회한다."""

    def __init__(self) -> None:
        self._zones: list[dict] = []

    def load(self, path: str) -> None:
        """JSON 파일에서 zone 목록을 로드. 파일이 없으면 빈 목록."""
        src = Path(path)
        if not src.exists():
            log.info("[ZoneManager] %s not found - empty zones", path)
            self._zones = []
            return
        try:
            with src.open("r", encoding="utf-8") as fh:
                data = json.load(fh)
            zones = data.get("zones", [])
            self._zones = zones if isinstance(zones, list) else []
            log.info("[ZoneManager] loaded %d zones from %s", len(self._zones), path)
        except Exception as exc:
            log.warning("[ZoneManager] load failed %s: %s - empty zones", path, exc)
            self._zones = []

    def save(self, path: str) -> None:
        """zone 목록을 JSON 파일에 저장. 상위 디렉토리는 자동 생성."""
        dst = Path(path)
        dst.parent.mkdir(parents=True, exist_ok=True)
        with dst.open("w", encoding="utf-8") as fh:
            json.dump({"zones": self._zones}, fh, ensure_ascii=False, indent=2)
        log.info("[ZoneManager] saved %d zones to %s", len(self._zones), path)

    def add_zone(self, key: str, label: str, polygon: list[dict]) -> None:
        """동일 key가 있으면 교체, 없으면 추가."""
        self.remove_zone(key)
        self._zones.append({"key": key, "label": label, "polygon": polygon})

    def remove_zone(self, key: str) -> bool:
        """key에 해당하는 zone을 제거."""
        before = len(self._zones)
        self._zones = [zone for zone in self._zones if zone.get("key") != key]
        return len(self._zones) < before

    def list(self) -> list[dict]:
        """전체 zone 목록 반환."""
        return list(self._zones)

    def get(self, key: str) -> dict | None:
        """key에 해당하는 zone 반환."""
        return next((zone for zone in self._zones if zone.get("key") == key), None)

    def get_as_bypass_areas(self) -> list[dict]:
        """모든 zone을 AMR block_area 포맷으로 변환."""
        return [
            {
                "id": zone.get("key", ""),
                "robot_path": zone.get("polygon", []),
            }
            for zone in self._zones
        ]

    def is_point_in_zone(self, key: str, x: float, y: float) -> bool:
        """좌표가 지정 zone 내부인지 판단."""
        zone = self.get(key)
        if zone is None:
            return False
        return _ray_cast(x, y, zone.get("polygon", []))

    def find_zones_containing(self, x: float, y: float) -> list[str]:
        """좌표를 포함하는 모든 zone key 반환."""
        return [
            str(zone.get("key"))
            for zone in self._zones
            if _ray_cast(x, y, zone.get("polygon", []))
        ]
