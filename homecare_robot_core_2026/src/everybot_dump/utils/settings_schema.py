"""
settings_change_value 페이로드 스키마.
"""
from __future__ import annotations

import logging
from typing import Any

log = logging.getLogger(__name__)


SETTINGS_SCHEMA: dict[str, dict[str, Any]] = {
    "speaker_volume": {"type": int, "range": [0, 100], "desc": "전체 스피커 볼륨 (0~100)"},
    "tts_volume": {"type": int, "range": [0, 100], "desc": "TTS 전용 볼륨 (0~100)"},
    "bgm_volume": {"type": int, "range": [0, 100], "desc": "BGM 전용 볼륨 (0~100)"},
    "arrival_wait_sec": {"type": float, "range": [0.0, 30.0], "desc": "목적지 도착 후 대기 시간 (초)"},
    "battery_threshold": {"type": float, "range": [5.0, 30.0], "desc": "배터리 위험 임계값 (%)"},
    "charge_done_threshold": {"type": float, "range": [50.0, 100.0], "desc": "충전 완료 임계값 (%)"},
    "scenario_timeout_sec": {"type": float, "range": [30.0, 600.0], "desc": "시나리오 전체 타임아웃 (초)"},
}


def validate_settings_delta(delta: dict[str, Any]) -> dict[str, Any]:
    """수신된 delta dict의 키, 타입, 범위를 검증해 유효한 값만 반환."""
    valid: dict[str, Any] = {}
    for key, value in delta.items():
        spec = SETTINGS_SCHEMA.get(key)
        if spec is None:
            log.warning("[settings] unknown key '%s' - ignored", key)
            continue
        try:
            casted = spec["type"](value)
        except (TypeError, ValueError):
            log.warning("[settings] invalid type for '%s': %r - ignored", key, value)
            continue
        lo, hi = spec["range"]
        if not (lo <= casted <= hi):
            log.warning("[settings] '%s'=%r out of range [%s, %s] - ignored", key, casted, lo, hi)
            continue
        valid[key] = casted
    return valid
