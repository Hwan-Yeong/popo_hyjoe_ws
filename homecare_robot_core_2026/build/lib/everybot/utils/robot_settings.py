"""
RobotSettingsManager — 로봇 운영 설정 파일 기반 영속화.

설정 항목:
  speaker_volume        : 스피커 음량 (0~100)
  tts_volume            : TTS 전용 음량 (0~100)
  bgm_volume            : BGM 전용 음량 (0~100)
  arrival_wait_sec      : 목적지 도착 후 대기 시간(초)
  battery_threshold     : 배터리 위험 임계값(%) — 이하 시 충전 복귀
  charge_done_threshold : 충전 완료 임계값(%) — 이상 시 운영 재개
  scenario_timeout_sec  : 시나리오 전체 타임아웃 (초)

MQTT settings/change_value → MainService → update(delta) → 파일 저장.
"""
from __future__ import annotations

import json
import logging
from dataclasses import dataclass, asdict
from pathlib import Path

log = logging.getLogger(__name__)


@dataclass
class RobotSettings:
    """로봇 운영 설정값 컨테이너."""
    speaker_volume:        int   = 50
    tts_volume:            int   = 70
    bgm_volume:            int   = 40
    arrival_wait_sec:      float = 3.0
    battery_threshold:     float = 15.0
    charge_done_threshold: float = 80.0
    scenario_timeout_sec:  float = 10000.0


class RobotSettingsManager:
    """
    JSON 파일 기반 설정 로드/저장.

    사용 예:
        mgr = RobotSettingsManager()
        mgr.load("configs/robot_settings.json")
        s = mgr.get()           # RobotSettings
        mgr.update({"speaker_volume": 70})
    """

    def __init__(self) -> None:
        self._settings = RobotSettings()
        self._path: str = ""

    # ── 로드 ────────────────────────────────────────────────────

    def load(self, path: str) -> None:
        """JSON 파일에서 설정값 로드. 파일 없으면 기본값 사용."""
        self._path = path
        p = Path(path)
        if not p.exists():
            log.info("[RobotSettings] file not found: %s — using defaults", path)
            self.save()   # 기본값으로 파일 생성
            return
        try:
            data = json.loads(p.read_text(encoding="utf-8"))
            self._apply(data)
            log.info("[RobotSettings] loaded from %s: %s", path, asdict(self._settings))
        except Exception as e:
            log.error("[RobotSettings] load failed: %s — using defaults", e)

    # ── 읽기/쓰기 ────────────────────────────────────────────────

    def get(self) -> RobotSettings:
        """현재 설정값 반환 (복사본)."""
        return RobotSettings(**asdict(self._settings))

    def update(self, delta: dict) -> None:
        """부분 업데이트 후 파일 저장. 알 수 없는 키는 무시."""
        self._apply(delta)
        self.save()
        log.info("[RobotSettings] updated: %s → saved to %s",
                 delta, self._path)

    def save(self) -> None:
        """현재 설정값을 JSON 파일로 저장."""
        if not self._path:
            return
        p = Path(self._path)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(asdict(self._settings), indent=2, ensure_ascii=False),
                     encoding="utf-8")

    def as_dict(self) -> dict:
        """MQTT publish용 dict 반환."""
        return asdict(self._settings)

    # ── Private ─────────────────────────────────────────────────

    def _apply(self, data: dict) -> None:
        s = self._settings
        if "speaker_volume" in data:
            s.speaker_volume = int(data["speaker_volume"])
        if "tts_volume" in data:
            s.tts_volume = int(data["tts_volume"])
        if "bgm_volume" in data:
            s.bgm_volume = int(data["bgm_volume"])
        if "arrival_wait_sec" in data:
            s.arrival_wait_sec = float(data["arrival_wait_sec"])
        if "battery_threshold" in data:
            s.battery_threshold = float(data["battery_threshold"])
        if "charge_done_threshold" in data:
            s.charge_done_threshold = float(data["charge_done_threshold"])
        if "scenario_timeout_sec" in data:
            s.scenario_timeout_sec = float(data["scenario_timeout_sec"])
