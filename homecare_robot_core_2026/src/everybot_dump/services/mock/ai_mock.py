"""
MockAiService — AI 서비스(Flask API) Mock 구현.

inject_event() / inject_events() 로 AI 이벤트를 외부에서 주입하고,
call() 은 DEFAULT_RESPONSES 또는 override 테이블에서 응답을 반환한다.
"""
from __future__ import annotations

import logging
from collections import deque
from dataclasses import dataclass, field

log = logging.getLogger(__name__)


@dataclass
class MockAiService:
    """
    AiServiceProtocol 을 만족하는 Mock 구현.

    실제 HTTP 요청 없이 미리 설정된 응답 테이블에서 반환한다.
    """

    # 기본 응답 테이블 (엔드포인트 → 응답 dict)
    DEFAULT_RESPONSES: dict[str, dict] = field(default_factory=lambda: {
        "/api/tts/speak":          {"status": "ok", "duration_ms": 2000},
        "/api/face/recognize":     {"name": "홍길동", "unit": "101", "confidence": 0.95},
        "/api/camera/capture":     {"image_path": "/tmp/mock_photo.jpg"},
        "/api/stt/listen":         {"text": "안녕하세요", "confidence": 0.88},
        "/api/detections/latest":  {"person_count": 0, "lying_down": False},
    })

    # ── Private state ────────────────────────────────────────────
    _events:         deque[dict]       = field(default_factory=deque, init=False)
    _api_overrides:  dict[str, dict]   = field(default_factory=dict,  init=False)

    # ── AiServiceProtocol ────────────────────────────────────────

    def drain_events(self) -> list[dict]:
        """누적 이벤트 반환 후 내부 큐 비움."""
        result = list(self._events)
        self._events.clear()
        if result:
            log.debug("[MockAI] drain_events → %d events", len(result))
        return result

    def call(self, endpoint: str, method: str = "GET",
             payload: dict | None = None,
             timeout: float = 5.0) -> dict | None:
        """
        override → DEFAULT 순서로 응답 반환.
        등록되지 않은 엔드포인트는 None.
        """
        resp = self._api_overrides.get(endpoint) \
               or self.DEFAULT_RESPONSES.get(endpoint)
        log.debug("[MockAI] call %s %s → %s", method, endpoint, resp)
        return resp

    # ── 테스트 헬퍼 ────────────────────────────────────────────────

    def inject_event(self, ev: dict) -> None:
        """단건 이벤트 주입."""
        self._events.append(ev)
        log.debug("[MockAI] inject_event %s", ev.get("type"))

    def inject_events(self, evs: list[dict]) -> None:
        """복수 이벤트 주입."""
        for ev in evs:
            self._events.append(ev)
        log.debug("[MockAI] inject_events count=%d", len(evs))

    def set_api_response(self, endpoint: str, resp: dict) -> None:
        """특정 엔드포인트 응답 override."""
        self._api_overrides[endpoint] = resp

    def reset_responses(self) -> None:
        """override 초기화 (DEFAULT_RESPONSES 복원)."""
        self._api_overrides.clear()

    def reset(self) -> None:
        """전체 상태 초기화."""
        self._events.clear()
        self._api_overrides.clear()
