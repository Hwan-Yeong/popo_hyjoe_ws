from __future__ import annotations

import logging
import queue
import socket
import threading
from dataclasses import dataclass, field
from typing import Any

from ..interfaces.pubsub import Publisher

log = logging.getLogger(__name__)


@dataclass
class AIService:
    """
    Jetson AI 서비스 STUB.

    AiServiceProtocol 인터페이스를 구현하여 BT 계층에서 사용 가능.
    AI Flask API 연동은 추후 구현. 현재는 빈 이벤트 큐를 반환한다.
    """

    def __post_init__(self) -> None:
        self._started  = False
        self._stop     = threading.Event()
        self._thread:  threading.Thread | None = None
        self._pub:     Publisher | None = None
        self._rx:      queue.Queue[dict] = queue.Queue()  # AI 이벤트 수신 큐

    @property
    def started(self) -> bool:
        return self._started

    def start(self) -> None:
        if self._started:
            return
        self._thread = threading.Thread(
            target=self._run, name="jetson-ai-service", daemon=True,
        )
        self._thread.start()
        self._started = True

    def tick(self) -> None:
        pass

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        self._started = False

    # ── AiServiceProtocol ───────────────────────────────────────────

    def drain_events(self) -> list[dict]:
        """
        누적된 AI 이벤트를 모두 꺼내 반환하고 내부 큐를 비운다.
        STUB: 현재는 빈 리스트 반환 (AI 연동 후 큐에 push 구현 예정).
        """
        events: list[dict] = []
        while True:
            try:
                events.append(self._rx.get_nowait())
            except queue.Empty:
                break
        return events

    def call(self, endpoint: str, method: str = "GET",
             payload: dict | None = None,
             timeout: float = 5.0) -> dict | None:
        """
        온디맨드 AI Flask API 호출.
        STUB: 현재는 None 반환 (Flask API 연동 후 requests 호출 구현 예정).
        """
        log.debug("[AIService] call STUB: %s %s", method, endpoint)
        return None

    # ── 내부 이벤트 주입 헬퍼 (테스트 / AI 모듈 연동용) ─────────────

    def inject_event(self, event: dict) -> None:
        """외부(AI 수신 스레드 등)에서 이벤트를 주입한다."""
        self._rx.put_nowait(event)

    # ── 수신 루프 (STUB) ────────────────────────────────────────────

    def _run(self) -> None:
        """AI 서비스 백그라운드 스레드 (연동 시 구현)."""
        while not self._stop.is_set():
            try:
                # TODO: Jetson AI 모듈 연동 구현
                self._stop.wait(timeout=1.0)
            except Exception as e:
                log.error("[AIService] _run error: %s", e)


# 외부에서 JetsonAiService 이름으로도 import 가능하도록 alias 추가
JetsonAiService = AIService