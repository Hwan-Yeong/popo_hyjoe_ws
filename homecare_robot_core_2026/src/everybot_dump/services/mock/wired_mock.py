"""
MockWiredService — OrangePi ↔ Jetson TCP JSONL Mock 구현.

inject_cmd() / inject_scenario_start() 로 수신 메시지를 시뮬레이션하고,
send() 로 전송된 메시지는 sent_messages 에 누적된다.
"""
from __future__ import annotations

import logging
from collections import deque
from dataclasses import dataclass, field

log = logging.getLogger(__name__)


@dataclass
class MockWiredService:
    """
    WiredServiceProtocol 을 만족하는 Mock 구현.

    실제 TCP 연결 없이 인메모리 큐로 동작한다.
    """
    has_client: bool = True

    # ── Private state ────────────────────────────────────────────
    _rx:           deque[dict]   = field(default_factory=deque, init=False)
    sent_messages: list[dict]    = field(default_factory=list,  init=False)

    # ── WiredServiceProtocol ─────────────────────────────────────

    def try_recv(self) -> dict | None:
        """수신 큐에서 메시지 1건 꺼내기. 없으면 None."""
        if self._rx:
            msg = self._rx.popleft()
            log.debug("[MockWired] try_recv → %s", msg.get("type"))
            return msg
        return None

    def send(self, msg: dict) -> None:
        """전송 메시지를 sent_messages 에 기록."""
        self.sent_messages.append(msg)
        log.debug("[MockWired] send type=%s", msg.get("type"))

    # ── 테스트 헬퍼 ────────────────────────────────────────────────

    def inject_cmd(self, msg: dict) -> None:
        """수신 큐에 메시지 삽입."""
        self._rx.append(msg)
        log.debug("[MockWired] inject_cmd type=%s", msg.get("type"))

    def inject_scenario_start(self, scenario_id: str,
                              params: dict | None = None) -> None:
        """시나리오 시작 명령 단축 주입."""
        self.inject_cmd({
            "type": "request_scenario_start",
            "payload": {"scenario_id": scenario_id, "params": params or {}},
        })

    def inject_scenario_stop(self) -> None:
        """시나리오 중지 명령 단축 주입."""
        self.inject_cmd({"type": "request_scenario_stop", "payload": {}})

    def clear_sent(self) -> None:
        """sent_messages 초기화."""
        self.sent_messages.clear()

    def get_last_sent(self, msg_type: str) -> dict | None:
        """sent_messages 에서 특정 type 의 마지막 메시지 반환."""
        for msg in reversed(self.sent_messages):
            if msg.get("type") == msg_type:
                return msg
        return None

    def reset(self) -> None:
        """전체 상태 초기화."""
        self._rx.clear()
        self.sent_messages.clear()
