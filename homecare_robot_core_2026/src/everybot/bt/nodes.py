"""
bt/nodes.py — 로봇 BT에서 사용하는 모든 노드 (Condition 7종 + Action 10종).

py_trees 표준 라이프사이클:
  setup(self, **kwargs)      — 트리 setup() 호출 시 1회 (공유 리소스 초기화)
  initialise(self)           — 노드가 RUNNING 상태로 처음 진입할 때 1회
  update(self) -> Status     — RUNNING 상태에서 매 tick 호출
  terminate(self, new_status) — RUNNING에서 벗어날 때 (SUCCESS/FAILURE/INVALID) 1회

Condition 노드: initialise/terminate 없음, update()만 구현 (즉시 SUCCESS/FAILURE)
Action 노드:    필요에 따라 initialise/update/terminate 모두 구현
"""
from __future__ import annotations

import json
import logging
import re
import subprocess
import time
import urllib.error
import urllib.request
from concurrent.futures import Future, ThreadPoolExecutor
from pathlib import Path
from typing import Callable

import py_trees

from .blackboard import RobotBlackboard
from .bridge import ServiceBundle
from ..services.amr_constants import MovingState
from ..utils.event_code import EventCode

log = logging.getLogger(__name__)


def _post_tts_speak(
    bundle: ServiceBundle,
    text: str,
    timeout: float = 5.0,
    *,
    source: str = "core_sw_bt",
    interrupt: bool = False,
) -> dict:
    """Queue a dynamic TTS utterance through the local AI-TTS API."""
    clean_text = str(text or "").strip()
    if not clean_text:
        return {"status": "skipped", "reason": "empty_text"}

    base_url = getattr(bundle, "tts_api_base_url", "http://127.0.0.1:8085")
    url = f"{str(base_url).rstrip('/')}/v1/tts/speak"
    payload = {
        "text": clean_text,
        "source": source,
        "interrupt": bool(interrupt),
    }
    body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
    req = urllib.request.Request(
        url,
        data=body,
        headers={"Content-Type": "application/json"},
        method="POST",
    )

    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            raw = resp.read().decode("utf-8", errors="replace")
            if resp.status < 200 or resp.status >= 300:
                raise RuntimeError(f"TTS API returned HTTP {resp.status}: {raw}")
            if not raw.strip():
                return {"status": "queued"}
            try:
                return json.loads(raw)
            except json.JSONDecodeError:
                return {"status": "queued", "raw": raw}
    except (urllib.error.URLError, TimeoutError, RuntimeError) as exc:
        log.warning("[TTS API] speak failed url=%s text=%r: %s", url, clean_text, exc)
        raise


def _post_tts_speak_and_wait(
    bundle: ServiceBundle,
    text: str,
    timeout: float = 5.0,
    *,
    wait_until_done: bool | None = None,
    source: str = "core_sw_bt",
    interrupt: bool = False,
) -> dict:
    """Queue a TTS utterance and optionally wait for playback completion."""
    result = _post_tts_speak(
        bundle,
        text,
        timeout,
        source=source,
        interrupt=interrupt,
    )
    ai_cfg = bundle.feature_cfg.get("ai_integration", {}) if isinstance(bundle.feature_cfg, dict) else {}
    should_wait = (
        bool(ai_cfg.get("tts_wait_until_done", True))
        if wait_until_done is None
        else bool(wait_until_done)
    )
    job_id = str(result.get("job_id", "") or "")
    wait_fn = getattr(bundle.ai, "wait_tts_job_done", None)
    if should_wait and job_id and callable(wait_fn):
        result = dict(result)
        result["job_result"] = wait_fn(job_id)
    return result


def _post_mic_mute(bundle: ServiceBundle, muted: bool, timeout: float = 2.0) -> dict:
    """Set mic_manager mute state through its HTTP API."""
    endpoint = "mute" if muted else "unmute"
    url = f"{bundle.mic_base_url.rstrip('/')}/{endpoint}"
    req = urllib.request.Request(
        url,
        data=b"{}",
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        raw = resp.read().decode("utf-8", errors="replace")
        if resp.status < 200 or resp.status >= 300:
            raise RuntimeError(f"Mic API returned HTTP {resp.status}: {raw}")
        return json.loads(raw) if raw.strip() else {"ok": True, "muted": muted}


def _speaker_status(bundle: ServiceBundle, timeout: float = 1.0) -> dict:
    """Read speaker_manager status through its HTTP API."""
    url = f"{bundle.speaker_base_url.rstrip('/')}/status"
    with urllib.request.urlopen(url, timeout=timeout) as resp:
        raw = resp.read().decode("utf-8", errors="replace")
        if resp.status < 200 or resp.status >= 300:
            raise RuntimeError(f"Speaker API returned HTTP {resp.status}: {raw}")
        return json.loads(raw) if raw.strip() else {}


def _speaker_high_idle(status: dict) -> bool:
    return not status.get("playing_high") and int(status.get("queue_high", 0) or 0) <= 0


def _waypoint_candidates(key: str) -> list[str]:
    """Return compatibility aliases for legacy room/waypoint identifiers."""
    key = str(key or "").strip()
    candidates = [key]
    if key and not key.startswith("room_"):
        candidates.append(f"room_{key}")
    if key and key.endswith("호"):
        room_no = key[:-1]
        candidates.append(f"room_{room_no}")
    elif key and key.isdigit():
        candidates.append(f"{key}호")
    return list(dict.fromkeys(c for c in candidates if c))


def _resolve_waypoint_coord(bundle: ServiceBundle, key: str) -> tuple[str, dict | None]:
    """Resolve waypoint by key, common legacy aliases, or display label."""
    for candidate in _waypoint_candidates(key):
        coord = bundle.waypoints.get(candidate)
        if coord is not None:
            return candidate, coord
        if bundle.waypoint_mgr is not None:
            wp = bundle.waypoint_mgr.get(candidate)
            if wp is not None:
                return candidate, {"x": wp.x, "y": wp.y, "theta": wp.theta}

    if bundle.waypoint_mgr is not None:
        for wp in bundle.waypoint_mgr.list():
            if wp.label == key:
                return wp.key, {"x": wp.x, "y": wp.y, "theta": wp.theta}

    return key, None


# ─────────────────────────────────────────────────────────────────────────────
# Condition 노드
# update() → Status.SUCCESS or Status.FAILURE  (Status.RUNNING 반환 금지)
# ─────────────────────────────────────────────────────────────────────────────

class ConditionIdleStatus(py_trees.behaviour.Behaviour):
    """bb.idle == True 일 때 SUCCESS — Welcome mode 실행 트리거"""

    def __init__(self, bb: RobotBlackboard) -> None:
        super().__init__(name="[C] Idle State? ")
        self._bb = bb

    def update(self) -> py_trees.common.Status:
        
        if self._bb.idle_status:
            log.debug("[ConditionIdleStatus] Idle Status is True")
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE
        

class ConditionNotInitialized(py_trees.behaviour.Behaviour):
    """bb.initialized == False 일 때 SUCCESS — Init Sequence 첫 번째 gate."""

    def __init__(self, bb: RobotBlackboard) -> None:
        super().__init__(name="[C] NotInitialized?")
        self._bb = bb

    def update(self) -> py_trees.common.Status:
        return (py_trees.common.Status.SUCCESS
                if not self._bb.initialized
                else py_trees.common.Status.FAILURE)


class ConditionEmergencyStop(py_trees.behaviour.Behaviour):
    """bb.emergency_stop == True 일 때 SUCCESS — UI 긴급정지 버튼 감지."""

    def __init__(self, bb: RobotBlackboard) -> None:
        super().__init__(name="[C] EmergencyStop?")
        self._bb = bb

    def update(self) -> py_trees.common.Status:
        return (py_trees.common.Status.SUCCESS
                if self._bb.emergency_stop
                else py_trees.common.Status.FAILURE)


class ConditionMoveStopRequested(py_trees.behaviour.Behaviour):
    """UI robot/cmd/stop을 시스템 emergency가 아닌 이동 정지로 처리."""

    def __init__(self, bb: RobotBlackboard) -> None:
        super().__init__(name="[C] MoveStopRequested?")
        self._bb = bb

    def update(self) -> py_trees.common.Status:
        return (py_trees.common.Status.SUCCESS
                if self._bb.move_stop_requested
                else py_trees.common.Status.FAILURE)


class ConditionBatteryCritical(py_trees.behaviour.Behaviour):
    """bb.battery_percent <= threshold 일 때 SUCCESS.

    threshold 는 settings_mgr.get().battery_threshold 에서 동적으로 읽는다.
    settings_mgr 가 None 이면 기본값 15.0% 를 사용한다.
    """

    _DEFAULT_THRESHOLD = 15.0

    def __init__(self, name: str, bundle: ServiceBundle) -> None:
        super().__init__(name=name)
        self._bundle = bundle

    def update(self) -> py_trees.common.Status:
        threshold = self._DEFAULT_THRESHOLD
        if self._bundle.settings_mgr is not None:
            threshold = self._bundle.settings_mgr.get().battery_threshold
        pct = self._bundle.amr.battery_percent
        return (py_trees.common.Status.SUCCESS
                if pct <= threshold
                else py_trees.common.Status.FAILURE)


class ConditionScenarioActive(py_trees.behaviour.Behaviour):
    """bb.active_scenario == scenario_id 일 때 SUCCESS."""

    def __init__(self, scenario_id: str, bb: RobotBlackboard) -> None:
        super().__init__(name=f"[C] Active({scenario_id})?")
        self._scenario_id = scenario_id
        self._bb          = bb

    def update(self) -> py_trees.common.Status:
        return (py_trees.common.Status.SUCCESS
                if self._bb.active_scenario == self._scenario_id
                else py_trees.common.Status.FAILURE)


class ConditionPersonDetected(py_trees.behaviour.Behaviour):
    """bb.has_ai_event('person_detected') 일 때 SUCCESS."""

    def __init__(self, bb: RobotBlackboard) -> None:
        super().__init__(name="[C] PersonDetected?")
        self._bb = bb

    def update(self) -> py_trees.common.Status:
        return (py_trees.common.Status.SUCCESS
                if self._bb.has_ai_event("person_detected")
                else py_trees.common.Status.FAILURE)


class ConditionPersonLyingDown(py_trees.behaviour.Behaviour):
    """bb.has_ai_event('person_lying_down') 일 때 SUCCESS — 이상 감지."""

    def __init__(self, bb: RobotBlackboard) -> None:
        super().__init__(name="[C] PersonLyingDown?")
        self._bb = bb

    def update(self) -> py_trees.common.Status:
        return (py_trees.common.Status.SUCCESS
                if self._bb.has_ai_event("person_lying_down")
                else py_trees.common.Status.FAILURE)


class ConditionFaceRecognized(py_trees.behaviour.Behaviour):
    """bb.has_ai_event('face_recognized') 일 때 SUCCESS."""

    def __init__(self, bb: RobotBlackboard) -> None:
        super().__init__(name="[C] FaceRecognized?")
        self._bb = bb

    def update(self) -> py_trees.common.Status:
        return (py_trees.common.Status.SUCCESS
                if self._bb.has_ai_event("face_recognized")
                else py_trees.common.Status.FAILURE)


# ─────────────────────────────────────────────────────────────────────────────
# Action 노드 
# ─────────────────────────────────────────────────────────────────────────────

class ActionMarkInitialized(py_trees.behaviour.Behaviour):
    """
    bb.initialized = True 설정. 즉시 SUCCESS.
    Init Sequence 마지막에 위치 — 부팅 후 딱 1회만 실행.
    """

    def __init__(self, bb: RobotBlackboard) -> None:
        super().__init__(name="[A] MarkInitialized")
        self._bb = bb

    def update(self) -> py_trees.common.Status:
        self._bb.initialized = True
        log.info("[MarkInit] initialized = True")
        return py_trees.common.Status.SUCCESS


class ActionIdleStateSet(py_trees.behaviour.Behaviour):
    """
    Idle State 설정 노드. 
    Welcome 시나리오 1회 진입 원하면 True, 아닐시 False
    """

    def __init__(self, bb: RobotBlackboard, bundle: ServiceBundle,
                change_state: bool = False) -> None:
        super().__init__(name="[A] IdleStateSet" + " to {}".format(change_state))
        self._bb        = bb
        self._bundle    = bundle
        
        self.change_state = change_state

    def update(self) -> py_trees.common.Status:
        log.debug("Idle State change {}".format(self._bb.idle_status) + "to {}".format(self.change_state))
        self._bb.idle_status = self.change_state
        
        return py_trees.common.Status.SUCCESS


class ActionIdleWaiting(py_trees.behaviour.Behaviour):
    """
    시나리오 대기 상태.

    one_shot=False (기본): 항상 RUNNING. interval 초마다 request_show_menu 전송.
    one_shot=True         : initialise() 에서 1회 전송 후 즉시 SUCCESS.
                            긴급정지/배터리 복귀 완료 후 메뉴 1회 표시에 사용.
    """

    def __init__(self, bb: RobotBlackboard, bundle: ServiceBundle,
                 interval: float = 2.0, one_shot: bool = False) -> None:
        super().__init__(name="[A] IdleWaiting" + ("(1shot)" if one_shot else ""))
        self._bb        = bb
        self._bundle    = bundle
        self._interval  = interval
        self._one_shot  = one_shot
        self._last_sent: float = 0.0

    def initialise(self) -> None:
        self._last_sent = 0.0
        if self._one_shot:
            self._bundle.send_to_ui({"type": "request_show_menu", "payload": {}})
            log.debug("[IdleWaiting] one-shot request_show_menu sent")

    def update(self) -> py_trees.common.Status:
        if self._one_shot:
            return py_trees.common.Status.SUCCESS

        now = time.monotonic()
        if now - self._last_sent >= self._interval:
            self._bundle.send_to_ui({"type": "request_show_menu", "payload": {}})
            self._last_sent = now
            log.debug("[IdleWaiting] request_show_menu sent")
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None:
        pass


class ActionSleep(py_trees.behaviour.Behaviour):
    """
    Sleep 노드 대기 상태.

    sec 만큼 노드를 대기한 후 Success 한다
    """

    def __init__(self, bb: RobotBlackboard, bundle: ServiceBundle,
                 sec: float = 1.0) -> None:
        super().__init__(name="[A] Sleep" + "[{} sec]".format(sec))
        self._bb        = bb
        self._bundle    = bundle
        self._sec  = sec

    def initialise(self) -> None:
        self._start  = time.monotonic()

    def update(self) -> py_trees.common.Status:
        now = time.monotonic()
        
        if now - self._start >= self._sec:
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None:
        pass


class ActionRotation(py_trees.behaviour.Behaviour):
    """
    AMR 제자리 회전 명령 전송.

    rotation_type(int) : 0 ( diff ) 상대 각도 회전 / 1 ( pose ) 절대 각도 기준 회전 / 2 ( auto ) 자동 회전 (Lidar scan) / 3 ( left 360turn ) 360도 좌회전 / 4 ( right 360turn ) 360도 우회전
    radian(float) : 회전 각도(라디안)

    update:
      bb.amr_moving_state = 6  → SUCCESS
      bb.amr_moving_state = 5  → RUNNING
    """

    _MAX_RETRIES = 3          # 주행 불가 시 최대 재시도 횟수
    _RETRY_DELAY = 2.0        # 재시도 전 대기 시간 (초)

    def __init__(self, bb: RobotBlackboard,
                 bundle: ServiceBundle, rotation_type , radian) -> None:
        super().__init__(name=f"[A] Rotation)")
        self._bb           = bb
        self._bundle       = bundle

        self.rotation_type= rotation_type
        self.radian = radian


    def initialise(self) -> None:
        # 파라미터 읽어오는거 안됨. 디버깅 필요
        self.rotation_type = int(self._bb.scenario_params.get('rotation_type') or 0)
        self.radian = float(self._bb.scenario_params.get("radian") or 0.0)
        self._send_rotation()

    def _send_rotation(self) -> None:
        self._bundle.amr.send_rotation(self.rotation_type, self.radian)
        log.info("[AMR] → Rotation, type : %d, theta(rad) : %f", self.rotation_type, self.radian)

    def update(self) -> py_trees.common.Status:
        # 도착 이벤트
        if self._bb.amr_moving_state == 6:
            log.info("[AMR] Rotation Finished ✓")
            return py_trees.common.Status.SUCCESS

        # ----------- 예외처리 필요함 --------------------
        if self._bb.amr_moving_state == 5:
            self._send_rotation()

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None:
        self._retry_at = 0.0


class ActionNavigateTo(py_trees.behaviour.Behaviour):
    """
    AMR 이동 명령 전송 후 도착 대기.

    waypoint_key:
      - 리터럴 키  "entrance"       → bundle.waypoints["entrance"]
      - 파라미터  "{target_pos}"   → bb.scenario_params["target_pos"] 로 치환 후 조회

    update:
      bb.amr_arrived = True  → SUCCESS
      elapsed > timeout      → SUCCESS (데모용 관대한 처리)
      else                   → RUNNING
    """

    _MAX_RETRIES = 3          # 주행 불가 시 최대 재시도 횟수
    _RETRY_DELAY = 2.0        # 재시도 전 대기 시간 (초)

    def __init__(self, waypoint_key: str, bb: RobotBlackboard,
                 bundle: ServiceBundle, timeout: float = 300.0) -> None:
        super().__init__(name=f"[A] NavTo({waypoint_key})")
        self._waypoint_key = waypoint_key
        self._bb           = bb
        self._bundle       = bundle
        self._timeout      = timeout
        self._start: float = 0.0
        self._retry_count: int = 0
        self._retry_at: float  = 0.0   # 재시도 예약 시각 (0=없음)
        self._nav_sent: bool = False

    def initialise(self) -> None:
        self._start       = time.monotonic()
        self._retry_count = 0
        self._retry_at    = 0.0
        self._nav_sent    = False
        self._send_nav()

    def _send_nav(self) -> None:
        raw_key = self._resolve_key(self._waypoint_key)
        key, coord = _resolve_waypoint_coord(self._bundle, raw_key)
        if coord is None:
            self._nav_sent = False
            log.error("[NavTo] unknown waypoint '%s' — skip navigation", raw_key)
            return
        self._bundle.amr.send_target_position(coord)
        self._nav_sent = True
        log.info("[NavTo] → '%s' %s (retry=%d)", key, coord, self._retry_count)

    def update(self) -> py_trees.common.Status:
        if not self._nav_sent:
            return py_trees.common.Status.SUCCESS

        # 도착 이벤트
        if self._bb.amr_arrived:
            log.info("[NavTo] arrived ✓")
            return py_trees.common.Status.SUCCESS

        # 주행 실패 응답 수신 → 재시도 예약
        if hasattr(self._bundle.amr, "pop_drive_failed") and \
                self._bundle.amr.pop_drive_failed():
            self._retry_count += 1
            if self._retry_count <= self._MAX_RETRIES:
                log.warning("[NavTo] drive failed → retry %d/%d in %.1fs",
                            self._retry_count, self._MAX_RETRIES, self._RETRY_DELAY)
                self._retry_at = time.monotonic() + self._RETRY_DELAY
            else:
                log.error("[NavTo] drive failed %d times → proceed", self._MAX_RETRIES)
                return py_trees.common.Status.SUCCESS

        # 재시도 대기 중
        if self._retry_at > 0:
            if time.monotonic() >= self._retry_at:
                self._retry_at = 0.0
                self._send_nav()
            return py_trees.common.Status.RUNNING

        # 타임아웃
        if time.monotonic() - self._start >= self._timeout:
            log.warning("[NavTo] timeout %.1fs → proceed", self._timeout)
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None:
        self._retry_at = 0.0

    def _resolve_key(self, key: str) -> str:
        if key.startswith("{") and key.endswith("}"):
            param = key[1:-1]
            return str(self._bb.scenario_params.get(param, key))
        return key


class ActionNavigateDirectMove(py_trees.behaviour.Behaviour):
    """Execute UI cmd_move through BT and publish final move_status."""

    _MAX_RETRIES = 3
    _RETRY_DELAY = 2.0

    def __init__(self, bb: RobotBlackboard, bundle: ServiceBundle, timeout: float = 300.0) -> None:
        super().__init__(name="[A] DirectMove")
        self._bb = bb
        self._bundle = bundle
        self._timeout = timeout
        self._start = 0.0
        self._retry_count = 0
        self._retry_at = 0.0
        self._sent = False
        self._finished = False
        self._movement_seen = False

    def initialise(self) -> None:
        self._start = time.monotonic()
        self._retry_count = 0
        self._retry_at = 0.0
        self._sent = False
        self._finished = False
        self._movement_seen = False
        self._bb.amr_arrived = False
        self._send_nav()
        if self._sent:
            self._publish("moving")

    def update(self) -> py_trees.common.Status:
        if self._finished:
            return py_trees.common.Status.SUCCESS

        if not self._sent:
            self._finish("failed", "unknown_waypoint")
            return py_trees.common.Status.SUCCESS

        if self._bb.amr_moving_state == MovingState.MOVING:
            self._movement_seen = True

        if self._bb.amr_arrived and self._movement_seen:
            self._finish("arrived")
            return py_trees.common.Status.SUCCESS
        if self._bb.amr_arrived:
            log.debug("[DirectMove] ignore stale arrived before movement target=%s", self._target_id())

        if hasattr(self._bundle.amr, "pop_drive_failed") and self._bundle.amr.pop_drive_failed():
            self._retry_count += 1
            if self._retry_count <= self._MAX_RETRIES:
                self._retry_at = time.monotonic() + self._RETRY_DELAY
                log.warning(
                    "[DirectMove] drive failed -> retry %d/%d in %.1fs",
                    self._retry_count,
                    self._MAX_RETRIES,
                    self._RETRY_DELAY,
                )
            else:
                self._finish("failed", "drive_failed")
                return py_trees.common.Status.SUCCESS

        if self._retry_at > 0.0:
            if time.monotonic() >= self._retry_at:
                self._retry_at = 0.0
                self._send_nav()
            return py_trees.common.Status.RUNNING

        if time.monotonic() - self._start >= self._timeout:
            self._finish("failed", "timeout")
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None:
        self._retry_at = 0.0

    def _target_id(self) -> str:
        return str(
            self._bb.scenario_params.get("target_pos")
            or self._bb.ctx.get("cmd_move_target")
            or ""
        )

    def _send_nav(self) -> None:
        target_id = self._target_id()
        resolved_id, coord = _resolve_waypoint_coord(self._bundle, target_id)
        if coord is None:
            self._sent = False
            log.error("[DirectMove] unknown target '%s'", target_id)
            return
        self._bundle.amr.send_target_position(coord)
        self._sent = True
        self._bb.ctx["cmd_move_resolved_target"] = resolved_id
        log.info("[DirectMove] -> target=%s coord=%s retry=%d", resolved_id, coord, self._retry_count)

    def _finish(self, status: str, reason: str = "") -> None:
        self._publish(status, reason)
        self._bundle.send_to_ui({
            "type": "notify_scenario_done",
            "payload": {"scenario_id": self._bb.active_scenario},
        })
        log.info("[DirectMove] finished status=%s reason=%s", status, reason or "-")
        self._bb.active_scenario = None
        self._bb.scenario_params = {}
        self._bb.current_event_code = EventCode.NORMAL
        self._bb.clear_ctx()
        self._finished = True

    def _publish(self, status: str, reason: str = "") -> None:
        payload = {
            "command_id": self._bb.ctx.get("cmd_move_command_id", ""),
            "target_id": self._target_id(),
            "target_name": self._bb.ctx.get("cmd_move_name", ""),
            "resolved_target_id": self._bb.ctx.get("cmd_move_resolved_target", ""),
            "status": status,
            "reason": reason,
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        }
        self._bundle.send_to_ui({"type": "move_status", "payload": payload})


class ActionReturntToStation(py_trees.behaviour.Behaviour):
    """
    AMR 충전기 이동 및 연결 명령 전송 후 도착 대기.

    update:
      bb.amr_arrived = True  → SUCCESS
      else                   → RUNNING
    """

    _MAX_RETRIES = 3          # 주행 불가 시 최대 재시도 횟수
    _RETRY_DELAY = 2.0        # 재시도 전 대기 시간 (초)

    def __init__(self, bb: RobotBlackboard,
                 bundle: ServiceBundle, timeout: float = 300.0) -> None:
        super().__init__(name=f"[A] Return To Station)")
        self._bb           = bb
        self._bundle       = bundle
        self._timeout      = timeout
        self._start: float = 0.0
        self._retry_count: int = 0
        self._retry_at: float  = 0.0   # 재시도 예약 시각 (0=없음)

    def initialise(self) -> None:
        self._start       = time.monotonic()
        self._retry_count = 0
        self._retry_at    = 0.0
        self._send_docking()

    def _send_docking(self) -> None:
        self._bundle.amr.send_return_charging_station()
        log.info("[Charging] → Return to Station ")

    def update(self) -> py_trees.common.Status:
        # 도착 이벤트
        if self._bb.amr_robot_state == 7:
            log.info("[Charging] arrived ✓")
            return py_trees.common.Status.SUCCESS

        # ----------- 예외처리 필요함 --------------------
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None:
        self._retry_at = 0.0



class ActionSpeak(py_trees.behaviour.Behaviour):
    """
    TTS 발화 — local AI-TTS API /v1/tts/speak 를 별도 스레드에서 비동기 실행.

    text_template 치환 규칙 (re 기반):
      "{ctx_key}"    → bb.ctx 에서 우선 조회
      "{param_key}"  → bb.scenario_params 에서 조회
      키가 없으면 원문 유지

    update:
      future 완료      → SUCCESS
      elapsed > timeout → SUCCESS (발화 실패해도 다음 스텝 진행)
    """

    def __init__(self, text_template: str, bb: RobotBlackboard,
                 bundle: ServiceBundle, timeout: float = 3.0,
                 wait_until_done: bool = True) -> None:
        label = text_template[:18] + "…" if len(text_template) > 18 else text_template
        super().__init__(name=f"[A] Speak({label})")
        self._template = text_template
        self._bb       = bb
        self._bundle   = bundle
        self._timeout  = timeout
        self._wait_until_done = wait_until_done
        self._executor: ThreadPoolExecutor | None = None
        self._future:   Future | None = None
        self._start: float = 0.0

    def initialise(self) -> None:
        self._start    = time.monotonic()
        text           = self._resolve(self._template)
        self._executor = ThreadPoolExecutor(max_workers=1,
                                            thread_name_prefix="speak")
        self._future   = self._executor.submit(
            _post_tts_speak_and_wait,
            self._bundle,
            text,
            self._timeout,
            wait_until_done=self._wait_until_done,
        )
        log.info("[Speak] → '%s'", text)

    def update(self) -> py_trees.common.Status:
        if self._future is not None and self._future.done():
            try:
                result = self._future.result()
                job_result = result.get("job_result", {}) if isinstance(result, dict) else {}
                if isinstance(job_result, dict) and job_result.get("status") not in (None, "", "done", "skipped"):
                    log.warning("[Speak] TTS job ended with status=%s", job_result.get("status"))
            except Exception:
                log.warning("[Speak] TTS API failed → proceed", exc_info=True)
            return py_trees.common.Status.SUCCESS
        if time.monotonic() - self._start >= self._effective_timeout():
            log.warning("[Speak] timeout → proceed")
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None:
        if self._executor is not None:
            self._executor.shutdown(wait=False)
            self._executor = None
        self._future = None

    def _resolve(self, template: str) -> str:
        def sub(m: re.Match) -> str:
            k = m.group(1)
            if k in self._bb.ctx:
                v = self._bb.ctx.get(k)
            else:
                v = self._bb.scenario_params.get(k)
            return str(v) if v is not None else m.group(0)
        return re.sub(r"\{(\w+)\}", sub, template)

    def _effective_timeout(self) -> float:
        ai_cfg = self._bundle.feature_cfg.get("ai_integration", {}) if isinstance(self._bundle.feature_cfg, dict) else {}
        should_wait = (
            bool(ai_cfg.get("tts_wait_until_done", True))
            if self._wait_until_done is None
            else bool(self._wait_until_done)
        )
        if not should_wait:
            return self._timeout
        done_timeout = float(ai_cfg.get("tts_done_timeout_sec", 20.0) or 20.0)
        return max(self._timeout, self._timeout + done_timeout + 1.0)


class ActionSpeakFile(py_trees.behaviour.Behaviour):
    """
    speaker_manager API를 통해 TTS 파일을 재생하는 BT 액션 노드.
    """

    _HTTP_TIMEOUT = 10.0

    def __init__(
        self,
        file_path: str,
        bundle: ServiceBundle,
        blocking: bool = True,
    ) -> None:
        label = file_path[:18] + "…" if len(file_path) > 18 else file_path
        super().__init__(name=f"[A] Speak({label})")
        self._file_path = file_path
        self._bundle = bundle
        self._blocking  = blocking
        self._executor: ThreadPoolExecutor | None = None
        self._future: Future | None = None

    def initialise(self) -> None:
        self._future = None
        p = Path(self._file_path)
        if not p.exists():
            log.warning("[SpeakFile] file not found: %s", self._file_path)
            return
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="speak-file")
        self._future = self._executor.submit(self._call_speak)
        log.info("[SpeakFile] playing: %s (blocking=%s)", p.name, self._blocking)

    def update(self) -> py_trees.common.Status:
        if self._future is None:
            return py_trees.common.Status.SUCCESS
        if not self._blocking:
            return py_trees.common.Status.SUCCESS
        if not self._future.done():
            return py_trees.common.Status.RUNNING
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: py_trees.common.Status) -> None:
        if self._executor is not None:
            self._executor.shutdown(wait=False)
            self._executor = None
        self._future = None

    def _call_speak(self) -> None:
        try:
            import requests

            requests.post(
                f"{self._bundle.speaker_base_url}/play",
                json={"file": self._file_path, "type": "tts", "loop": False},
                timeout=self._HTTP_TIMEOUT,
            )
        except Exception as exc:
            log.warning("[SpeakFile] speaker_manager 호출 실패: %s", exc)


class ActionWaitPersonDetected(py_trees.behaviour.Behaviour):
    """
    'person_detected' AI 이벤트 대기.

    update:
      이벤트 있음           → ctx['detected_person'] 저장 → SUCCESS
      timeout + skip=True   → SUCCESS (skip 처리, ctx 미저장)
      timeout + skip=False  → FAILURE
      else                  → RUNNING
    """

    def __init__(self, bb: RobotBlackboard,
                 timeout: float = 15.0,
                 skip_on_timeout: bool = True) -> None:
        super().__init__(name="[A] WaitPerson")
        self._bb             = bb
        self._timeout        = timeout
        self._skip_on_timeout = skip_on_timeout
        self._start: float   = 0.0

    def initialise(self) -> None:
        self._start = time.monotonic()
        log.debug("[WaitPerson] start (timeout=%.1fs)", self._timeout)

    def update(self) -> py_trees.common.Status:
        ev = self._bb.get_ai_event("person_detected")
        if ev is not None:
            self._bb.ctx["detected_person"] = ev
            log.info("[WaitPerson] detected conf=%.2f", ev.get("confidence", 0.0))
            return py_trees.common.Status.SUCCESS

        if time.monotonic() - self._start >= self._timeout:
            if self._skip_on_timeout:
                log.debug("[WaitPerson] timeout → skip")
                return py_trees.common.Status.SUCCESS
            log.debug("[WaitPerson] timeout → FAILURE")
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None:
        pass


class ActionFaceRecognize(py_trees.behaviour.Behaviour):
    """
    Face API 최신 snapshot 조회 비동기.

    update:
      name 있음     → ctx['recognized_name'], ctx['recognized_unit'] → SUCCESS
      name 없음     → FAILURE
      미완료        → RUNNING
      timeout       → FAILURE
    """

    def __init__(self, bb: RobotBlackboard, bundle: ServiceBundle,
                 timeout: float = 8.0) -> None:
        super().__init__(name="[A] FaceRecognize")
        self._bb      = bb
        self._bundle  = bundle
        self._timeout = timeout
        self._executor: ThreadPoolExecutor | None = None
        self._future:   Future | None = None
        self._start: float = 0.0

    def initialise(self) -> None:
        self._start    = time.monotonic()
        self._executor = ThreadPoolExecutor(max_workers=1,
                                            thread_name_prefix="face")
        self._future   = self._executor.submit(self._bundle.ai.recognize_face)
        log.debug("[FaceRecog] started")

    def update(self) -> py_trees.common.Status:
        if self._future is not None and self._future.done():
            resp = self._future.result()
            if resp and (resp.get("recognized") or resp.get("name")):
                self._bb.ctx["recognized_name"] = resp["name"]
                self._bb.ctx["recognized_unit"] = resp.get("unit", "")
                self._bb.ctx["face_confidence"] = float(resp.get("confidence", 0.0) or 0.0)
                self._bb.ctx["face_summary_text"] = str(resp.get("summary_text", "") or "")
                self._bb.ctx["face_source"] = str(resp.get("source", "") or "")
                log.info("[FaceRecog] → %s / unit=%s",
                         resp["name"], resp.get("unit"))
                return py_trees.common.Status.SUCCESS
            self._bb.ctx["face_unknown_count"] = int((resp or {}).get("unknown_count", 0) or 0)
            self._bb.ctx["face_stale"] = bool((resp or {}).get("stale", False))
            log.debug("[FaceRecog] no match → FAILURE")
            return py_trees.common.Status.FAILURE

        if time.monotonic() - self._start >= self._timeout:
            log.warning("[FaceRecog] timeout → FAILURE")
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None:
        if self._executor is not None:
            self._executor.shutdown(wait=False)
            self._executor = None
        self._future = None


class ActionPhotoCapture(py_trees.behaviour.Behaviour):
    """
    camera_manager snapshot API 호출 비동기.

    update:
      image_path 있음 → ctx['photo_path'] 저장 → SUCCESS
      없음 / timeout  → FAILURE
    """

    def __init__(self, bb: RobotBlackboard, bundle: ServiceBundle,
                 timeout: float = 5.0) -> None:
        super().__init__(name="[A] PhotoCapture")
        self._bb      = bb
        self._bundle  = bundle
        self._timeout = timeout
        self._executor: ThreadPoolExecutor | None = None
        self._future:   Future | None = None
        self._start: float = 0.0

    def initialise(self) -> None:
        self._start    = time.monotonic()
        self._executor = ThreadPoolExecutor(max_workers=1,
                                            thread_name_prefix="photo")
        self._future   = self._executor.submit(self._call_capture)
        log.debug("[Photo] capture started")

    def update(self) -> py_trees.common.Status:
        if self._future is not None and self._future.done():
            resp = self._future.result()
            if resp and resp.get("image_path"):
                self._bb.ctx["photo_path"] = resp["image_path"]
                log.info("[Photo] → %s", resp["image_path"])
                return py_trees.common.Status.SUCCESS
            log.warning("[Photo] capture failed → FAILURE")
            return py_trees.common.Status.FAILURE

        if time.monotonic() - self._start >= self._timeout:
            log.warning("[Photo] timeout → FAILURE")
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None:
        if self._executor is not None:
            self._executor.shutdown(wait=False)
            self._executor = None
        self._future = None

    def _call_capture(self) -> dict | None:
        try:
            import requests

            r = requests.get(
                f"{self._bundle.camera_base_url}/snapshot/color",
                timeout=self._timeout,
            )
            r.raise_for_status()
            content_type = (r.headers.get("Content-Type") or "").lower()
            if "application/json" in content_type:
                data = r.json()
                return {"image_path": data.get("image_path")} if data.get("image_path") else None

            save_dir = Path("/dev/shm/hw_data/camera")
            save_dir.mkdir(parents=True, exist_ok=True)
            save_path = save_dir / f"capture_{int(time.time() * 1000)}.jpg"
            save_path.write_bytes(r.content)
            return {"image_path": str(save_path)}
        except Exception as exc:
            log.warning("[PhotoCapture] camera_manager 호출 실패: %s", exc)
            return None


class ActionNotifyWired(py_trees.behaviour.Behaviour):
    """
    OrangePi 로 wired 메시지 전송. 즉시 SUCCESS.

    payload:
      dict 리터럴   → 그대로 전송
      Callable[[], dict] → 호출 결과를 전송 (bb.ctx 참조용 closure)
    """

    def __init__(self, msg_type: str,
                 payload: dict | Callable[[], dict],
                 bundle: ServiceBundle) -> None:
        super().__init__(name=f"[A] Notify({msg_type})")
        self._msg_type = msg_type
        self._payload  = payload
        self._bundle   = bundle

    def update(self) -> py_trees.common.Status:
        body = self._payload() if callable(self._payload) else self._payload
        self._bundle.send_to_ui({"type": self._msg_type, "payload": body})
        log.info("[Notify] → %s %s", self._msg_type, body)
        return py_trees.common.Status.SUCCESS


class ActionAmrStop(py_trees.behaviour.Behaviour):
    """
    AMR 즉시 정지 명령. 즉시 SUCCESS.
    Emergency / Battery 인터럽트 진입 시 첫 번째로 실행.
    """

    def __init__(self, bundle: ServiceBundle) -> None:
        super().__init__(name="[A] AmrStop")
        self._bundle = bundle

    def update(self) -> py_trees.common.Status:
        self._bundle.amr.send_stop()
        log.info("[AmrStop] stop sent")
        return py_trees.common.Status.SUCCESS


class ActionHandleMoveStop(py_trees.behaviour.Behaviour):
    """Stop the active scenario without entering emergency release wait."""

    def __init__(self, bb: RobotBlackboard, bundle: ServiceBundle) -> None:
        super().__init__(name="[A] HandleMoveStop")
        self._bb = bb
        self._bundle = bundle

    def update(self) -> py_trees.common.Status:
        reason = str(self._bb.ctx.get("stop_reason", "move_stop") or "move_stop")
        target_id = self._target_id()

        self._bundle.amr.send_stop()
        self._stop_audio()
        self._mute_mic()
        if target_id:
            self._bundle.send_to_ui({
                "type": "move_status",
                "payload": {
                    "target_id": target_id,
                    "status": "cancelled",
                    "reason": reason,
                    "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                },
            })
        self._bundle.send_to_ui({
            "type": "notify_scenario_done",
            "payload": {"scenario_id": self._bb.active_scenario},
        })

        log.info("[MoveStop] cancelled scenario=%s target=%s reason=%s",
                 self._bb.active_scenario, target_id or "-", reason)
        self._bb.move_stop_requested = False
        self._bb.active_scenario = None
        self._bb.scenario_params = {}
        self._bb.current_event_code = EventCode.NORMAL
        self._bb.voice_agent_intent = ""
        self._bb.voice_agent_response = ""
        self._bb.voice_agent_action = {}
        self._bb.clear_ctx()
        return py_trees.common.Status.SUCCESS

    def _stop_audio(self) -> None:
        try:
            body = json.dumps({"type": "all"}).encode("utf-8")
            req = urllib.request.Request(
                f"{self._bundle.speaker_base_url}/stop",
                data=body,
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            with urllib.request.urlopen(req, timeout=1.0):
                pass
            log.info("[MoveStop] speaker stop all sent")
        except Exception as exc:
            log.warning("[MoveStop] speaker stop failed: %s", exc)

    def _mute_mic(self) -> None:
        try:
            _post_mic_mute(self._bundle, True, 1.0)
            log.info("[MoveStop] mic mute sent")
        except Exception as exc:
            log.warning("[MoveStop] mic mute failed: %s", exc)

    def _target_id(self) -> str:
        ctx_target = self._bb.ctx.get("cmd_move_target") or self._bb.ctx.get("cmd_move_resolved_target")
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

        return ""


class ActionScenarioDone(py_trees.behaviour.Behaviour):
    """
    시나리오 완료 처리. 즉시 SUCCESS.
      1. wired: 'notify_scenario_done' 전송
      2. bb.active_scenario = None  (→ IdleWaiting 복귀)
      3. bb.clear_ctx()
    """

    def __init__(self, bb: RobotBlackboard, bundle: ServiceBundle) -> None:
        super().__init__(name="[A] ScenarioDone")
        self._bb     = bb
        self._bundle = bundle

    def update(self) -> py_trees.common.Status:
        self._bundle.send_to_ui({
            "type":    "notify_scenario_done",
            "payload": {"scenario_id": self._bb.active_scenario},
        })
        log.info("[ScenarioDone] scenario=%s → Idle", self._bb.active_scenario)
        self._bb.active_scenario = None
        self._bb.scenario_params = {}
        self._bb.current_event_code = EventCode.NORMAL
        self._bb.voice_agent_intent = ""
        self._bb.voice_agent_response = ""
        self._bb.voice_agent_action = {}
        self._bb.clear_ctx()
        
        return py_trees.common.Status.SUCCESS


# =============================================================================
# v2 신규 노드
# =============================================================================

# ─────────────────────────────────────────────────────────────────────────────
# Condition 노드
# ─────────────────────────────────────────────────────────────────────────────

class ConditionMapNotReady(py_trees.behaviour.Behaviour):
    """bb.map_ready == False 일 때 SUCCESS — 맵이 없으면 맵 생성 서브시나리오 진입."""

    def __init__(self, bb: RobotBlackboard) -> None:
        super().__init__(name="[C] MapNotReady?")
        self._bb = bb

    def update(self) -> py_trees.common.Status:
        return (py_trees.common.Status.SUCCESS
                if not self._bb.map_ready
                else py_trees.common.Status.FAILURE)


class ConditionMapReady(py_trees.behaviour.Behaviour):
    """bb.map_ready == True 일 때 SUCCESS — Selector 통과 조건."""

    def __init__(self, bb: RobotBlackboard) -> None:
        super().__init__(name="[C] MapReady?")
        self._bb = bb

    def update(self) -> py_trees.common.Status:
        return (py_trees.common.Status.SUCCESS
                if self._bb.map_ready
                else py_trees.common.Status.FAILURE)


class ConditionWifiNotRegistered(py_trees.behaviour.Behaviour):
    """bb.wifi_registered == False 일 때 SUCCESS — WiFi 미등록 시 등록 진입."""

    def __init__(self, bb: RobotBlackboard) -> None:
        super().__init__(name="[C] WifiNotReg?")
        self._bb = bb

    def update(self) -> py_trees.common.Status:
        return (py_trees.common.Status.SUCCESS
                if not self._bb.wifi_registered
                else py_trees.common.Status.FAILURE)


class ConditionWifiRegistered(py_trees.behaviour.Behaviour):
    """bb.wifi_registered == True 일 때 SUCCESS — Selector 통과 조건."""

    def __init__(self, bb: RobotBlackboard) -> None:
        super().__init__(name="[C] WifiReg?")
        self._bb = bb

    def update(self) -> py_trees.common.Status:
        return (py_trees.common.Status.SUCCESS
                if self._bb.wifi_registered
                else py_trees.common.Status.FAILURE)


class ConditionForceSoftAP(py_trees.behaviour.Behaviour):
    """bb.force_softap == True 일 때 SUCCESS — Force SoftAP Sequence gate."""

    def __init__(self, bb: RobotBlackboard) -> None:
        super().__init__(name="[C] ForceSoftAP?")
        self._bb = bb

    def update(self) -> py_trees.common.Status:
        return (py_trees.common.Status.SUCCESS
                if self._bb.force_softap
                else py_trees.common.Status.FAILURE)


class ConditionFactoryReset(py_trees.behaviour.Behaviour):
    """bb.factory_reset == True 일 때 SUCCESS."""

    def __init__(self, bb: RobotBlackboard) -> None:
        super().__init__(name="[C] FactoryReset?")
        self._bb = bb

    def update(self) -> py_trees.common.Status:
        return (py_trees.common.Status.SUCCESS
                if self._bb.factory_reset
                else py_trees.common.Status.FAILURE)


class ConditionBatteryReady(py_trees.behaviour.Behaviour):
    """bb.battery_percent >= charge_done_threshold 일 때 SUCCESS — 충전 완료 판단."""

    def __init__(self, bb: RobotBlackboard, bundle: ServiceBundle) -> None:
        super().__init__(name="[C] BatteryReady?")
        self._bb     = bb
        self._bundle = bundle

    def update(self) -> py_trees.common.Status:
        threshold = 80.0
        if self._bundle.settings_mgr is not None:
            threshold = self._bundle.settings_mgr.get().charge_done_threshold
        return (py_trees.common.Status.SUCCESS
                if self._bb.battery_percent >= threshold
                else py_trees.common.Status.FAILURE)


# ─────────────────────────────────────────────────────────────────────────────
# v2 Action 노드
# ─────────────────────────────────────────────────────────────────────────────

class ActionWifiProvision(py_trees.behaviour.Behaviour):
    """
    WiFi 프로비저닝 완료 대기 노드.

    실제 프로비저닝은 MainService._handle_mobile()이 처리:
      OrangePi → request_robot_regist → SoftAP 활성화
      Mobile App → provision_start + provision → nmcli connect → state.json 저장

    Bridge.update()에서 wifi_reg_fn()을 매 tick 호출하여
    bb.wifi_registered = True 갱신 → 다음 tick에 SUCCESS 반환.
    """

    def __init__(self, bb: RobotBlackboard) -> None:
        super().__init__(name="[A] WifiProvision")
        self._bb = bb

    def initialise(self) -> None:
        log.info("[WifiProvision] waiting for WiFi registration...")

    def update(self) -> py_trees.common.Status:
        if self._bb.wifi_registered:
            log.info("[WifiProvision] WiFi registered → SUCCESS")
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


class ActionForceSoftAP(py_trees.behaviour.Behaviour):
    """
    Force SoftAP 활성화 → 대기 → 해제 후 home WiFi 복귀.

    initialise: SoftApManager.enable() → 192.168.0.1 활성화
    update:     bb.force_softap == False → SUCCESS
    terminate:  SoftApManager.disable() + WifiManager.reconnect()

    WiFi 칩 1개 → SoftAP 중 home WiFi 연결 끊김 (의도된 동작).
    해제 시 WifiManager.reconnect()로 기존 nmcli 프로파일 기반 재연결.
    프로파일 미존재 시 WARNING만 기록.
    """

    def __init__(self, bb: RobotBlackboard, bundle: ServiceBundle) -> None:
        super().__init__(name="[A] ForceSoftAP")
        self._bb     = bb
        self._bundle = bundle

    def initialise(self) -> None:
        if self._bundle.softap is None:
            log.warning("[ForceSoftAP] softap not in bundle — clearing flag")
            self._bb.force_softap = False
            return
        try:
            self._bundle.softap.enable()
            log.info("[ForceSoftAP] SoftAP enabled (192.168.0.1)")
        except Exception as e:
            log.error("[ForceSoftAP] enable failed: %s — clearing flag", e)
            self._bb.force_softap = False

    def update(self) -> py_trees.common.Status:
        if not self._bb.force_softap:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None:
        if self._bundle.softap is not None:
            try:
                self._bundle.softap.disable()
                log.info("[ForceSoftAP] SoftAP disabled")
            except Exception as e:
                log.warning("[ForceSoftAP] disable failed: %s", e)
        if self._bundle.wifi is not None:
            try:
                self._bundle.wifi.reconnect()
                log.info("[ForceSoftAP] home WiFi reconnected")
            except Exception as e:
                log.warning("[ForceSoftAP] wifi reconnect failed (no profile?): %s", e)


class ActionWaitEmergencyRelease(py_trees.behaviour.Behaviour):
    """
    bb.emergency_stop == False 될 때까지 RUNNING.
    긴급정지 Sequence 내에서 해제 대기용.
    """

    def __init__(self, bb: RobotBlackboard) -> None:
        super().__init__(name="[A] WaitEmgRelease")
        self._bb = bb

    def update(self) -> py_trees.common.Status:
        if not self._bb.emergency_stop:
            log.info("[WaitEmgRelease] emergency released → SUCCESS")
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


class ActionWaitBatteryReady(py_trees.behaviour.Behaviour):
    """
    bb.battery_percent >= charge_done_threshold 될 때까지 RUNNING.
    매 20틱(1초)마다 notify_battery_status를 UI에 전송.
    """
    _NOTIFY_INTERVAL = 20   # 20Hz * 20 = 1초

    def __init__(self, bb: RobotBlackboard, bundle: ServiceBundle) -> None:
        super().__init__(name="[A] WaitBattReady")
        self._bb        = bb
        self._bundle    = bundle
        self._tick_cnt  = 0

    def initialise(self) -> None:
        self._tick_cnt = 0

    def update(self) -> py_trees.common.Status:
        self._tick_cnt += 1
        if self._tick_cnt % self._NOTIFY_INTERVAL == 0:
            self._bundle.send_to_ui({
                "type":    "notify_battery_status",
                "payload": {"percent": self._bb.battery_percent},
            })

        threshold = 80.0
        if self._bundle.settings_mgr is not None:
            threshold = self._bundle.settings_mgr.get().charge_done_threshold

        if self._bb.battery_percent >= threshold:
            log.info("[WaitBattReady] battery=%.1f%% >= %.1f%% → SUCCESS",
                     self._bb.battery_percent, threshold)
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


class ActionFactoryReset(py_trees.behaviour.Behaviour):
    """
    공장초기화: 맵 파일, waypoints 파일, WiFi 자격증명 삭제 + BB 상태 초기화
                + AMR 소프트웨어 리셋.

    AMR SW 리셋(cmd=56): AMR 보드 재부팅 → 활성 맵 메모리 초기화.
    즉시 SUCCESS (AMR 재부팅 완료 대기는 이후 init 흐름에서 자연스럽게 처리됨).
    """

    def __init__(self, bb: RobotBlackboard, bundle: ServiceBundle,
                 map_pgm_path: str, map_yaml_path: str,
                 waypoints_path: str) -> None:
        super().__init__(name="[A] FactoryReset")
        self._bb             = bb
        self._bundle         = bundle
        self._map_pgm        = map_pgm_path
        self._map_yaml       = map_yaml_path
        self._waypoints_path = waypoints_path

    def update(self) -> py_trees.common.Status:
        import os
        for p in (self._map_pgm, self._map_yaml, self._waypoints_path):
            try:
                if os.path.exists(p):
                    os.remove(p)
                    log.info("[FactoryReset] deleted: %s", p)
            except Exception as e:
                log.warning("[FactoryReset] delete failed %s: %s", p, e)

        # AMR 소프트웨어 리셋 — AMR 보드 재부팅으로 활성 맵 메모리 초기화
        try:
            self._bundle.amr.send_software_reset()
            log.info("[FactoryReset] AMR software reset sent (cmd=56)")
        except Exception as e:
            log.warning("[FactoryReset] AMR reset failed: %s", e)

        # BB 초기화
        self._bb.factory_reset   = False
        self._bb.initialized     = False
        self._bb.map_ready       = False
        self._bb.wifi_registered = False
        self._bb.active_scenario = None
        self._bb.clear_ctx()

        log.warning("[FactoryReset] completed — re-entering init (AMR rebooting ~10s)")
        return py_trees.common.Status.SUCCESS


class ActionStartMapCreationServer(py_trees.behaviour.Behaviour):
    """
    맵 생성 웹서버 기동. 실제 맵 생성 시작은 웹 UI /api/start_mapping에서 수행.
    """

    def __init__(self, bb: RobotBlackboard, bundle: ServiceBundle,
                 port: int = 8080,
                 map_pgm_path: str = "configs/map/map.pgm",
                 waypoints_path: str = "configs/waypoints.json",
                 forbidden_zones_path: str = "configs/map/forbidden_zones.json",
                 roi_zones_path: str = "configs/map/roi_zones.json") -> None:
        super().__init__(name="[A] StartMapServer")
        self._bb     = bb
        self._bundle = bundle
        self._port   = port
        self._map_pgm_path = map_pgm_path
        self._waypoints_path = waypoints_path
        self._forbidden_zones_path = forbidden_zones_path
        self._roi_zones_path = roi_zones_path
        self._server = None

    def initialise(self) -> None:
        # bb.map_creation_done 초기화
        self._bb.map_creation_done = False
        self._bb.latest_map_data   = {}

    def update(self) -> py_trees.common.Status:
        # 서버는 lazy import (순환 방지)
        from ..services.map_creation_server import MapCreationServer
        self._server = MapCreationServer(
            self._bb, self._bundle, port=self._port,
            waypoint_mgr=self._bundle.waypoint_mgr,  # Fix: waypoint_mgr 전달
            map_pgm_path=self._map_pgm_path,
            waypoints_path=self._waypoints_path,
            forbidden_zones_path=self._forbidden_zones_path,
            roi_zones_path=self._roi_zones_path,
        )
        self._server.start()
        self._bb.ctx["map_creation_server"] = self._server  # Fix: StopMapServer가 참조할 수 있도록 저장

        # 맵 생성 시작(cmd=62)은 BT에서 자동 전송하지 않음.
        # 사용자가 충전기에서 이동 후 웹 UI의 [▶ 맵 생성 시작] 버튼 → /api/start_mapping 에서 전송.
        self._bundle.send_to_ui({
            "type":    "notify_map_creation_started",
            "payload": {"port": self._port},
        })
        log.info("[StartMapServer] server started on port %d — waiting for user to start mapping", self._port)
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: py_trees.common.Status) -> None:
        pass


class ActionWaitMapCreationDone(py_trees.behaviour.Behaviour):
    """
    bb.map_creation_done == True 될 때까지 RUNNING.
    MapCreationServer Phase 5(ROI 저장) 완료 시 bb.map_creation_done=True.
    """

    def __init__(self, bb: RobotBlackboard) -> None:
        super().__init__(name="[A] WaitMapDone")
        self._bb = bb

    def update(self) -> py_trees.common.Status:
        if self._bb.map_creation_done:
            log.info("[WaitMapDone] map creation done signal received")
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


class ActionSaveMap(py_trees.behaviour.Behaviour):
    """
    ① AMR cmd=87(SaveMap) 전송
    ② AMR cmd=15(MapData) 요청
    ③ bb.latest_map_data 수신 대기 (최대 timeout 초)
    ④ base64_map_convert_to_file() 로 PGM/YAML 저장
    ⑤ bb.map_ready = True
    """
    _STATE_SAVE     = "save"
    _STATE_REQUEST  = "request"
    _STATE_WAIT     = "wait"
    _STATE_DONE     = "done"

    def __init__(self, bb: RobotBlackboard, bundle: ServiceBundle,
                 map_pgm_path: str, timeout: float = 15.0) -> None:
        super().__init__(name="[A] SaveMap")
        self._bb          = bb
        self._bundle      = bundle
        self._map_pgm     = map_pgm_path
        self._timeout     = timeout
        self._state       = self._STATE_SAVE
        self._start: float = 0.0

    def initialise(self) -> None:
        self._state = self._STATE_SAVE
        self._start = time.monotonic()
        self._bb.latest_map_data = {}

    def update(self) -> py_trees.common.Status:
        if self._state == self._STATE_SAVE:
            # ① SaveMap 전송
            self._bundle.amr.send_raw_cmd(87, 2,
                {"Request": {"Set": {"SaveMap": {}}}})
            log.info("[SaveMap] cmd=87 SaveMap sent")
            self._state = self._STATE_REQUEST

        elif self._state == self._STATE_REQUEST:
            # ② MapData 요청
            self._bundle.amr.send_raw_cmd(15, 1,
                {"Request": {"Get": {"MapData": {}}}})
            log.info("[SaveMap] cmd=15 MapData requested")
            self._state = self._STATE_WAIT

        elif self._state == self._STATE_WAIT:
            # ③ 수신 대기
            if self._bb.latest_map_data:
                log.info("[SaveMap] MapData received — saving to file")
                self._write_map(self._bb.latest_map_data)
                self._bb.map_ready = True
                self._bb.map_creation_done = False
                self._state = self._STATE_DONE
                return py_trees.common.Status.SUCCESS

            if time.monotonic() - self._start >= self._timeout:
                log.error("[SaveMap] timeout %.1fs — FAILURE", self._timeout)
                return py_trees.common.Status.FAILURE

        elif self._state == self._STATE_DONE:
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING

    def _write_map(self, map_data_dict: dict) -> None:
        """base64_map_convert_to_file 로직을 map_name 경로로 저장."""
        import base64
        import os
        try:
            import yaml as _yaml
        except ImportError:
            log.error("[SaveMap] PyYAML not installed — cannot write YAML")
            return

        map_name = self._map_pgm[:-4] if self._map_pgm.endswith(".pgm") else self._map_pgm
        os.makedirs(os.path.dirname(map_name) or ".", exist_ok=True)

        width      = int(map_data_dict["width"])
        height     = int(map_data_dict["height"])
        resolution = map_data_dict["resolution"]
        origin     = [map_data_dict["posX"], map_data_dict["posY"], 0.0]
        raw_data   = base64.b64decode(map_data_dict["data"])

        header = f"P5\n{width} {height}\n255\n".encode("ascii")
        with open(f"{map_name}.pgm", "wb") as f:
            f.write(header)
            f.write(raw_data)

        yaml_data = {
            "image":           f"{os.path.basename(map_name)}.pgm",
            "resolution":      resolution,
            "origin":          origin,
            "negate":          0,
            "occupied_thresh": 0.65,
            "free_thresh":     0.25,
        }
        with open(f"{map_name}.yaml", "w") as f:
            _yaml.dump(yaml_data, f, default_flow_style=False)

        log.info("[SaveMap] saved: %s.pgm + %s.yaml", map_name, map_name)


class ActionStopMapCreationServer(py_trees.behaviour.Behaviour):
    """
    맵 생성 웹서버 종료 + AMR 맵 생성 정지. 즉시 SUCCESS.
    서버 인스턴스를 bb.ctx에서 꺼내서 stop() 호출.
    """

    def __init__(self, bb: RobotBlackboard, bundle: ServiceBundle) -> None:
        super().__init__(name="[A] StopMapServer")
        self._bb     = bb
        self._bundle = bundle

    def update(self) -> py_trees.common.Status:
        server = self._bb.ctx.get("map_creation_server")
        if server is not None:
            try:
                server.stop()
                log.info("[StopMapServer] server stopped")
            except Exception as e:
                log.warning("[StopMapServer] stop error: %s", e)
            self._bb.ctx.pop("map_creation_server", None)

        # AMR 맵 생성 정지
        self._bundle.amr.send_raw_cmd(62, 2,
            {"Request": {"Set": {"Mapping": {"set": 4}}}})
        log.info("[StopMapServer] AMR mapping stopped")
        return py_trees.common.Status.SUCCESS


class ConditionMapEdit(py_trees.behaviour.Behaviour):
    """bb.map_edit == True 일 때 SUCCESS — Map Edit Sequence gate."""

    def __init__(self, bb: RobotBlackboard) -> None:
        super().__init__(name="[C] MapEdit?")
        self._bb = bb

    def update(self) -> py_trees.common.Status:
        return (py_trees.common.Status.SUCCESS
                if self._bb.map_edit
                else py_trees.common.Status.FAILURE)


class ActionMapEdit(py_trees.behaviour.Behaviour):
    """
    맵 편집 모드 — cmd_map_edit enabled=true 수신 시 MapCreationServer 기동.

    기존 맵/waypoints/금지영역/ROI를 불러와 편집(추가·수정·삭제)하고,
    저장 버튼으로 파일에 반영하는 운영 점검 모드.

    initialise: MapCreationServer 생성 + start()
    update:     bb.map_edit == False → SUCCESS (비활성화 커맨드로 종료)
    terminate:  MapCreationServer stop() + UI 알림
    """

    def __init__(self, bb: RobotBlackboard, bundle: ServiceBundle,
                 port: int = 8080,
                 map_pgm_path: str = "configs/map/map.pgm",
                 waypoints_path: str = "configs/waypoints.json",
                 forbidden_zones_path: str = "configs/map/forbidden_zones.json",
                 roi_zones_path: str = "configs/map/roi_zones.json") -> None:
        super().__init__(name="[A] MapEdit")
        self._bb     = bb
        self._bundle = bundle
        self._port   = port
        self._map_pgm_path = map_pgm_path
        self._waypoints_path = waypoints_path
        self._forbidden_zones_path = forbidden_zones_path
        self._roi_zones_path = roi_zones_path
        self._server = None

    def initialise(self) -> None:
        from ..services.map_creation_server import MapCreationServer
        self._server = MapCreationServer(
            self._bb, self._bundle, port=self._port,
            waypoint_mgr=self._bundle.waypoint_mgr,
            map_pgm_path=self._map_pgm_path,
            waypoints_path=self._waypoints_path,
            forbidden_zones_path=self._forbidden_zones_path,
            roi_zones_path=self._roi_zones_path,
        )
        self._server.start()
        self._bb.ctx["map_creation_server"] = self._server
        self._bundle.send_to_ui({
            "type":    "notify_map_edit_started",
            "payload": {"port": self._port},
        })
        log.info("[MapEdit] server started on port %d — edit mode", self._port)

    def update(self) -> py_trees.common.Status:
        if not self._bb.map_edit:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None:
        if self._server is not None:
            try:
                self._server.stop()
                log.info("[MapEdit] server stopped")
            except Exception as e:
                log.warning("[MapEdit] stop error: %s", e)
            self._bb.ctx.pop("map_creation_server", None)
            self._server = None
        # 편집 모드에서 ROI 저장 시 map_creation_done=True가 설정될 수 있음 — 정리
        self._bb.map_creation_done = False
        self._bundle.send_to_ui({
            "type":    "notify_map_edit_stopped",
            "payload": {},
        })
        log.info("[MapEdit] edit mode ended")


class ActionWaitVoiceReply(py_trees.behaviour.Behaviour):
    """
    STT 이벤트('stt_reply') 대기. timeout 초 초과 시 SUCCESS(skip).
    결과: bb.ctx['stt_text'] 에 텍스트 저장.
    """

    def __init__(self, bb: RobotBlackboard, bundle: ServiceBundle,
                 timeout: float = 10.0) -> None:
        super().__init__(name=f"[A] WaitVoiceReply({timeout:.0f}s)")
        self._bb      = bb
        self._bundle  = bundle
        self._timeout = timeout
        self._start: float = 0.0

    def initialise(self) -> None:
        self._start = time.monotonic()
        self._bb.ctx["stt_text"] = ""
        log.debug("[WaitVoiceReply] waiting (timeout=%.1fs)", self._timeout)

    def update(self) -> py_trees.common.Status:
        ev = self._bb.get_ai_event("stt_reply")
        if ev is not None:
            self._bb.ctx["stt_text"] = ev.get("text", "")
            log.info("[WaitVoiceReply] STT result: '%s'", self._bb.ctx["stt_text"])
            return py_trees.common.Status.SUCCESS

        if time.monotonic() - self._start >= self._timeout:
            log.debug("[WaitVoiceReply] timeout → skip")
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING


class ActionNavigateWaypoints(py_trees.behaviour.Behaviour):
    """
    scenario_params['waypoints'] 리스트를 순서대로 이동.
    bb.ctx['wp_index'] 로 진행 상태 추적.
    repeat < 0 이면 무한 순찰, repeat > 0 이면 지정 횟수만큼 전체 경로 순회.
    """

    def __init__(self, bb: RobotBlackboard, bundle: ServiceBundle,
                 timeout_per_wp: float = 300.0, dwell_sec: float = 0.0) -> None:
        super().__init__(name="[A] NavWaypoints")
        self._bb             = bb
        self._bundle         = bundle
        self._timeout_per_wp = timeout_per_wp
        self._dwell_sec      = dwell_sec
        self._wp_list: list[str] = []
        self._wp_start: float    = 0.0
        self._dwell_until: float = 0.0
        self._pending_next_idx: int | None = None
        self._loop_forever = False
        self._repeat = 0
        try:
            self._repeat = int(self._bb.scenario_params.get("repeat") or 0)
        except (TypeError, ValueError):
            self._repeat = 0

    def initialise(self) -> None:
        self._wp_list = list(self._bb.scenario_params.get("waypoints", []))
        wp_list_dump = self._wp_list
        self._dwell_until = 0.0
        self._pending_next_idx = None
        
        try:
            self._repeat = int(self._bb.scenario_params.get("repeat") or 0)
        except (TypeError, ValueError):
            self._repeat = 0
        self._loop_forever = self._repeat < 0
        
        if self._repeat > 0:
            self._wp_list = wp_list_dump * self._repeat
        
        self._bb.ctx["wp_index"] = 0
        self._bb.ctx.pop("nav_waypoint_target", None)
        self._bb.ctx.pop("nav_waypoint_resolved_target", None)
        self._bb.ctx.pop("resume_patrol_nav", None)
        self._bb.ctx.pop("resume_patrol_target", None)
        self._bb.ctx.pop("dwell_remaining", None)
        log.info(
            "[NavWaypoints] waypoints=%s repeat=%s dwell=%.1fs",
            self._wp_list,
            self._repeat,
            self._dwell_sec,
        )
        if self._wp_list:
            self._navigate_to(0)

    def update(self) -> py_trees.common.Status:
        if self._bb.ctx.pop("resume_patrol_nav", False):
            self._resume_navigation()
            return py_trees.common.Status.RUNNING

        if self._bb.patrol_interrupted:
            if self._pending_next_idx is not None and self._dwell_until > 0.0:
                self._bb.ctx["dwell_remaining"] = max(0.0, self._dwell_until - time.monotonic())
            return py_trees.common.Status.RUNNING

        if self._pending_next_idx is not None:
            if time.monotonic() < self._dwell_until:
                self._bb.ctx["dwell_remaining"] = max(0.0, self._dwell_until - time.monotonic())
                return py_trees.common.Status.RUNNING
            next_idx = self._pending_next_idx
            self._pending_next_idx = None
            self._bb.ctx.pop("dwell_remaining", None)
            self._navigate_to(next_idx)
            return py_trees.common.Status.RUNNING

        idx = self._bb.ctx.get("wp_index", 0)
        if idx >= len(self._wp_list):
            if not self._loop_forever or not self._wp_list:
                self._bb.ctx.pop("nav_waypoint_target", None)
                self._bb.ctx.pop("nav_waypoint_resolved_target", None)
                self._bb.ctx.pop("dwell_remaining", None)
                return py_trees.common.Status.SUCCESS
            idx = 0
            self._bb.ctx["wp_index"] = idx
            self._navigate_to(idx)
            return py_trees.common.Status.RUNNING

        event_state = (
            self._bb.amr_arrived_state
            if self._bb.amr_arrived_state is not None
            else self._bb.amr_moving_state
        )
        arrival_state = event_state in (
            MovingState.ARRIVED,
            MovingState.ALTERNATIVE_GOAL,
        )
        arrived = self._bb.amr_arrived and arrival_state
        if self._bb.amr_arrived and not arrival_state:
            log.info(
                "[NavWaypoints] ignore non-arrival transition wp[%d] state=%s",
                idx,
                event_state,
            )
            self._bb.amr_arrived = False
        timeout = (time.monotonic() - self._wp_start) >= self._timeout_per_wp
        if arrived or timeout:
            if timeout:
                log.warning("[NavWaypoints] wp[%d] timeout → next", idx)
            else:
                log.info(
                    "[NavWaypoints] wp[%d] arrived ✓ state=%s",
                    idx,
                    event_state,
                )
            idx += 1
            if idx >= len(self._wp_list):
                if not self._loop_forever:
                    self._bb.ctx["wp_index"] = idx
                    log.info("[NavWaypoints] all waypoints done")
                    return py_trees.common.Status.SUCCESS
                idx = 0
                log.info("[NavWaypoints] repeat loop restart")
            self._bb.ctx["wp_index"] = idx
            self._wait_then_navigate(idx)

        return py_trees.common.Status.RUNNING

    def _wait_then_navigate(self, idx: int) -> None:
        if self._dwell_sec <= 0:
            self._navigate_to(idx)
            return
        self._pending_next_idx = idx
        self._dwell_until = time.monotonic() + self._dwell_sec
        self._bb.ctx["dwell_remaining"] = self._dwell_sec
        log.info("[NavWaypoints] dwell %.1fs before wp[%d]", self._dwell_sec, idx)

    def _navigate_to(self, idx: int) -> None:
        key = self._wp_list[idx]
        resolved_key, coord = _resolve_waypoint_coord(self._bundle, key)
        if coord is None:
            log.error("[NavWaypoints] unknown waypoint '%s' — skip", key)
            self._wp_start = time.monotonic() - self._timeout_per_wp
            return
        self._bb.amr_arrived = False
        self._bundle.amr.send_target_position(coord)
        self._wp_start = time.monotonic()
        self._bb.patrol_current_idx = idx
        self._bb.ctx["wp_index"] = idx
        self._bb.ctx["nav_waypoint_target"] = key
        self._bb.ctx["nav_waypoint_resolved_target"] = resolved_key
        self._bb.ctx.pop("dwell_remaining", None)
        log.info("[NavWaypoints] → wp[%d]='%s' %s", idx, resolved_key, coord)

    def _resume_navigation(self) -> None:
        if not self._wp_list:
            log.warning("[NavWaypoints] resume requested without waypoints")
            return
        target = str(self._bb.ctx.pop("resume_patrol_target", "") or "")
        try:
            preferred_idx = int(self._bb.ctx.get("wp_index", self._bb.patrol_current_idx) or 0)
        except (TypeError, ValueError):
            preferred_idx = 0
        idx = self._index_for_waypoint(target, preferred_idx)
        if idx is None:
            idx = preferred_idx
        if idx >= len(self._wp_list):
            if not self._loop_forever:
                idx = max(0, len(self._wp_list) - 1)
            else:
                idx = 0
        if idx < 0:
            idx = 0
        self._pending_next_idx = None
        self._dwell_until = 0.0
        self._navigate_to(idx)
        log.info("[NavWaypoints] resume patrol navigation wp[%d] target=%s", idx, target or "-")

    def _index_for_waypoint(self, target: str, preferred_idx: int | None = None) -> int | None:
        if not target:
            return None
        matches: list[int] = []
        for idx, key in enumerate(self._wp_list):
            if str(key) == target:
                matches.append(idx)
                continue
            resolved_key, _ = _resolve_waypoint_coord(self._bundle, str(key))
            if str(resolved_key) == target:
                matches.append(idx)
        if not matches:
            return None
        if preferred_idx is None:
            return matches[0]
        return min(matches, key=lambda item: abs(item - preferred_idx))


class ActionAnomalyWatch(py_trees.behaviour.Behaviour):
    """
    Patrol Parallel 내에서 항상 RUNNING.
    이상 감지 시 내부 상태머신으로: 정지 → 알림 → 대기 → 재개.
    """
    _MONITORING = "monitoring"
    _NOTIFYING  = "notifying"
    _WAITING    = "waiting"

    WAIT_SEC = 30.0   # 이상 감지 후 대기 시간(초)

    def __init__(self, bb: RobotBlackboard, bundle: ServiceBundle) -> None:
        super().__init__(name="[A] AnomalyWatch")
        self._bb     = bb
        self._bundle = bundle
        self._state  = self._MONITORING
        self._wait_start: float = 0.0

    def initialise(self) -> None:
        self._state = self._MONITORING

    def update(self) -> py_trees.common.Status:
        if self._state == self._MONITORING:
            if self._bb.has_ai_event("person_lying_down"):
                self._bundle.amr.send_stop()
                self._bundle.send_to_ui({
                    "type":    "notify_anomaly_detected",
                    "payload": {"position": dict(self._bb.amr_position)},
                })
                log.warning("[AnomalyWatch] anomaly detected — stopped AMR, notified UI")
                self._state = self._NOTIFYING

        elif self._state == self._NOTIFYING:
            # 알림 전송 완료 → 대기
            self._wait_start = time.monotonic()
            self._state = self._WAITING
            log.info("[AnomalyWatch] waiting %.1fs before resuming", self.WAIT_SEC)

        elif self._state == self._WAITING:
            if time.monotonic() - self._wait_start >= self.WAIT_SEC:
                self._state = self._MONITORING
                log.info("[AnomalyWatch] resuming patrol monitoring")

        return py_trees.common.Status.RUNNING   # 항상 RUNNING


# ─────────────────────────────────────────────────────────────────────────────
# 5/21 데모: 긴급상황감지 / 모닝콜 노드
# ─────────────────────────────────────────────────────────────────────────────

class ConditionFallDetected(py_trees.behaviour.Behaviour):
    """AI fall-status API의 fall_detected 결과가 있으면 SUCCESS."""

    def __init__(self, bb: RobotBlackboard, bundle: ServiceBundle) -> None:
        super().__init__(name="[C] FallDetected?")
        self._bb = bb
        self._bundle = bundle

    def update(self) -> py_trees.common.Status:
        ev = self._bb.get_ai_event("fall_detected") or self._bb.get_ai_event("person_lying_down")
        result = dict(ev) if ev else self._bundle.ai.detect_fall()
        detected = bool(result.get("fall_detected", result.get("detected", ev is not None)))
        self._bb.fall_candidate = bool(result.get("fall_candidate", result.get("candidate", False)))
        self._bb.fall_status = str(result.get("fall_status", result.get("status", "")) or "")
        if not detected:
            return py_trees.common.Status.FAILURE
        self._bb.fall_detected = True
        self._bb.fall_confidence = float(result.get("confidence", 0.0) or 0.0)
        self._bb.fall_image_path = str(result.get("image_path", "") or "")
        self._bb.detection_type = "fall"
        self._bb.ctx["detected_person_position"] = result.get("position", result.get("coordinates", {})) or {}
        log.warning(
            "[FallDetected] status=%s confidence=%.2f position=%s",
            self._bb.fall_status,
            self._bb.fall_confidence,
            self._bb.ctx.get("detected_person_position", {}),
        )
        return py_trees.common.Status.SUCCESS


class ConditionFallCandidate(py_trees.behaviour.Behaviour):
    """AI fall-status API의 fall_candidate 결과가 있으면 SUCCESS."""

    def __init__(self, bb: RobotBlackboard, bundle: ServiceBundle) -> None:
        super().__init__(name="[C] FallCandidate?")
        self._bb = bb
        self._bundle = bundle

    def update(self) -> py_trees.common.Status:
        ev = self._bb.get_ai_event("fall_candidate")
        result = dict(ev) if ev else self._bundle.ai.detect_fall()
        candidate = bool(result.get("fall_candidate", result.get("candidate", ev is not None)))
        detected = bool(result.get("fall_detected", result.get("detected", False)))
        self._bb.fall_candidate = candidate
        self._bb.fall_detected = detected
        self._bb.fall_status = str(result.get("fall_status", result.get("status", "")) or "")
        if not (candidate or detected):
            return py_trees.common.Status.FAILURE
        self._bb.fall_confidence = float(result.get("confidence", 0.0) or 0.0)
        self._bb.fall_image_path = str(result.get("image_path", "") or "")
        self._bb.detection_type = "fall"
        self._bb.ctx["detected_person_position"] = result.get("position", result.get("coordinates", {})) or {}
        log.warning(
            "[FallCandidate] status=%s detected=%s candidate=%s position=%s",
            self._bb.fall_status,
            detected,
            candidate,
            self._bb.ctx.get("detected_person_position", {}),
        )
        return py_trees.common.Status.SUCCESS


class ConditionFallDetectedDummy(py_trees.behaviour.Behaviour):
    """MQTT debug 주입으로 BB에 저장된 낙상 플래그만 확인한다."""

    def __init__(self, bb: RobotBlackboard) -> None:
        super().__init__(name="[C] FallDetectedDummy?")
        self._bb = bb

    def update(self) -> py_trees.common.Status:
        detected = bool(self._bb.ctx.get("debug_fall_detected", False))
        candidate = bool(self._bb.ctx.get("debug_fall_candidate", False))
        if not (detected or candidate):
            return py_trees.common.Status.FAILURE
        self._bb.fall_detected = detected
        self._bb.fall_candidate = candidate
        self._bb.detection_type = "fall"
        log.warning(
            "[FallDetectedDummy] detected=%s candidate=%s status=%s",
            detected,
            candidate,
            self._bb.fall_status,
        )
        return py_trees.common.Status.SUCCESS


class ConditionWanderDetected(py_trees.behaviour.Behaviour):
    """AI 반복 감지 기반 배회 판정이 있으면 SUCCESS."""

    def __init__(self, bb: RobotBlackboard, bundle: ServiceBundle) -> None:
        super().__init__(name="[C] WanderDetected?")
        self._bb = bb
        self._bundle = bundle

    def update(self) -> py_trees.common.Status:
        ev = self._bb.get_ai_event("wander_detected") or self._bb.get_ai_event("person_detected_repeated")
        result = dict(ev) if ev else self._bundle.ai.detect_wander(self._bb.wander_person_id)
        repeated = bool(result.get("repeated", ev is not None))
        if not repeated:
            return py_trees.common.Status.FAILURE
        self._bb.wander_detected = True
        self._bb.wander_person_id = str(result.get("person_id", self._bb.wander_person_id or "P001") or "")
        self._bb.wander_count = int(result.get("count", 2) or 0)
        self._bb.wander_image_path = str(result.get("image_path", "") or "")
        self._bb.detection_type = "wander"
        log.warning("[WanderDetected] person=%s count=%d", self._bb.wander_person_id, self._bb.wander_count)
        return py_trees.common.Status.SUCCESS


class ConditionWanderDetectedDummy(py_trees.behaviour.Behaviour):
    """MQTT debug 주입으로 BB에 저장된 배회 플래그만 확인한다."""

    def __init__(self, bb: RobotBlackboard) -> None:
        super().__init__(name="[C] WanderDetectedDummy?")
        self._bb = bb

    def update(self) -> py_trees.common.Status:
        if not bool(self._bb.ctx.get("debug_wander_detected", False)):
            return py_trees.common.Status.FAILURE
        self._bb.wander_detected = True
        self._bb.detection_type = "wander"
        if not self._bb.wander_person_id:
            self._bb.wander_person_id = "debug_person"
        if self._bb.wander_count <= 0:
            self._bb.wander_count = 2
        log.warning(
            "[WanderDetectedDummy] person=%s count=%d",
            self._bb.wander_person_id,
            self._bb.wander_count,
        )
        return py_trees.common.Status.SUCCESS


class ConditionVoiceIntentOk(py_trees.behaviour.Behaviour):
    """낙상 음성 확인 응답이 '괜찮음' 계열이면 SUCCESS."""

    def __init__(self, bb: RobotBlackboard) -> None:
        super().__init__(name="[C] VoiceIntentOk?")
        self._bb = bb

    def update(self) -> py_trees.common.Status:
        intent = str(self._bb.voice_agent_intent or "").lower()
        text = str(self._bb.voice_agent_response or self._bb.ctx.get("stt_text", "") or "")
        ok = intent in ("ok", "fine", "normal") or any(token in text for token in ("괜찮", "괜찬", "문제없"))
        return py_trees.common.Status.SUCCESS if ok else py_trees.common.Status.FAILURE


class ConditionMoreRoomsToVisit(py_trees.behaviour.Behaviour):
    """모닝콜 방문 세대가 남아 있으면 SUCCESS."""

    def __init__(self, bb: RobotBlackboard) -> None:
        super().__init__(name="[C] MoreRooms?")
        self._bb = bb

    def update(self) -> py_trees.common.Status:
        return (
            py_trees.common.Status.SUCCESS
            if self._bb.current_visit_index < len(self._bb.morning_call_visits)
            else py_trees.common.Status.FAILURE
        )


class ActionLoadPatrolConfig(py_trees.behaviour.Behaviour):
    """feature_settings.json의 순찰 설정을 Blackboard/scenario_params에 반영."""

    def __init__(self, bb: RobotBlackboard, bundle: ServiceBundle) -> None:
        super().__init__(name="[A] LoadPatrolConfig")
        self._bb = bb
        self._bundle = bundle

    def update(self) -> py_trees.common.Status:
        patrol = self._bundle.feature_cfg.get("patrol", {}) if isinstance(self._bundle.feature_cfg, dict) else {}
        waypoints = list(patrol.get("waypoint_sequence", []) or self._bb.patrol_waypoints)
        if not waypoints:
            waypoints = list(self._bundle.waypoints.keys())[:2]
        if not waypoints:
            log.warning("[LoadPatrolConfig] no patrol waypoints")
            return py_trees.common.Status.FAILURE
        self._bb.patrol_waypoints = waypoints
        self._bb.patrol_dwell_sec = int(patrol.get("dwell_time_sec", self._bb.patrol_dwell_sec) or self._bb.patrol_dwell_sec)
        self._bb.scenario_params["waypoints"] = waypoints
        try:
            self._bb.scenario_params["repeat"] = int(patrol.get("repeat", -1))
        except (TypeError, ValueError):
            self._bb.scenario_params["repeat"] = -1
        reset_fn = getattr(self._bundle.ai, "reset_patrol_detection_state", None)
        if callable(reset_fn):
            reset_fn()
        log.info(
            "[LoadPatrolConfig] waypoints=%s dwell=%s repeat=%s",
            waypoints,
            self._bb.patrol_dwell_sec,
            self._bb.scenario_params["repeat"],
        )
        return py_trees.common.Status.SUCCESS


class ActionSavePatrolState(py_trees.behaviour.Behaviour):
    """감지 발생 시 순찰 진행 상태를 임시 저장."""

    def __init__(self, bb: RobotBlackboard, bundle: ServiceBundle) -> None:
        super().__init__(name="[A] SavePatrolState")
        self._bb = bb
        self._bundle = bundle

    def update(self) -> py_trees.common.Status:
        idx = int(self._bb.ctx.get("wp_index", self._bb.patrol_current_idx) or 0)
        self._bb.patrol_current_idx = idx
        dwell_remaining = float(self._bb.ctx.get("dwell_remaining", 0.0) or 0.0)
        target = ""
        if dwell_remaining > 0.0 and self._bb.patrol_waypoints:
            target = str(self._bb.patrol_waypoints[idx % len(self._bb.patrol_waypoints)])
        if not target:
            target = str(
                self._bb.ctx.get("nav_waypoint_target")
                or self._bb.ctx.get("nav_waypoint_resolved_target")
                or ""
            )
        if not target and self._bb.patrol_waypoints:
            target = str(self._bb.patrol_waypoints[idx % len(self._bb.patrol_waypoints)])
        self._bb.saved_nav_target = {"waypoint_id": target, "index": idx}
        self._bb.saved_dwell_remaining = dwell_remaining
        self._bb.patrol_interrupted = True
        log.info("[SavePatrolState] target=%s idx=%d dwell_remaining=%.1f", target, idx, dwell_remaining)
        return py_trees.common.Status.SUCCESS


class ActionRestorePatrolState(py_trees.behaviour.Behaviour):
    """상황 처리 후 저장된 순찰 상태를 복원."""

    def __init__(self, bb: RobotBlackboard, bundle: ServiceBundle) -> None:
        super().__init__(name="[A] RestorePatrolState")
        self._bb = bb
        self._bundle = bundle

    def update(self) -> py_trees.common.Status:
        target = str(self._bb.saved_nav_target.get("waypoint_id", "") or "")
        if target:
            self._bb.ctx["wp_index"] = int(self._bb.saved_nav_target.get("index", 0) or 0)
            self._bb.ctx["resume_patrol_target"] = target
            self._bb.ctx["resume_patrol_nav"] = True
        self._bb.patrol_interrupted = False
        self._bb.saved_nav_target = {}
        self._bb.saved_dwell_remaining = 0.0
        self._bb.fall_detected = False
        self._bb.fall_candidate = False
        self._bb.fall_status = ""
        self._bb.fall_confidence = 0.0
        self._bb.fall_image_path = ""
        self._bb.wander_detected = False
        self._bb.ctx.pop("debug_fall_detected", None)
        self._bb.ctx.pop("debug_fall_candidate", None)
        self._bb.ctx.pop("debug_wander_detected", None)
        log.info("[RestorePatrolState] restored target=%s", target)
        return py_trees.common.Status.SUCCESS


class ActionClearFallDetection(py_trees.behaviour.Behaviour):
    """낙상 시나리오 종료 후 BB 낙상 감지 상태를 초기화한다."""

    def __init__(self, bb: RobotBlackboard, bundle: ServiceBundle | None = None) -> None:
        super().__init__(name="[A] ClearFallDetection")
        self._bb = bb
        self._bundle = bundle

    def update(self) -> py_trees.common.Status:
        self._bb.fall_detected = False
        self._bb.fall_candidate = False
        self._bb.fall_status = ""
        self._bb.fall_confidence = 0.0
        self._bb.fall_image_path = ""
        if self._bb.detection_type == "fall":
            self._bb.detection_type = ""
        self._bb.ctx.pop("detected_person_position", None)
        self._bb.ctx.pop("debug_fall_detected", None)
        self._bb.ctx.pop("debug_fall_candidate", None)
        if self._bundle is not None:
            reset_fn = getattr(self._bundle.ai, "mark_fall_detection_handled", None)
            if not callable(reset_fn):
                reset_fn = getattr(self._bundle.ai, "reset_fall_candidate_state", None)
            if callable(reset_fn):
                reset_fn()
        log.info("[ClearFallDetection] fall flags reset")
        return py_trees.common.Status.SUCCESS


class ActionClearWanderDetection(py_trees.behaviour.Behaviour):
    """배회 시나리오 종료 후 BB 배회 감지 상태를 초기화한다."""

    def __init__(self, bb: RobotBlackboard, bundle: ServiceBundle | None = None) -> None:
        super().__init__(name="[A] ClearWanderDetection")
        self._bb = bb
        self._bundle = bundle

    def update(self) -> py_trees.common.Status:
        self._bb.wander_detected = False
        self._bb.wander_person_id = ""
        self._bb.wander_count = 0
        self._bb.wander_image_path = ""
        if self._bb.detection_type == "wander":
            self._bb.detection_type = ""
        self._bb.ctx.pop("debug_wander_detected", None)
        if self._bundle is not None:
            reset_fn = getattr(self._bundle.ai, "mark_wander_detection_handled", None)
            if callable(reset_fn):
                reset_fn()
        log.info("[ClearWanderDetection] wander flags reset")
        return py_trees.common.Status.SUCCESS


class ActionApproachPerson(py_trees.behaviour.Behaviour):
    """감지된 사람 위치로 접근. 위치가 없으면 정지 상태로 즉시 통과."""

    def __init__(self, bb: RobotBlackboard, bundle: ServiceBundle, timeout: float = 30.0) -> None:
        super().__init__(name="[A] ApproachPerson")
        self._bb = bb
        self._bundle = bundle
        self._timeout = timeout
        self._start = 0.0
        self._sent = False

    def initialise(self) -> None:
        self._start = time.monotonic()
        pos = self._bb.ctx.get("detected_person_position", {}) or {}
        if isinstance(pos, dict) and {"x", "y"} <= set(pos.keys()):
            coord = {
                "x": float(pos.get("x", 0.0)),
                "y": float(pos.get("y", 0.0)),
                "theta": float(pos.get("theta", 0.0)),
            }
            self._bundle.amr.send_target_position(coord)
            self._sent = True
            log.info("[ApproachPerson] target=%s", coord)
        else:
            self._sent = False
            log.info("[ApproachPerson] no target position, skip navigation")

    def update(self) -> py_trees.common.Status:
        if not self._sent:
            return py_trees.common.Status.SUCCESS
        if self._bb.amr_arrived or time.monotonic() - self._start >= self._timeout:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


class ActionVoiceStatusCheck(py_trees.behaviour.Behaviour):
    """낙상자에게 상태 확인 질문을 보내고 Agent Events 응답 intent를 저장."""

    def __init__(self, bb: RobotBlackboard, bundle: ServiceBundle) -> None:
        super().__init__(name="[A] VoiceStatusCheck")
        self._bb = bb
        self._bundle = bundle
        self._executor: ThreadPoolExecutor | None = None
        self._future: Future | None = None
        self._start: float = 0.0
        self._timeout: float = 0.0

    def initialise(self) -> None:
        self._start = time.monotonic()
        ai = getattr(self._bundle, "ai", None)
        tts_timeout = float(getattr(ai, "tts_done_timeout", 20.0) or 20.0)
        conversation_timeout = float(getattr(ai, "conversation_wait_timeout", 45.0) or 45.0)
        self._timeout = max(conversation_timeout, 10.0)
        self._bb.voice_agent_intent = ""
        self._bb.voice_agent_response = ""
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="voice-status")
        self._future = self._executor.submit(self._run_check)

    def update(self) -> py_trees.common.Status:
        if self._future is None:
            return py_trees.common.Status.FAILURE
        if self._future.done():
            try:
                result = self._future.result()
            except Exception:
                log.warning("[VoiceStatusCheck] worker failed", exc_info=True)
                result = {"intent": "unknown", "end_reason": "error"}
            self._store_result(result)
            return py_trees.common.Status.SUCCESS
        if time.monotonic() - self._start >= self._timeout:
            self._store_result({"intent": "timeout", "end_reason": "timeout"})
            log.warning("[VoiceStatusCheck] timeout after %.1fs", self._timeout)
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None:
        if self._executor is not None:
            self._executor.shutdown(wait=False)
            self._executor = None
        self._future = None

    def _run_check(self) -> dict:
        question = "괜찮으세요? 다치신 데는 없으세요? 도움이 필요하시면 말씀해 주세요."
        try:
            _post_tts_speak_and_wait(self._bundle, question, 5.0, wait_until_done=True)
        except Exception:
            log.warning("[VoiceStatusCheck] question TTS failed", exc_info=True)

        return self._bundle.ai.start_conversation("fall_status_check", {})

    def _store_result(self, result: dict) -> None:
        user_text = str(result.get("last_user_text", "") or "")
        self._bb.voice_agent_response = user_text or str(result.get("text", "") or "")
        self._bb.voice_agent_intent = str(result.get("intent", "") or self._classify_intent(self._bb.voice_agent_response)).lower()
        self._bb.ctx["voice_last_user_text"] = user_text
        self._bb.ctx["voice_end_reason"] = str(result.get("end_reason", "") or "")
        self._bb.ctx["voice_event_id"] = int(result.get("event_id", 0) or 0)
        self._bb.ctx["voice_source"] = str(result.get("source", "") or "")
        log.info(
            "[VoiceStatusCheck] intent=%s end_reason=%s response=%s",
            self._bb.voice_agent_intent,
            self._bb.ctx["voice_end_reason"],
            self._bb.voice_agent_response,
        )
        return py_trees.common.Status.SUCCESS

    def _classify_intent(self, text: str) -> str:
        if any(token in text for token in ("괜찮", "괜찬", "문제없")):
            return "ok"
        if any(token in text for token in ("도와", "도움", "살려", "아파", "119")):
            return "help"
        return "unknown"


class ActionTakePhoto(py_trees.behaviour.Behaviour):
    """카메라 촬영 요청. 실패해도 데모 흐름은 계속한다."""

    def __init__(self, bb: RobotBlackboard, bundle: ServiceBundle) -> None:
        super().__init__(name="[A] TakePhoto")
        self._bb = bb
        self._bundle = bundle

    def update(self) -> py_trees.common.Status:
        result = self._bundle.ai.call("/api/camera/capture", "POST", {}, 5.0) or {}
        image_path = str(result.get("image_path", "") or self._bb.fall_image_path or self._bb.wander_image_path or "")
        self._bb.ctx["photo_path"] = image_path
        log.info("[TakePhoto] image=%s", image_path or "(none)")
        return py_trees.common.Status.SUCCESS


class ActionDetermineLocation(py_trees.behaviour.Behaviour):
    """현재 ROI 위치를 감지 이벤트 컨텍스트에 저장."""

    def __init__(self, bb: RobotBlackboard, bundle: ServiceBundle) -> None:
        super().__init__(name="[A] DetermineLocation")
        self._bb = bb
        self._bundle = bundle

    def update(self) -> py_trees.common.Status:
        self._bb.ctx["detection_location_id"] = self._bb.current_location_id
        self._bb.ctx["detection_location_name"] = self._bb.current_location_name
        return py_trees.common.Status.SUCCESS


class ActionNotifyManager(py_trees.behaviour.Behaviour):
    """관리자/UI 알림 발행."""

    def __init__(self, bb: RobotBlackboard, bundle: ServiceBundle) -> None:
        super().__init__(name="[A] NotifyManager")
        self._bb = bb
        self._bundle = bundle

    def update(self) -> py_trees.common.Status:
        detection_type = self._bb.detection_type or "unknown"
        self._bundle.send_to_ui({
            "type": "notify_manager",
            "payload": {
                "detection_type": detection_type,
                "location_id": self._bb.current_location_id,
                "location_name": self._bb.current_location_name,
                "image_path": self._bb.ctx.get("photo_path", ""),
            },
        })
        log.warning("[NotifyManager] detection=%s location=%s", detection_type, self._bb.current_location_name)
        return py_trees.common.Status.SUCCESS


class ActionPublishDetection(py_trees.behaviour.Behaviour):
    """robot/detection 토픽 발행."""

    def __init__(self, bb: RobotBlackboard, bundle: ServiceBundle, detection_type: str) -> None:
        super().__init__(name=f"[A] PublishDetection({detection_type})")
        self._bb = bb
        self._bundle = bundle
        self._detection_type = detection_type

    def update(self) -> py_trees.common.Status:
        image_path = self._bb.fall_image_path if self._detection_type == "fall" else self._bb.wander_image_path
        confidence = self._bb.fall_confidence if self._detection_type == "fall" else 0.0
        self._bundle.send_to_ui({
            "type": "detection_event",
            "payload": {
                "type": self._detection_type,
                "location": self._bb.current_location_name,
                "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                "image_path": image_path or self._bb.ctx.get("photo_path", ""),
                "image_attached": bool(image_path or self._bb.ctx.get("photo_path")),
            },
        })
        return py_trees.common.Status.SUCCESS


class ActionLoadMorningCallSchedule(py_trees.behaviour.Behaviour):
    """feature_settings.json의 모닝콜 방문 리스트를 로드."""

    def __init__(self, bb: RobotBlackboard, bundle: ServiceBundle) -> None:
        super().__init__(name="[A] LoadMorningCall")
        self._bb = bb
        self._bundle = bundle

    def update(self) -> py_trees.common.Status:
        cfg = self._bundle.feature_cfg.get("morning_call", {}) if isinstance(self._bundle.feature_cfg, dict) else {}
        visits = list(cfg.get("visits", []) or self._bb.morning_call_visits)
        if not visits:
            log.warning("[LoadMorningCall] no visits configured")
            return py_trees.common.Status.FAILURE
        self._bb.morning_call_visits = visits
        self._bb.current_visit_index = 0
        self._bb.door_retry_count = 0
        self._bb.door_opened = False
        self._bb.morning_call_active = True
        return py_trees.common.Status.SUCCESS


class ActionNavigateToRoom(py_trees.behaviour.Behaviour):
    """현재 모닝콜 방문 세대 waypoint로 이동."""

    def __init__(self, bb: RobotBlackboard, bundle: ServiceBundle, timeout: float = 120.0) -> None:
        super().__init__(name="[A] NavToRoom")
        self._bb = bb
        self._bundle = bundle
        self._timeout = timeout
        self._start = 0.0
        self._sent = False

    def initialise(self) -> None:
        self._start = time.monotonic()
        visit = self._current_visit()
        waypoint_id = str(visit.get("target_location_id", visit.get("room", "")) or "")
        self._bb.ctx["target_unit"] = waypoint_id
        resolved_id, coord = _resolve_waypoint_coord(self._bundle, waypoint_id)
        if coord is None:
            log.warning("[NavToRoom] unknown waypoint=%s, skip navigation", waypoint_id)
            self._sent = False
            return
        self._bundle.amr.send_target_position(coord)
        self._sent = True
        log.info("[NavToRoom] room=%s waypoint=%s", visit.get("room", ""), resolved_id)

    def update(self) -> py_trees.common.Status:
        if not self._sent:
            return py_trees.common.Status.SUCCESS
        if self._bb.amr_arrived or time.monotonic() - self._start >= self._timeout:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

    def _current_visit(self) -> dict:
        if self._bb.current_visit_index < len(self._bb.morning_call_visits):
            visit = self._bb.morning_call_visits[self._bb.current_visit_index]
            return visit if isinstance(visit, dict) else {}
        return {}


class ActionRingBell(py_trees.behaviour.Behaviour):
    """도착 알림벨을 duration_sec 동안 유지.

    bb.ctx["target_unit"] (ActionNavigateToRoom이 설정)에서 waypoint_id를 읽고,
    waypoint_mgr에서 bell_id를 조회하여 rf_manager POST /notify 호출.
    bell_id가 없으면 경고 후 타이머만 동작 (벨 없이 대기).
    """

    def __init__(self, bb: RobotBlackboard, bundle: ServiceBundle, duration_sec: float = 20.0) -> None:
        super().__init__(name=f"[A] RingBell({duration_sec:.0f}s)")
        self._bb = bb
        self._bundle = bundle
        self._duration = duration_sec
        self._start = 0.0
        self._sent = False

    def initialise(self) -> None:
        self._start = time.monotonic()
        self._sent = False

        # bb.ctx["target_unit"]에서 현재 방문 세대 waypoint_id 조회
        dest = self._bb.ctx.get("target_unit", "")
        bell_id = ""
        if dest and self._bundle.waypoint_mgr:
            wp = self._bundle.waypoint_mgr.get(str(dest))
            if wp and wp.bell_id:
                bell_id = wp.bell_id

        if not bell_id:
            log.warning("[RingBell] bell_id 없음 (dest='%s') — 벨 없이 대기", dest)
            return

        try:
            import requests
            requests.post(
                f"{self._bundle.rf_base_url}/notify",
                json={"bell_id": bell_id},
                timeout=3.0,
            )
            self._sent = True
            log.info("[RingBell] RF notify → bell_id=%s (dest=%s)", bell_id, dest)
        except Exception as exc:
            log.warning("[RingBell] RF notify failed (bell_id=%s): %s", bell_id, exc)

    def update(self) -> py_trees.common.Status:
        return (
            py_trees.common.Status.SUCCESS
            if time.monotonic() - self._start >= self._duration
            else py_trees.common.Status.RUNNING
        )


class ActionWaitDoorOpenDummy(py_trees.behaviour.Behaviour):
    """테스트용 문열림 대기. 지정 시간 뒤 자동으로 문열림 처리."""

    def __init__(self, bb: RobotBlackboard, bundle: ServiceBundle, delay_sec: float = 3.0) -> None:
        super().__init__(name=f"[A] WaitDoorDummy({delay_sec:.1f}s)")
        self._bb = bb
        self._bundle = bundle
        self._delay_sec = max(float(delay_sec), 0.0)
        self._start = 0.0

    def initialise(self) -> None:
        self._start = time.monotonic()
        self._bb.door_opened = False
        reset_fn = getattr(self._bundle.ai, "reset_door_open_detection", None)
        if callable(reset_fn):
            reset_fn()

    def update(self) -> py_trees.common.Status:
        if time.monotonic() - self._start < self._delay_sec:
            return py_trees.common.Status.RUNNING
        self._bb.door_opened = True
        return py_trees.common.Status.SUCCESS


class ActionWaitDoorOpen(py_trees.behaviour.Behaviour):
    """Face API 기반 문열림 감지 대기."""

    def __init__(self, bb: RobotBlackboard, bundle: ServiceBundle, wait_sec: float = 300.0, max_retry: int = 2) -> None:
        super().__init__(name="[A] WaitDoorOpen")
        self._bb = bb
        self._bundle = bundle
        self._wait_sec = wait_sec
        self._max_retry = max_retry
        self._start = 0.0
        self._last_log = 0.0

    def initialise(self) -> None:
        self._start = time.monotonic()
        self._last_log = 0.0
        self._bb.door_opened = False
        reset_fn = getattr(self._bundle.ai, "reset_door_open_detection", None)
        if callable(reset_fn):
            reset_fn()

    def update(self) -> py_trees.common.Status:
        result = self._bundle.ai.detect_door_open()
        if bool(result.get("opened", False)):
            self._bb.door_opened = True
            log.info(
                "[WaitDoorOpen] opened source=%s hits=%s unknown=%s recognized=%s",
                result.get("source", ""),
                result.get("hits", ""),
                result.get("unknown_count", ""),
                result.get("recognized", ""),
            )
            return py_trees.common.Status.SUCCESS
        now = time.monotonic()
        if now - self._last_log >= 2.0:
            self._last_log = now
            log.info(
                "[WaitDoorOpen] waiting source=%s hits=%s/%s stale=%s unknown=%s known=%s recognized=%s",
                result.get("source", ""),
                result.get("hits", 0),
                result.get("min_hits", ""),
                result.get("stale", False),
                result.get("unknown_count", 0),
                result.get("known_count", 0),
                result.get("recognized", False),
            )
        if time.monotonic() - self._start < self._wait_sec:
            return py_trees.common.Status.RUNNING
        self._bb.door_retry_count += 1
        if self._bb.door_retry_count < self._max_retry:
            log.warning(
                "[WaitDoorOpen] timeout retry=%s/%s wait_sec=%.1f",
                self._bb.door_retry_count,
                self._max_retry,
                self._wait_sec,
            )
            self._start = time.monotonic()
            return py_trees.common.Status.RUNNING
        log.warning(
            "[WaitDoorOpen] failed retry=%s/%s wait_sec=%.1f",
            self._bb.door_retry_count,
            self._max_retry,
            self._wait_sec,
        )
        return py_trees.common.Status.FAILURE


class ActionGreetResident(py_trees.behaviour.Behaviour):
    """얼굴 인식 + 아침 인사."""

    def __init__(self, bb: RobotBlackboard, bundle: ServiceBundle, wait_sec: float = 8.0) -> None:
        super().__init__(name="[A] GreetResident")
        self._bb = bb
        self._bundle = bundle
        self._wait_sec = max(float(wait_sec), 0.0)
        self._start = 0.0
        self._last_log = 0.0

    def initialise(self) -> None:
        self._start = time.monotonic()
        self._last_log = 0.0

    def update(self) -> py_trees.common.Status:
        face = self._bundle.ai.recognize_face()
        visit = self._bb.morning_call_visits[self._bb.current_visit_index] if self._bb.current_visit_index < len(self._bb.morning_call_visits) else {}
        face_name = str(face.get("name", "") or "")
        scheduled_name = str(visit.get("resident_name", "") or "")
        mc_cfg = self._bundle.feature_cfg.get("morning_call", {}) if isinstance(self._bundle.feature_cfg, dict) else {}
        use_schedule_fallback = bool(mc_cfg.get("use_schedule_name_fallback", False))
        recognized = bool(face.get("recognized", False)) and bool(face_name)
        elapsed = time.monotonic() - self._start
        if not recognized and elapsed < self._wait_sec:
            now = time.monotonic()
            if now - self._last_log >= 2.0:
                self._last_log = now
                log.info(
                    "[GreetResident] waiting face source=%s stale=%s unknown=%s known=%s elapsed=%.1f/%.1f",
                    face.get("source", ""),
                    face.get("stale", False),
                    face.get("unknown_count", 0),
                    face.get("known_count", 0),
                    elapsed,
                    self._wait_sec,
                )
            return py_trees.common.Status.RUNNING

        name = face_name if recognized else (scheduled_name if use_schedule_fallback else "")
        stable_ment = ", 안녕하세요. 밤새 편히 쉬셨나요? 오늘은 기분이 어떠신지 궁금합니다."
        text = f" {name}님" + stable_ment if name else stable_ment
        self._bb.ctx["recognized_name"] = name
        self._bb.ctx["recognized_unit"] = str(face.get("unit", "") or visit.get("room_id", "") or "")
        self._bb.ctx["face_recognized"] = recognized
        self._bb.ctx["face_confidence"] = float(face.get("confidence", 0.0) or 0.0)
        self._bb.ctx["face_unknown_count"] = int(face.get("unknown_count", 0) or 0)
        self._bb.ctx["face_summary_text"] = str(face.get("summary_text", "") or "")
        self._bb.ctx["face_source"] = str(face.get("source", "") or "")
        if not recognized:
            log.info(
                "[GreetResident] face not recognized source=%s stale=%s unknown=%s schedule_fallback=%s",
                face.get("source", ""),
                face.get("stale", False),
                face.get("unknown_count", 0),
                use_schedule_fallback,
            )
        else:
            log.info(
                "[GreetResident] face recognized name=%s confidence=%.3f source=%s",
                name,
                self._bb.ctx["face_confidence"],
                face.get("source", ""),
            )
        try:
            _post_tts_speak_and_wait(self._bundle, text, 5.0, wait_until_done=True)
            self._bb.morning_call_detected_person = name
        except Exception:
            log.warning("[GreetResident] greeting TTS failed", exc_info=True)
        return py_trees.common.Status.SUCCESS


class ActionFreeConversation(py_trees.behaviour.Behaviour):
    """Keep morning-call free conversation open until explicit close or session timeout."""

    DEFAULT_END_REASONS = ("explicit_close",)

    def __init__(self, bb: RobotBlackboard, bundle: ServiceBundle) -> None:
        super().__init__(name="[A] FreeConversation")
        self._bb = bb
        self._bundle = bundle
        self._executor: ThreadPoolExecutor | None = None
        self._future: Future | None = None
        self._start: float = 0.0
        self._max_session_sec: float = 180.0
        self._poll_timeout_sec: float = 1.0
        self._end_reasons: set[str] = set(self.DEFAULT_END_REASONS)
        self._event_count: int = 0

    def initialise(self) -> None:
        self._start = time.monotonic()
        self._future = None
        self._event_count = 0
        self._load_config()
        self._bb.ctx["voice_end_reason"] = ""
        self._bb.ctx["voice_free_conversation_active"] = True
        self._bb.ctx["voice_free_conversation_events"] = 0
        self._bb.ctx["voice_free_conversation_max_sec"] = self._max_session_sec
        begin_fn = getattr(self._bundle.ai, "begin_conversation_session", None)
        if callable(begin_fn):
            try:
                self._bb.ctx["voice_event_baseline_id"] = int(begin_fn() or 0)
            except Exception:
                log.warning("[FreeConversation] baseline init failed", exc_info=True)
        self._executor = ThreadPoolExecutor(
            max_workers=1,
            thread_name_prefix="free-conversation",
        )
        log.info(
            "[FreeConversation] session started max=%.1fs poll=%.1fs end_reasons=%s",
            self._max_session_sec,
            self._poll_timeout_sec,
            sorted(self._end_reasons),
        )

    def update(self) -> py_trees.common.Status:
        if self._session_expired():
            self._bb.ctx["voice_end_reason"] = "max_session_timeout"
            log.info(
                "[FreeConversation] session timeout max=%.1fs events=%d",
                self._max_session_sec,
                self._event_count,
            )
            return py_trees.common.Status.SUCCESS

        if self._future is None:
            self._start_poll()
            return py_trees.common.Status.RUNNING

        if not self._future.done():
            return py_trees.common.Status.RUNNING

        try:
            result = self._future.result()
        except Exception:
            log.warning("[FreeConversation] event polling failed", exc_info=True)
            result = None
        self._future = None

        if not result:
            return py_trees.common.Status.RUNNING

        self._store_result(result)
        end_reason = str(result.get("end_reason", "") or "").lower()
        if end_reason in self._end_reasons:
            log.info(
                "[FreeConversation] explicit close id=%s reason=%s events=%d",
                self._bb.ctx.get("voice_event_id", 0),
                end_reason,
                self._event_count,
            )
            return py_trees.common.Status.SUCCESS

        log.info(
            "[FreeConversation] continue id=%s reason=%s events=%d",
            self._bb.ctx.get("voice_event_id", 0),
            end_reason,
            self._event_count,
        )
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None:
        self._bb.ctx["voice_free_conversation_active"] = False
        if self._executor is not None:
            self._executor.shutdown(wait=False)
            self._executor = None
        self._future = None

    def _load_config(self) -> None:
        cfg = self._bundle.feature_cfg.get("morning_call", {}) if isinstance(self._bundle.feature_cfg, dict) else {}
        self._max_session_sec = float(cfg.get("free_conversation_max_session_sec", 180.0) or 180.0)
        self._poll_timeout_sec = float(cfg.get("free_conversation_poll_timeout_sec", 1.0) or 1.0)
        raw_reasons = cfg.get("free_conversation_end_reasons", self.DEFAULT_END_REASONS)
        if isinstance(raw_reasons, str):
            reasons = [raw_reasons]
        elif isinstance(raw_reasons, (list, tuple, set)):
            reasons = list(raw_reasons)
        else:
            reasons = list(self.DEFAULT_END_REASONS)
        self._end_reasons = {
            str(reason or "").strip().lower()
            for reason in reasons
            if str(reason or "").strip()
        } or set(self.DEFAULT_END_REASONS)

    def _session_expired(self) -> bool:
        return time.monotonic() - self._start >= max(self._max_session_sec, 0.0)

    def _start_poll(self) -> None:
        if self._executor is None:
            return
        self._future = self._executor.submit(self._wait_for_event)

    def _wait_for_event(self) -> dict | None:
        wait_fn = getattr(self._bundle.ai, "wait_conversation_event", None)
        if callable(wait_fn):
            return wait_fn(wait_timeout=self._poll_timeout_sec)
        return self._bundle.ai.start_conversation("morning_greeting", {
            "resident_name": self._bb.ctx.get("recognized_name", ""),
        })

    def _store_result(self, result: dict) -> None:
        text = str(result.get("text", "") or result.get("last_tts_text", "") or "")
        self._event_count += 1
        self._bb.voice_agent_response = text
        self._bb.ctx["voice_last_user_text"] = str(result.get("last_user_text", "") or "")
        self._bb.ctx["voice_end_reason"] = str(result.get("end_reason", "") or "")
        self._bb.ctx["voice_conversation_id"] = str(result.get("conversation_id", "") or "")
        self._bb.ctx["voice_event_id"] = int(result.get("event_id", result.get("id", 0)) or 0)
        self._bb.ctx["voice_source"] = str(result.get("source", "agent_events") or "")
        self._bb.ctx["voice_pipeline_state"] = str(result.get("pipeline_state", "") or "")
        self._bb.ctx["voice_stream_running"] = bool(result.get("stream_running", False))
        self._bb.ctx["voice_free_conversation_events"] = self._event_count


class ActionAnnounceSchedule(py_trees.behaviour.Behaviour):
    """프로그램 일정 JSON을 읽어 TTS 안내."""

    def __init__(self, bb: RobotBlackboard, bundle: ServiceBundle) -> None:
        super().__init__(name="[A] AnnounceSchedule")
        self._bb = bb
        self._bundle = bundle

    def update(self) -> py_trees.common.Status:
        schedules = self._bundle.program_schedule.get("schedules", []) if isinstance(self._bundle.program_schedule, dict) else []
        valid = [item for item in schedules if isinstance(item, dict)]
        valid.sort(key=lambda item: str(item.get("time", "") or ""))
        mc_cfg = self._bundle.feature_cfg.get("morning_call", {}) if isinstance(self._bundle.feature_cfg, dict) else {}
        max_items = int(mc_cfg.get("schedule_announce_max_items", 5) or 5)
        if valid:
            selected = valid[:max(max_items, 1)]
            parts: list[str] = []
            for item in selected:
                time_text = str(item.get("time", "") or "").strip()
                title = str(item.get("title", "프로그램") or "프로그램").strip()
                location = str(item.get("location", "") or "").strip()
                if location:
                    parts.append(f"{time_text}에 {location}에서 {title}")
                else:
                    parts.append(f"{time_text}에 {title}")
            text = "오늘 일정은 " + ", ".join(parts)
            if len(valid) > len(selected):
                text += f" 등 총 {len(valid)}개입니다."
            else:
                text += "입니다."
        else:
            text = "오늘 등록된 일정은 없습니다."
        if text:
            try:
                _post_tts_speak(self._bundle, text, 5.0)
            except Exception:
                pass
        return py_trees.common.Status.SUCCESS


class ActionPublishPatrolEvent(py_trees.behaviour.Behaviour):
    """robot/patrol 토픽 발행."""

    def __init__(self, bb: RobotBlackboard, bundle: ServiceBundle, status: str) -> None:
        super().__init__(name=f"[A] PatrolEvent({status})")
        self._bb = bb
        self._bundle = bundle
        self.event_status = status
        self.STATUS_MAP = {"start" : "start",
                           "pause" : "pause",
                           "end" : "end"}

    def update(self) -> py_trees.common.Status:
        self._bundle.send_to_ui({
            "type": "patrol_event",
            "payload": {
                "status" : self.STATUS_MAP.get(self.event_status,"")
            },
        })
        return py_trees.common.Status.SUCCESS


class ActionPublishMorningCallEvent(py_trees.behaviour.Behaviour):
    """robot/morning-call/event 토픽 발행."""

    def __init__(self, bb: RobotBlackboard, bundle: ServiceBundle, status: str) -> None:
        super().__init__(name=f"[A] MorningEvent({status})")
        self._bb = bb
        self._bundle = bundle
        self._status = status
        self.STATUS_MAP = {"moving" : "로 이동합니다.",
                      "arrived" : "에 도착했습니다.",
                      "waiting_door" : "문 앞에서 대기 중입니다.",
                      "talking" : "대화 중 입니다.", 
                      "moving_next" : "다음 장소로 이동합니다.",
                      "completed" : "모닝 콜이 완료되었습니다.",
                      "failed" : "실패"}

    def update(self) -> py_trees.common.Status:
        visit = self._current_visit()
        self._bundle.send_to_ui({
            "type": "morning_call_event",
            "payload": {
                "schedule_id": visit.get("schedule_id", ""),
                "visit_order": self._bb.current_visit_index + 1,
                "room": visit.get("room", ""),
                "resident_name": visit.get("resident_name", ""),
                "status": self._status,
                "display_message": self.STATUS_MAP.get(self._status,""),
            },
        })
        return py_trees.common.Status.SUCCESS

    def _current_visit(self) -> dict:
        if self._bb.current_visit_index < len(self._bb.morning_call_visits):
            visit = self._bb.morning_call_visits[self._bb.current_visit_index]
            return visit if isinstance(visit, dict) else {}
        return {}


class ActionPublishDoorStatus(py_trees.behaviour.Behaviour):
    """robot/morning-call/door 토픽 발행."""

    def __init__(self, bb: RobotBlackboard, bundle: ServiceBundle, reason: str = "") -> None:
        super().__init__(name="[A] DoorStatus")
        self._bb = bb
        self._bundle = bundle
        self._reason = reason

    def update(self) -> py_trees.common.Status:
        visit = self._bb.morning_call_visits[self._bb.current_visit_index] if self._bb.current_visit_index < len(self._bb.morning_call_visits) else {}
        self._bundle.send_to_ui({
            "type": "morning_call_door",
            "payload": {
                "schedule_id": "morning_call_demo",
                "room_id": visit.get("room_id", "") if isinstance(visit, dict) else "",
                "door_opened": self._bb.door_opened,
                "retry_count": self._bb.door_retry_count,
                "reason": self._reason or ("opened" if self._bb.door_opened else "waiting"),
            },
        })
        return py_trees.common.Status.SUCCESS


class ActionAdvanceVisitIndex(py_trees.behaviour.Behaviour):
    """다음 모닝콜 방문 세대로 진행."""

    def __init__(self, bb: RobotBlackboard, bundle: ServiceBundle) -> None:
        super().__init__(name="[A] AdvanceVisit")
        self._bb = bb
        self._bundle = bundle

    def update(self) -> py_trees.common.Status:
        
        mc_cfg = self._bundle.feature_cfg.get("morning_call", {})
        if not mc_cfg:
            log.warning("[PublishSchedule] morning_call config is empty")

        visits = mc_cfg.get("visits", [])
        visit_count = len(visits)
        
        self._bb.current_visit_index += 1
        self._bb.door_retry_count = 0
        self._bb.door_opened = False
        
        if self._bb.current_visit_index >= visit_count:
            self._bb.current_visit_index = visit_count
        
        return py_trees.common.Status.SUCCESS


class ActionPublishRobotStatus(py_trees.behaviour.Behaviour):
    """robot/status 토픽 주기 발행용 노드. BtLayer에서도 동일 발행을 수행한다."""

    def __init__(self, bb: RobotBlackboard, bundle: ServiceBundle, interval: float = 5.0) -> None:
        super().__init__(name="[A] PublishRobotStatus")
        self._bb = bb
        self._bundle = bundle
        self._interval = interval

    def update(self) -> py_trees.common.Status:
        now = time.time()
        if now - self._bb.last_status_publish_time >= self._interval:
            self._bb.last_status_publish_time = now
            self._bundle.send_to_ui({
                "type": "robot_status",
                "payload": {
                    "mode": self._bb.active_scenario or "idle",
                    "battery": self._bb.battery_percent,
                    "connected": True,
                    "event_code": self._bb.current_event_code,
                    "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                },
            })
        return py_trees.common.Status.SUCCESS


# ─────────────────────────────────────────────────────────────────────────────
# RF 알림벨 Action
# ─────────────────────────────────────────────────────────────────────────────

class ActionRFBellNotify(py_trees.behaviour.Behaviour):
    """
    목적지 도착 후 해당 위치의 RF 알림벨에 Bell ID 송신.

    destination_ctx_key 로 bb.ctx 또는 bb.scenario_params 에서 waypoint key를 읽고,
    waypoint_mgr 에서 bell_id 를 조회해 rf_manager POST /notify API 를 호출한다.

    bell_id가 없는 waypoint(home 등)는 skip → 즉시 SUCCESS.
    rf_manager 통신 실패(시리얼 오류, HTTP 오류 등) → FAILURE.

    사용 예:
        ActionRFBellNotify(bb, bundle, destination_ctx_key="target_unit")

    Bell ID 규칙:
        BASE = 0x3FA17B18 (이 값 이하 동작 안 함)
        pos=N → hex(BASE + N), 예) pos=1 → "3FA17B19"
    """

    _HTTP_TIMEOUT = 5.0   # requests timeout (초)

    def __init__(
        self,
        bb: "RobotBlackboard",
        bundle: "ServiceBundle",
        destination_ctx_key: str = "destination",
    ) -> None:
        super().__init__(name="[A] RFBellNotify")
        self._bb      = bb
        self._bundle  = bundle
        self._ctx_key = destination_ctx_key

        self._skip:     bool                      = False
        self._bell_id:  str | None                = None
        self._executor: ThreadPoolExecutor | None = None
        self._future:   Future | None             = None

    def initialise(self) -> None:
        """bell_id 조회 + 비동기 HTTP 호출 시작."""
        self._skip    = False
        self._bell_id = None

        # ctx → scenario_params 순서로 destination key 조회
        dest = (
            self._bb.ctx.get(self._ctx_key)
            or self._bb.scenario_params.get(self._ctx_key)
        )

        if dest and self._bundle.waypoint_mgr:
            wp = self._bundle.waypoint_mgr.get(str(dest))
            if wp and wp.bell_id:
                self._bell_id = wp.bell_id

        if not self._bell_id:
            self._skip = True
            log.info("[RFBellNotify] bell_id 없음 (dest='%s') → skip", dest)
            return

        self._executor = ThreadPoolExecutor(max_workers=1,
                                             thread_name_prefix="rf-notify")
        self._future   = self._executor.submit(self._call_notify, self._bell_id)
        log.info("[RFBellNotify] 송신 시작 → bell_id=%s (dest=%s)",
                 self._bell_id, dest)

    def update(self) -> py_trees.common.Status:
        if self._skip:
            return py_trees.common.Status.SUCCESS   # bell 없는 장소 = 정상 skip

        if self._future is None:
            return py_trees.common.Status.FAILURE

        if not self._future.done():
            return py_trees.common.Status.RUNNING

        ok = self._future.result()
        if ok:
            log.info("[RFBellNotify] SUCCESS bell_id=%s", self._bell_id)
            return py_trees.common.Status.SUCCESS
        else:
            log.warning("[RFBellNotify] FAILURE bell_id=%s (시리얼/HTTP 오류)",
                        self._bell_id)
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status: py_trees.common.Status) -> None:
        if self._executor is not None:
            self._executor.shutdown(wait=False)
            self._executor = None
        self._future = None

    def _call_notify(self, bell_id: str) -> bool:
        """rf_manager POST /notify 호출 (별도 스레드). True=성공, False=실패."""
        try:
            import requests
            r = requests.post(
                f"{self._bundle.rf_base_url}/notify",
                json={"bell_id": bell_id},
                timeout=self._HTTP_TIMEOUT,
            )
            result = r.json()
            return bool(result.get("ok", False))
        except Exception as e:
            log.error("[RFBellNotify] HTTP 오류: %s", e)
            return False


# ─────────────────────────────────────────────────────────────────────────────
# BT 스피커 Action 노드
# speaker_manager(:8083) HTTP API 경유 — fire-and-forget, 항상 SUCCESS
# ─────────────────────────────────────────────────────────────────────────────

class ActionWaitSpeakerIdle(py_trees.behaviour.Behaviour):
    """speaker_manager HIGH(TTS/alert) 재생과 큐가 모두 비워질 때까지 대기."""

    def __init__(
        self,
        bundle: ServiceBundle,
        timeout: float = 45.0,
        poll_interval: float = 0.25,
    ) -> None:
        super().__init__(name="[A] WaitSpeakerIdle")
        self._bundle = bundle
        self._timeout = max(float(timeout), 0.0)
        self._poll_interval = max(float(poll_interval), 0.05)
        self._start = 0.0
        self._next_poll = 0.0
        self._last_status: dict = {}

    def initialise(self) -> None:
        self._start = time.monotonic()
        self._next_poll = 0.0
        self._last_status = {}

    def update(self) -> py_trees.common.Status:
        now = time.monotonic()
        if now - self._start >= self._timeout:
            log.warning("[WaitSpeakerIdle] timeout status=%s", self._last_status)
            return py_trees.common.Status.SUCCESS
        if now < self._next_poll:
            return py_trees.common.Status.RUNNING
        self._next_poll = now + self._poll_interval
        try:
            status = _speaker_status(self._bundle, timeout=1.0)
            self._last_status = status
            if _speaker_high_idle(status):
                log.info("[WaitSpeakerIdle] high idle")
                return py_trees.common.Status.SUCCESS
        except Exception as exc:
            log.warning("[WaitSpeakerIdle] status failed: %s", exc)
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


class ActionSetMicMute(py_trees.behaviour.Behaviour):
    """mic_manager(:8082) mute/unmute 제어 노드."""

    def __init__(self, bundle: ServiceBundle, muted: bool, timeout: float = 2.0) -> None:
        super().__init__(name="[A] MicMute" if muted else "[A] MicUnmute")
        self._bundle = bundle
        self._muted = muted
        self._timeout = timeout
        self._executor: ThreadPoolExecutor | None = None
        self._future: Future | None = None

    def initialise(self) -> None:
        self._executor = ThreadPoolExecutor(
            max_workers=1, thread_name_prefix="mic-mute"
        )
        self._future = self._executor.submit(self._call_mic)

    def update(self) -> py_trees.common.Status:
        if self._future and self._future.done():
            try:
                result = self._future.result()
                log.info("[MicMute] muted=%s result=%s", self._muted, result)
            except Exception as exc:
                log.warning("[MicMute] HTTP failed muted=%s: %s", self._muted, exc)
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None:
        if self._executor is not None:
            self._executor.shutdown(wait=False)
            self._executor = None
        self._future = None

    def _call_mic(self) -> dict:
        return _post_mic_mute(self._bundle, self._muted, self._timeout)


class ActionPlayBGM(py_trees.behaviour.Behaviour):
    """
    BT 스피커 BGM 재생 요청 노드.

    POST /play {"file": file, "type": "bgm", "loop": loop} — fire-and-forget.
    HTTP 호출 결과(성공/실패)와 무관하게 항상 SUCCESS.
    오디오 실패가 시나리오 전체를 중단시키지 않기 위함.

    파라미터:
        file   : 재생할 오디오 파일 절대 경로 (.wav)
        bundle : ServiceBundle (speaker_base_url 포함)
        loop   : True(기본) — 루프 재생
    """

    _HTTP_TIMEOUT = 5.0

    def __init__(
        self,
        file: str,
        bundle: ServiceBundle,
        loop: bool = True,
    ) -> None:
        label = Path(file).name
        super().__init__(name=f"[A] PlayBGM({label})")
        self._file   = file
        self._bundle = bundle
        self._loop   = loop
        self._executor: ThreadPoolExecutor | None = None
        self._future:   Future | None = None

    def initialise(self) -> None:
        self._executor = ThreadPoolExecutor(
            max_workers=1, thread_name_prefix="play-bgm"
        )
        self._future = self._executor.submit(self._call_play)

    def update(self) -> py_trees.common.Status:
        if self._future and self._future.done():
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None:
        if self._executor is not None:
            self._executor.shutdown(wait=False)
            self._executor = None
        self._future = None

    def _call_play(self) -> None:
        """speaker_manager POST /play 호출 (별도 스레드)."""
        try:
            import requests
            requests.post(
                f"{self._bundle.speaker_base_url}/play",
                json={"file": self._file, "type": "bgm", "loop": self._loop},
                timeout=self._HTTP_TIMEOUT,
            )
        except Exception as exc:
            log.warning("[PlayBGM] HTTP 실패 — %s", exc)


class ActionPlayTTS(py_trees.behaviour.Behaviour):
    """
    BT 스피커 TTS 파일 재생 요청 노드.

    POST /play {"file": file, "type": "tts"} — fire-and-forget.
    항상 SUCCESS. ActionSpeak(AI TTS)와 구분 — 이 노드는 BT 스피커 출력 전용.

    파라미터:
        file   : 재생할 TTS .wav 파일 절대 경로
        bundle : ServiceBundle (speaker_base_url 포함)
    """

    _HTTP_TIMEOUT = 5.0

    def __init__(
        self,
        file: str,
        bundle: ServiceBundle,
    ) -> None:
        label = Path(file).name
        super().__init__(name=f"[A] PlayTTS({label})")
        self._file   = file
        self._bundle = bundle
        self._executor: ThreadPoolExecutor | None = None
        self._future:   Future | None = None

    def initialise(self) -> None:
        self._executor = ThreadPoolExecutor(
            max_workers=1, thread_name_prefix="play-tts"
        )
        self._future = self._executor.submit(self._call_play)

    def update(self) -> py_trees.common.Status:
        if self._future and self._future.done():
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None:
        if self._executor is not None:
            self._executor.shutdown(wait=False)
            self._executor = None
        self._future = None

    def _call_play(self) -> None:
        """speaker_manager POST /play 호출 (별도 스레드)."""
        try:
            import requests
            requests.post(
                f"{self._bundle.speaker_base_url}/play",
                json={"file": self._file, "type": "tts", "loop": False},
                timeout=self._HTTP_TIMEOUT,
            )
        except Exception as exc:
            log.warning("[PlayTTS] HTTP 실패 — %s", exc)


class ActionStopAudio(py_trees.behaviour.Behaviour):
    """
    BT 스피커 오디오 정지 노드.

    POST /stop {"type": stop_type} — HTTP 완료를 기다리지 않고 즉시 SUCCESS.

    파라미터:
        bundle    : ServiceBundle (speaker_base_url 포함)
        stop_type : "bgm"(기본) | "all"
    """

    _HTTP_TIMEOUT = 5.0

    def __init__(
        self,
        bundle: ServiceBundle,
        stop_type: str = "bgm",
    ) -> None:
        super().__init__(name=f"[A] StopAudio({stop_type})")
        self._bundle    = bundle
        self._stop_type = stop_type
        self._executor: ThreadPoolExecutor | None = None

    def initialise(self) -> None:
        self._executor = ThreadPoolExecutor(
            max_workers=1, thread_name_prefix="stop-audio"
        )
        self._executor.submit(self._call_stop)

    def update(self) -> py_trees.common.Status:
        # fire-and-forget: HTTP 완료 대기 없음 → 즉시 SUCCESS
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: py_trees.common.Status) -> None:
        if self._executor is not None:
            self._executor.shutdown(wait=False)
            self._executor = None

    def _call_stop(self) -> None:
        """speaker_manager POST /stop 호출 (별도 스레드)."""
        try:
            import requests
            requests.post(
                f"{self._bundle.speaker_base_url}/stop",
                json={"type": self._stop_type},
                timeout=self._HTTP_TIMEOUT,
            )
        except Exception as exc:
            log.warning("[StopAudio] HTTP 실패 — %s", exc)


# ─────────────────────────────────────────────────────────────────────────────
# Config Publish 노드 (mqtt-config-publish)
# 트리에 연결하지 않고 독립 사용 가능. 향후 설정 변경 이벤트 트리에서 사용 예정.
# ─────────────────────────────────────────────────────────────────────────────

class ActionPublishDestinations(py_trees.behaviour.Behaviour):
    """목적지 목록을 robot/config/destinations 토픽으로 발행.

    WaypointManager.list()로 전체 waypoint를 읽어
    bell_id를 제외한 UI용 payload를 구성하여 send_to_ui()로 전송한다.

    트리 미연결 — 향후 설정 변경 이벤트 트리에 편입 예정.
    """

    def __init__(self, bb: "RobotBlackboard", bundle: "ServiceBundle"):
        super().__init__(name=f"[A] PubDestinations")
        self._bb = bb
        self._bundle = bundle

    def update(self) -> py_trees.common.Status:
        if self._bundle.waypoint_mgr is None:
            log.warning("[PublishDestinations] waypoint_mgr is None")
            return py_trees.common.Status.FAILURE

        wps = self._bundle.waypoint_mgr.list()
        destinations = [
            {
                "key": wp.key,
                "label": wp.label,
                "type": wp.type,
                "x": wp.x,
                "y": wp.y,
                "theta": wp.theta,
                "comment": wp.comment,
            }
            for wp in wps
        ]
        ts = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        self._bundle.send_to_ui({
            "type": "config_destinations",
            "payload": {
                "destinations": destinations,
                "count": len(destinations),
                "timestamp": ts,
            },
        })
        log.info("[PublishDestinations] published %d destinations", len(destinations))
        return py_trees.common.Status.SUCCESS


class ActionPublishMorningCallSchedule(py_trees.behaviour.Behaviour):
    """모닝콜 스케줄을 robot/config/morning-call-schedule 토픽으로 발행.

    bundle.feature_cfg["morning_call"]에서 스케줄 데이터를 읽어
    send_to_ui()로 전송한다.

    트리 미연결 — 향후 설정 변경 이벤트 트리에 편입 예정.
    """

    def __init__(self, bb: "RobotBlackboard", bundle: "ServiceBundle"):
        super().__init__(name=f"[A] PubMorningCallSchedule")
        self._bb = bb
        self._bundle = bundle

    def update(self) -> py_trees.common.Status:
        mc_cfg = self._bundle.feature_cfg.get("morning_call", {})
        if not mc_cfg:
            log.warning("[PublishSchedule] morning_call config is empty")
            return py_trees.common.Status.FAILURE

        visits = mc_cfg.get("visits", [])
        visit_count = len(visits)
        date = time.strftime("%Y-%m-%d", time.gmtime())
        self._bundle.send_to_ui({
            "type": "config_schedule",
            "payload": {
                    "schedule_date" : date,
                    "operating_hours": mc_cfg.get("operating_hours", {}),
                    "visits": visits,
                    "max_door_retry": mc_cfg.get("max_door_retry", -1),
                    "door_wait_sec": mc_cfg.get("door_wait_sec", -1.0),
            },
        })
        
        log.info("[PublishSchedule] published schedule (%d visits)",
                 len(mc_cfg.get("visits", [])))
        return py_trees.common.Status.SUCCESS
