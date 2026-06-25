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

import logging
import re
import subprocess
import time
from concurrent.futures import Future, ThreadPoolExecutor
from pathlib import Path
from typing import Callable

import py_trees

from .blackboard import RobotBlackboard
from .bridge import ServiceBundle
from ..utils.event_code import EventCode

log = logging.getLogger(__name__)


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

    def initialise(self) -> None:
        self._start       = time.monotonic()
        self._retry_count = 0
        self._retry_at    = 0.0
        self._send_nav()

    def _send_nav(self) -> None:
        key   = self._resolve_key(self._waypoint_key)
        coord = self._bundle.waypoints.get(key,
                    {"x": 0.0, "y": 0.0, "theta": 0.0})
        if key not in self._bundle.waypoints:
            log.warning("[NavTo] unknown waypoint '%s', using origin", key)
        self._bundle.amr.send_target_position(coord)
        log.info("[NavTo] → '%s' %s (retry=%d)", key, coord, self._retry_count)

    def update(self) -> py_trees.common.Status:
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
    TTS 발화 — ai.call('/api/tts/speak') 를 별도 스레드에서 비동기 실행.

    text_template 치환 규칙 (re 기반):
      "{ctx_key}"    → bb.ctx 에서 우선 조회
      "{param_key}"  → bb.scenario_params 에서 조회
      키가 없으면 원문 유지

    update:
      future 완료      → SUCCESS
      elapsed > timeout → SUCCESS (발화 실패해도 다음 스텝 진행)
    """

    def __init__(self, text_template: str, bb: RobotBlackboard,
                 bundle: ServiceBundle, timeout: float = 10.0) -> None:
        label = text_template[:18] + "…" if len(text_template) > 18 else text_template
        super().__init__(name=f"[A] Speak({label})")
        self._template = text_template
        self._bb       = bb
        self._bundle   = bundle
        self._timeout  = timeout
        self._executor: ThreadPoolExecutor | None = None
        self._future:   Future | None = None
        self._start: float = 0.0

    def initialise(self) -> None:
        self._start    = time.monotonic()
        text           = self._resolve(self._template)
        self._executor = ThreadPoolExecutor(max_workers=1,
                                            thread_name_prefix="speak")
        self._future   = self._executor.submit(
            self._bundle.ai.call,
            "/api/tts/speak", "POST", {"text": text}, self._timeout,
        )
        log.info("[Speak] → '%s'", text)

    def update(self) -> py_trees.common.Status:
        if self._future is not None and self._future.done():
            return py_trees.common.Status.SUCCESS
        if time.monotonic() - self._start >= self._timeout:
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
            v = self._bb.ctx.get(k) or self._bb.scenario_params.get(k)
            return str(v) if v is not None else m.group(0)
        return re.sub(r"\{(\w+)\}", sub, template)


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
    ai.call('/api/face/recognize') 비동기.

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
        self._future   = self._executor.submit(
            self._bundle.ai.call,
            "/api/face/recognize", "POST", None, self._timeout,
        )
        log.debug("[FaceRecog] started")

    def update(self) -> py_trees.common.Status:
        if self._future is not None and self._future.done():
            resp = self._future.result()
            if resp and resp.get("name"):
                self._bb.ctx["recognized_name"] = resp["name"]
                self._bb.ctx["recognized_unit"] = resp.get("unit", "")
                log.info("[FaceRecog] → %s / unit=%s",
                         resp["name"], resp.get("unit"))
                return py_trees.common.Status.SUCCESS
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
    전체 waypoint 이동 완료 시 SUCCESS.
    """

    def __init__(self, bb: RobotBlackboard, bundle: ServiceBundle,
                 timeout_per_wp: float = 300.0) -> None:
        super().__init__(name="[A] NavWaypoints")
        self._bb             = bb
        self._bundle         = bundle
        self._timeout_per_wp = timeout_per_wp
        self._wp_list: list[str] = []
        self._wp_start: float    = 0.0
        self._repeat = 0
        try:
            self._repeat = int(self._bb.scenario_params.get("repeat") or 0)
        except (TypeError, ValueError):
            self._repeat = 0

    def initialise(self) -> None:
        self._wp_list = list(self._bb.scenario_params.get("waypoints", []))
        wp_list_dump = self._wp_list
        
        try:
            self._repeat = int(self._bb.scenario_params.get("repeat") or 0)
        except (TypeError, ValueError):
            self._repeat = 0
        
        # Repeat count set
        for i in range(self._repeat):
            self._wp_list = self._wp_list + wp_list_dump
        
        self._bb.ctx["wp_index"] = 0
        log.info("[NavWaypoints] waypoints: %s", self._wp_list)
        if self._wp_list:
            self._navigate_to(0)

    def update(self) -> py_trees.common.Status:
        idx = self._bb.ctx.get("wp_index", 0)
        if idx >= len(self._wp_list):
            return py_trees.common.Status.SUCCESS

        arrived = self._bb.amr_arrived
        timeout = (time.monotonic() - self._wp_start) >= self._timeout_per_wp
        if arrived or timeout:
            if timeout:
                log.warning("[NavWaypoints] wp[%d] timeout → next", idx)
            else:
                log.info("[NavWaypoints] wp[%d] arrived ✓", idx)
            idx += 1
            self._bb.ctx["wp_index"] = idx
            if idx >= len(self._wp_list):
                log.info("[NavWaypoints] all waypoints done")
                return py_trees.common.Status.SUCCESS
            self._navigate_to(idx)

        return py_trees.common.Status.RUNNING

    def _navigate_to(self, idx: int) -> None:
        key = self._wp_list[idx]
        # WaypointManager 우선, 없으면 기존 dict 폴백
        coord: dict = {"x": 0.0, "y": 0.0, "theta": 0.0}
        if self._bundle.waypoint_mgr is not None:
            wp = self._bundle.waypoint_mgr.get(key)
            if wp:
                coord = {"x": wp.x, "y": wp.y, "theta": wp.theta}
        else:
            coord = self._bundle.waypoints.get(key, coord)
        self._bundle.amr.send_target_position(coord)
        self._wp_start = time.monotonic()
        log.info("[NavWaypoints] → wp[%d]='%s' %s", idx, key, coord)


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
