from __future__ import annotations

import json
import logging
import queue
import socket
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import dataclass, field
from typing import Any

from ..interfaces.pubsub import Publisher

log = logging.getLogger(__name__)


@dataclass
class AIService:
    """
    Jetson AI service adapter used by BT nodes.

    Injected events remain available for debug and demo control, while normal
    operation reads the local Face, Agent Events, Fall Detection, and TTS APIs.
    """
    fall_status_url: str = "http://192.168.31.167:8008/api/fall-status"
    fall_status_timeout: float = 0.25
    fall_status_cache_sec: float = 0.05
    agent_events_base_url: str = "http://127.0.0.1:8086"
    agent_events_event_name: str = "agent.wake_wait_started"
    agent_events_timeout: float = 2.0
    conversation_wait_timeout: float = 45.0
    conversation_poll_interval: float = 0.5
    face_api_base_url: str = "http://127.0.0.1:8087"
    face_api_timeout: float = 1.0
    tts_api_base_url: str = "http://127.0.0.1:8085"
    tts_poll_interval: float = 0.25
    tts_done_timeout: float = 20.0
    agent_events_baseline_on_start: bool = True
    agent_events_baseline_on_conversation_start: bool = True
    conversation_followup_grace_sec: float = 15.0
    fall_candidate_min_hits: int = 2
    fall_candidate_window_sec: float = 2.0
    fall_candidate_requires_clear: bool = True
    fall_cooldown_sec: float = 30.0
    wander_min_count: int = 3
    wander_window_sec: float = 60.0
    wander_cooldown_sec: float = 30.0
    wander_unknown_only: bool = True
    door_open_min_hits: int = 2

    def __post_init__(self) -> None:
        self._started  = False
        self._stop     = threading.Event()
        self._thread:  threading.Thread | None = None
        self._pub:     Publisher | None = None
        self._rx:      queue.Queue[dict] = queue.Queue()  # AI 이벤트 수신 큐
        self._fall_status_cache: dict | None = None
        self._fall_status_cache_at: float = 0.0
        self._agent_event_after_id: int = 0
        self._fall_candidate_hits: int = 0
        self._fall_candidate_first_at: float = 0.0
        self._fall_candidate_armed: bool = not self.fall_candidate_requires_clear
        self._fall_suppressed_until: float = 0.0
        self._door_open_hits: int = 0
        self._wander_last_face_seq: int = 0
        self._wander_sightings: list[dict] = []
        self._wander_suppressed_until: float = 0.0

    @property
    def started(self) -> bool:
        return self._started

    def start(self) -> None:
        if self._started:
            return
        self._stop.clear()
        if self.agent_events_baseline_on_start:
            self._init_agent_event_baseline()
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
        debug inject_event() 경로와 외부 수신 스레드가 같은 큐를 사용한다.
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
        온디맨드 AI API 호출.

        레거시 BT 노드가 /api/face/recognize를 호출하면 canonical Face API
        (/v1/face/latest)를 조회한 recognize_face() 결과로 호환 응답한다.
        """
        if endpoint in ("/api/face/recognize", "/v1/face/latest"):
            return self.recognize_face()
        log.debug("[AIService] call unsupported: %s %s", method, endpoint)
        return None

    def detect_fall(self) -> dict:
        """낙상 감지 결과를 반환한다. 주입 이벤트가 있으면 우선 사용한다."""
        ev = self._pop_event_by_type("fall_detected")
        if ev is not None:
            return {
                "detected": bool(ev.get("detected", True)),
                "candidate": bool(ev.get("candidate", ev.get("fall_candidate", False))),
                "confidence": float(ev.get("confidence", 0.90)),
                "image_path": str(ev.get("image_path", "")),
                "position": ev.get("position", {}),
                "status": str(ev.get("status", ev.get("fall_status", ""))),
                "timestamp": str(ev.get("timestamp", "")),
            }

        if time.monotonic() < self._fall_suppressed_until:
            return {
                "detected": False,
                "candidate": False,
                "fall_detected": False,
                "fall_candidate": False,
                "confidence": 0.0,
                "image_path": "",
                "position": {},
                "status": "cooldown",
                "timestamp": "",
                "suppressed": True,
            }

        api_result = self._fetch_fall_status()
        if api_result is not None:
            return api_result

        return {
            "detected": False,
            "candidate": False,
            "confidence": 0.0,
            "image_path": "",
            "position": {},
            "status": "unknown",
            "timestamp": "",
        }

    def detect_wander(self, person_id: str = "") -> dict:
        """Return repeated-person detection based on Face API recent snapshots."""
        ev = self._pop_event_by_type("wander_detected") or self._pop_event_by_type("person_detected_repeated")
        if ev is not None:
            return {
                "repeated": bool(ev.get("repeated", True)),
                "person_id": str(ev.get("person_id", person_id or "P001")),
                "count": int(ev.get("count", 2)),
                "duration_min": float(ev.get("duration_min", 15.0)),
                "image_path": str(ev.get("image_path", "")),
            }
        api_result = self._fetch_face_recent_for_wander(person_id)
        if api_result is not None:
            return api_result
        return {
            "repeated": False,
            "person_id": person_id,
            "count": 0,
            "duration_min": 0.0,
            "image_path": "",
        }

    def detect_door_open(self) -> dict:
        """Detect that a resident came out by checking Face API presence."""
        ev = self._pop_event_by_type("door_opened")
        if ev is not None:
            return {
                "opened": bool(ev.get("opened", True)),
                "confidence": float(ev.get("confidence", 0.90)),
            }
        face = self._fetch_face_latest()
        if face is None:
            self._door_open_hits = 0
            return {"opened": False, "confidence": 0.0, "source": "face_api_unavailable"}

        present = (
            not bool(face.get("stale", False))
            and (
                bool(face.get("recognized", False))
                or int(face.get("unknown_count", 0) or 0) > 0
                or int(face.get("known_count", 0) or 0) > 0
            )
        )
        if present:
            self._door_open_hits += 1
        else:
            self._door_open_hits = 0

        min_hits = max(int(self.door_open_min_hits or 1), 1)
        return {
            "opened": self._door_open_hits >= min_hits,
            "confidence": float(face.get("confidence", 0.0) or 0.0),
            "hits": self._door_open_hits,
            "min_hits": min_hits,
            "recognized": bool(face.get("recognized", False)),
            "unknown_count": int(face.get("unknown_count", 0) or 0),
            "known_count": int(face.get("known_count", 0) or 0),
            "stale": bool(face.get("stale", False)),
            "source": face.get("source", "face_api"),
        }

    def recognize_face(self) -> dict:
        """Face API 최신 snapshot을 조회해 얼굴 인식 결과를 반환한다."""
        ev = self._pop_event_by_type("face_recognized")
        if ev is not None:
            return {
                "name": str(ev.get("name", "데모 사용자")),
                "unit": str(ev.get("unit", "")),
                "confidence": float(ev.get("confidence", 0.90)),
                "recognized": bool(ev.get("recognized", True)),
                "source": "injected_event",
            }

        api_result = self._fetch_face_latest()
        if api_result is not None:
            return api_result

        return {
            "name": "",
            "unit": "",
            "confidence": 0.0,
            "recognized": False,
            "stale": True,
            "unknown_count": 0,
            "summary_text": "",
            "source": "face_api_unavailable",
        }

    def start_conversation(self, conversation_type: str = "", context: dict | None = None) -> dict:
        """자유대화 결과를 반환한다."""
        ev = self._pop_event_by_type("conversation_result")
        if ev is not None:
            return {
                "text": str(ev.get("text", "")),
                "audio_path": str(ev.get("audio_path", "")),
            }
        if conversation_type == "morning_greeting":
            if self.agent_events_baseline_on_conversation_start:
                self._init_agent_event_baseline()
            agent_event = self._wait_for_agent_wake_event(confirm_end=True)
            if agent_event is not None:
                return {
                    "text": str(agent_event.get("last_tts_text", "") or ""),
                    "audio_path": "",
                    "last_user_text": str(agent_event.get("last_user_text", "") or ""),
                    "end_reason": str(agent_event.get("end_reason", "") or ""),
                    "conversation_id": str(agent_event.get("conversation_id", "") or ""),
                    "event_id": int(agent_event.get("id", 0) or 0),
                    "source": "agent_events",
                    "pipeline_state": str(agent_event.get("pipeline_state", "") or ""),
                    "stream_running": bool(agent_event.get("stream_running", False)),
                }
            return {
                "text": "",
                "audio_path": "",
                "last_user_text": "",
                "end_reason": "timeout",
                "conversation_id": "",
                "event_id": self._agent_event_after_id,
                "source": "agent_events",
            }
        elif conversation_type == "fall_status_check":
            if self.agent_events_baseline_on_conversation_start:
                self._init_agent_event_baseline()
            agent_event = self._wait_for_agent_wake_event()
            if agent_event is not None:
                last_user_text = str(agent_event.get("last_user_text", "") or "")
                last_tts_text = str(agent_event.get("last_tts_text", "") or "")
                intent = self._classify_fall_status_intent(last_user_text, str(agent_event.get("end_reason", "") or ""))
                return {
                    "text": last_tts_text,
                    "audio_path": "",
                    "last_user_text": last_user_text,
                    "end_reason": str(agent_event.get("end_reason", "") or ""),
                    "conversation_id": str(agent_event.get("conversation_id", "") or ""),
                    "event_id": int(agent_event.get("id", 0) or 0),
                    "source": "agent_events",
                    "pipeline_state": str(agent_event.get("pipeline_state", "") or ""),
                    "stream_running": bool(agent_event.get("stream_running", False)),
                    "intent": intent,
                }
            return {
                "text": "",
                "audio_path": "",
                "last_user_text": "",
                "end_reason": "timeout",
                "conversation_id": "",
                "event_id": self._agent_event_after_id,
                "source": "agent_events",
                "intent": "timeout",
            }
        resident_name = ""
        if isinstance(context, dict):
            resident_name = str(context.get("resident_name", "") or "")
        prefix = f"{resident_name}님, " if resident_name else ""
        if conversation_type == "program_schedule":
            text = f"{prefix}오늘 오전에는 아침 체조와 건강 상담 일정이 있습니다."
        else:
            text = f"{prefix}좋은 아침입니다. 오늘도 건강한 하루 보내세요."
        return {"text": text, "audio_path": ""}

    def begin_conversation_session(self) -> int:
        """Start a Core-managed conversation window without changing global timeouts."""
        if self.agent_events_baseline_on_conversation_start:
            self._init_agent_event_baseline()
        return self._agent_event_after_id

    def wait_conversation_event(self, *, wait_timeout: float = 1.0) -> dict | None:
        """Poll one fresh Agent Events conversation event for stateful BT nodes."""
        return self._wait_for_agent_wake_event(
            confirm_end=False,
            wait_timeout=wait_timeout,
        )

    def wait_tts_job_done(self, job_id: str) -> dict:
        """Poll the TTS queue API until the job reaches a terminal state."""
        job_id = str(job_id or "").strip()
        if not job_id:
            return {"status": "skipped", "reason": "no_job_id"}

        base_url = self.tts_api_base_url.rstrip("/")
        if not base_url:
            return {"status": "skipped", "reason": "no_tts_api_base_url"}

        url = f"{base_url}/v1/tts/jobs/{urllib.parse.quote(job_id)}"
        deadline = time.monotonic() + max(float(self.tts_done_timeout or 0.0), 1.0)
        while time.monotonic() < deadline and not self._stop.is_set():
            try:
                raw = self._get_json(url, 2.0)
                status = str(raw.get("status", "") or "").lower()
                if status in ("done", "completed", "complete", "finished", "success", "succeeded"):
                    return {
                        "status": "done",
                        "metrics": raw.get("metrics", {}),
                        "finished_at": raw.get("finished_at", ""),
                    }
                if bool(raw.get("done", False)) or bool(raw.get("completed", False)):
                    return {
                        "status": "done",
                        "metrics": raw.get("metrics", {}),
                        "finished_at": raw.get("finished_at", ""),
                    }
                if raw.get("finished_at") and status not in ("queued", "pending", "running", "playing", "speaking"):
                    return {
                        "status": "done",
                        "metrics": raw.get("metrics", {}),
                        "finished_at": raw.get("finished_at", ""),
                    }
                if status in ("failed", "error", "cancelled"):
                    return {"status": "failed", "reason": str(raw.get("error", status))}
            except (urllib.error.URLError, TimeoutError, ValueError, json.JSONDecodeError) as exc:
                log.debug("[AIService] TTS job poll error url=%s: %s", url, exc)
            self._stop.wait(timeout=max(float(self.tts_poll_interval or 0.0), 0.1))

        log.warning("[AIService] TTS job wait timeout job_id=%s after %.1fs", job_id, self.tts_done_timeout)
        return {"status": "timeout", "job_id": job_id}

    def reset_patrol_detection_state(self) -> None:
        """Reset stateful fall/wander filters at patrol start."""
        self.reset_fall_candidate_state()
        self.reset_wander_detection_state()

    def reset_fall_candidate_state(self) -> None:
        """Require a clear fall-candidate baseline before the next trigger."""
        self._fall_candidate_hits = 0
        self._fall_candidate_first_at = 0.0
        self._fall_candidate_armed = not self.fall_candidate_requires_clear
        self._fall_suppressed_until = 0.0

    def mark_fall_detection_handled(self) -> None:
        """Apply cooldown after one fall detection flow has been handled."""
        self.reset_fall_candidate_state()
        self._fall_suppressed_until = time.monotonic() + max(float(self.fall_cooldown_sec or 0.0), 0.0)

    def reset_wander_detection_state(self) -> None:
        """Reset Face recent cursor so one detection is not emitted every tick."""
        self._wander_sightings = []
        latest = self._fetch_face_latest()
        if latest is not None:
            self._wander_last_face_seq = int(latest.get("seq", self._wander_last_face_seq) or 0)

    def mark_wander_detection_handled(self) -> None:
        """Apply a short cooldown after publishing one wander detection."""
        self.reset_wander_detection_state()
        self._wander_suppressed_until = time.monotonic() + max(float(self.wander_cooldown_sec or 0.0), 0.0)

    def reset_door_open_detection(self) -> None:
        """Reset consecutive door-open hits for a new visit."""
        self._door_open_hits = 0

    # ── 내부 이벤트 주입 헬퍼 (테스트 / AI 모듈 연동용) ─────────────

    def inject_event(self, event: dict) -> None:
        """외부(AI 수신 스레드 등)에서 이벤트를 주입한다."""
        self._rx.put_nowait(event)

    def _pop_event_by_type(self, event_type: str) -> dict | None:
        kept: list[dict] = []
        found: dict | None = None
        while True:
            try:
                ev = self._rx.get_nowait()
            except queue.Empty:
                break
            if found is None and ev.get("type") == event_type:
                found = ev
            else:
                kept.append(ev)
        for ev in kept:
            self._rx.put_nowait(ev)
        return found

    def _fetch_fall_status(self) -> dict | None:
        """Read the latest fall status from the external fall AI HTTP API."""
        if not self.fall_status_url:
            return None

        now = time.monotonic()
        if self._fall_status_cache is not None and now - self._fall_status_cache_at <= self.fall_status_cache_sec:
            return dict(self._fall_status_cache)

        try:
            req = urllib.request.Request(
                self.fall_status_url,
                headers={"Accept": "application/json"},
                method="GET",
            )
            with urllib.request.urlopen(req, timeout=self.fall_status_timeout) as resp:
                body = resp.read().decode("utf-8", errors="replace")
            raw = json.loads(body)
            if not isinstance(raw, dict):
                raise ValueError("fall-status response is not a JSON object")

            coordinates = raw.get("coordinates", {})
            if not isinstance(coordinates, dict):
                coordinates = {}
            detected = bool(raw.get("fall_detected", False))
            candidate = bool(raw.get("fall_candidate", False))
            filtered_candidate = self._update_fall_candidate_filter(candidate or detected)
            result = {
                "detected": detected,
                "candidate": filtered_candidate,
                "fall_detected": detected,
                "fall_candidate": filtered_candidate,
                "raw_candidate": candidate,
                "confidence": 1.0 if detected else 0.0,
                "image_path": "",
                "position": coordinates,
                "coordinates": coordinates,
                "status": str(raw.get("fall_status", "")),
                "fall_status": str(raw.get("fall_status", "")),
                "timestamp": str(raw.get("timestamp", "")),
            }
            self._fall_status_cache = dict(result)
            self._fall_status_cache_at = now
            return result
        except (urllib.error.URLError, TimeoutError, ValueError, json.JSONDecodeError) as exc:
            log.debug("[AIService] fall-status API unavailable url=%s: %s", self.fall_status_url, exc)
            return None

    def _fetch_face_latest(self) -> dict | None:
        """Read and normalize the latest snapshot from the canonical Face API."""
        base_url = self.face_api_base_url.rstrip("/")
        if not base_url:
            return None

        url = f"{base_url}/v1/face/latest"
        try:
            raw = self._get_json(url, self.face_api_timeout)
            if not isinstance(raw, dict):
                raise ValueError("face latest response is not a JSON object")
            return self._normalize_face_snapshot(raw)
        except (urllib.error.URLError, TimeoutError, ValueError, json.JSONDecodeError) as exc:
            log.debug("[AIService] face API unavailable url=%s: %s", url, exc)
            return None

    def _fetch_face_recent_for_wander(self, person_id: str = "") -> dict | None:
        """Poll Face recent snapshots and accumulate sightings in a time window."""
        if time.monotonic() < self._wander_suppressed_until:
            return {
                "repeated": False,
                "person_id": person_id,
                "count": 0,
                "duration_min": 0.0,
                "image_path": "",
                "suppressed": True,
            }

        base_url = self.face_api_base_url.rstrip("/")
        if not base_url:
            return None

        query = urllib.parse.urlencode({
            "after_seq": self._wander_last_face_seq,
            "limit": 50,
        })
        url = f"{base_url}/v1/face/recent?{query}"
        try:
            raw = self._get_json(url, self.face_api_timeout)
            snapshots = raw.get("snapshots", raw.get("items", raw.get("events", [])))
            if isinstance(snapshots, dict):
                snapshots = [snapshots]
            if not isinstance(snapshots, list):
                raise ValueError("face recent response does not contain a snapshot list")

            now = time.monotonic()
            for snapshot in snapshots:
                if not isinstance(snapshot, dict):
                    continue
                seq = int(snapshot.get("seq", 0) or 0)
                if seq <= self._wander_last_face_seq:
                    continue
                self._wander_last_face_seq = max(self._wander_last_face_seq, seq)
                if bool(snapshot.get("stale", False)):
                    continue

                presence = snapshot.get("presence", {})
                if not isinstance(presence, dict):
                    presence = {}
                unknown_count = int(presence.get("unknown_count", 0) or 0)
                known_persons = self._normalize_known_persons(presence.get("known_persons", []))
                if self.wander_unknown_only and unknown_count <= 0:
                    continue
                if not self.wander_unknown_only and unknown_count <= 0 and not known_persons:
                    continue

                self._wander_sightings.append({
                    "seq": seq,
                    "ts": now,
                    "unknown_count": unknown_count,
                    "known_persons": known_persons,
                    "summary_text": str(snapshot.get("summary_text", "") or ""),
                })

            window = max(float(self.wander_window_sec or 0.0), 1.0)
            cutoff = now - window
            self._wander_sightings = [
                item for item in self._wander_sightings
                if float(item.get("ts", 0.0) or 0.0) >= cutoff
            ]
            result = self._build_wander_result(person_id, window)
            return result
        except (urllib.error.URLError, TimeoutError, ValueError, json.JSONDecodeError) as exc:
            log.debug("[AIService] face recent API unavailable url=%s: %s", url, exc)
            return None

    def _normalize_face_snapshot(self, raw: dict) -> dict:
        """Convert Face API variants into the schema used by BT nodes."""
        stale = bool(raw.get("stale", False))
        presence = raw.get("presence", {})
        if not isinstance(presence, dict):
            presence = {}

        faces = raw.get("faces", [])
        if not faces:
            faces = raw.get("detections", raw.get("persons", raw.get("recognized_faces", [])))
        if not isinstance(faces, list):
            faces = []

        best_name = ""
        best_unit = ""
        best_conf = 0.0
        best_face: dict = {}
        for face in faces:
            if not isinstance(face, dict):
                continue
            name = self._first_str(
                face,
                "display_name",
                "displayName",
                "resident_name",
                "name",
                "label",
                "person_name",
                "person_id",
                "identity",
            )
            if not name or name.lower() in ("unknown", "none"):
                continue
            conf = self._first_float(face, "confidence", "score", "similarity", default=0.0)
            if conf >= best_conf:
                best_name = name
                best_unit = self._first_str(face, "unit", "room_id", "room", "resident_unit")
                best_conf = conf
                best_face = face

        known_persons = presence.get("known_persons", [])
        known_count = len(known_persons) if isinstance(known_persons, list) else 0
        known_count = max(known_count, int(presence.get("known_count", raw.get("known_count", 0)) or 0))
        if not best_name and isinstance(known_persons, list) and known_persons:
            first = known_persons[0]
            if isinstance(first, dict):
                best_name = self._first_str(
                    first,
                    "display_name",
                    "displayName",
                    "resident_name",
                    "name",
                    "label",
                    "person_name",
                    "person_id",
                )
                best_unit = self._first_str(first, "unit", "room_id", "room", "resident_unit")
                best_conf = self._first_float(first, "confidence", "score", default=best_conf)
            else:
                best_name = str(first or "")

        unknown_count = int(presence.get("unknown_count", raw.get("unknown_count", 0)) or 0)
        person_count = int(
            presence.get(
                "person_count",
                presence.get("face_count", raw.get("person_count", raw.get("face_count", 0))),
            )
            or 0
        )
        if not best_name and unknown_count == 0 and person_count > known_count:
            unknown_count = max(person_count - known_count, 0)
        recognized = bool(best_name) and not stale
        return {
            "name": best_name if recognized else "",
            "unit": best_unit,
            "confidence": best_conf,
            "recognized": recognized,
            "stale": stale,
            "unknown_count": unknown_count,
            "known_count": known_count,
            "summary_text": str(raw.get("summary_text", "") or ""),
            "seq": int(raw.get("seq", 0) or 0),
            "created_at": str(raw.get("created_at", "") or ""),
            "bbox": best_face.get("bbox", []) if isinstance(best_face, dict) else [],
            "bbox_norm": best_face.get("bbox_norm", []) if isinstance(best_face, dict) else [],
            "source": "face_api",
        }

    def _get_json(self, url: str, timeout: float) -> dict:
        req = urllib.request.Request(
            url,
            headers={"Accept": "application/json"},
            method="GET",
        )
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            body = resp.read().decode("utf-8", errors="replace")
        raw = json.loads(body)
        if not isinstance(raw, dict):
            raise ValueError("response is not a JSON object")
        return raw

    def _init_agent_event_baseline(self) -> None:
        """Set the Agent Events cursor to the latest event id."""
        base_url = self.agent_events_base_url.rstrip("/")
        if not base_url:
            return

        query = urllib.parse.urlencode({"event": self.agent_events_event_name})
        url = f"{base_url}/v1/agent/events/latest?{query}"
        try:
            raw = self._get_json(url, self.agent_events_timeout)
            event = raw.get("event", raw)
            if not isinstance(event, dict):
                return
            latest_id = int(event.get("id", 0) or 0)
            if latest_id > self._agent_event_after_id:
                self._agent_event_after_id = latest_id
                log.info("[AIService] agent event baseline set id=%d", latest_id)
        except (urllib.error.URLError, TimeoutError, ValueError, json.JSONDecodeError) as exc:
            log.warning("[AIService] agent event baseline init failed: %s", exc)

    def _update_fall_candidate_filter(self, candidate: bool) -> bool:
        now = time.monotonic()
        window = max(float(self.fall_candidate_window_sec or 0.0), 0.1)
        if not candidate:
            self._fall_candidate_hits = 0
            self._fall_candidate_first_at = 0.0
            self._fall_candidate_armed = True
            return False
        if not self._fall_candidate_armed:
            return False
        if self._fall_candidate_hits <= 0 or now - self._fall_candidate_first_at > window:
            self._fall_candidate_hits = 1
            self._fall_candidate_first_at = now
        else:
            self._fall_candidate_hits += 1
        return self._fall_candidate_hits >= max(int(self.fall_candidate_min_hits or 1), 1)

    def _normalize_known_persons(self, value: Any) -> list[str]:
        if not isinstance(value, list):
            return []
        names: list[str] = []
        for item in value:
            if isinstance(item, dict):
                name = self._first_str(
                    item,
                    "display_name",
                    "displayName",
                    "resident_name",
                    "name",
                    "label",
                    "person_name",
                    "person_id",
                    "identity",
                )
            else:
                name = str(item or "").strip()
            if name and name.lower() not in ("unknown", "none"):
                names.append(name)
        return names

    def _build_wander_result(self, person_id: str, window: float) -> dict:
        min_count = max(int(self.wander_min_count or 1), 1)
        unknown_count = sum(1 for item in self._wander_sightings if int(item.get("unknown_count", 0) or 0) > 0)
        known_counts: dict[str, int] = {}
        for item in self._wander_sightings:
            for name in item.get("known_persons", []) or []:
                known_counts[name] = known_counts.get(name, 0) + 1

        best_known = ""
        best_known_count = 0
        for name, count in known_counts.items():
            if count > best_known_count:
                best_known = name
                best_known_count = count

        if self.wander_unknown_only:
            count = unknown_count
            resolved_person_id = person_id or "unknown"
        elif best_known_count >= unknown_count:
            count = best_known_count
            resolved_person_id = person_id or best_known
        else:
            count = unknown_count
            resolved_person_id = person_id or "unknown"

        first_ts = min((float(item.get("ts", 0.0) or 0.0) for item in self._wander_sightings), default=time.monotonic())
        duration_min = max((time.monotonic() - first_ts) / 60.0, 0.0)
        return {
            "repeated": count >= min_count,
            "person_id": resolved_person_id,
            "count": count,
            "duration_min": duration_min,
            "window_sec": window,
            "image_path": "",
        }

    def _classify_fall_status_intent(self, text: str, end_reason: str = "") -> str:
        lower = str(text or "").lower()
        if any(token in lower for token in ("괜찮", "괜찬", "괜차", "문제없", "괜찮아", "fine", "ok", "okay")):
            return "ok"
        if any(token in lower for token in ("도와", "도움", "살려", "아파", "못", "119", "help", "hurt", "pain")):
            return "help"
        if str(end_reason or "").lower() == "timeout":
            return "timeout"
        return "unknown"

    def _first_str(self, data: dict, *keys: str) -> str:
        for key in keys:
            value = data.get(key)
            if value is not None and str(value).strip():
                return str(value).strip()
        return ""

    def _first_float(self, data: dict, *keys: str, default: float = 0.0) -> float:
        for key in keys:
            value = data.get(key)
            if value is None:
                continue
            try:
                return float(value)
            except (TypeError, ValueError):
                continue
        return default

    def _wait_for_agent_wake_event(
        self,
        *,
        confirm_end: bool = False,
        wait_timeout: float | None = None,
    ) -> dict | None:
        """대화 종료 후 Wake 대기 복귀 이벤트를 polling한다."""
        timeout_sec = (
            self.conversation_wait_timeout
            if wait_timeout is None
            else float(wait_timeout or 0.0)
        )
        deadline = time.monotonic() + max(timeout_sec, 0.0)
        candidate: dict | None = None
        candidate_at = 0.0
        grace_sec = max(float(self.conversation_followup_grace_sec or 0.0), 0.0)
        while time.monotonic() < deadline and not self._stop.is_set():
            now = time.monotonic()
            event = self._fetch_next_agent_wake_event()
            if event is not None:
                end_reason = str(event.get("end_reason", "") or "").lower()
                if self._is_excluded_agent_event(end_reason):
                    log.info(
                        "[AIService] agent event ignored id=%s reason=%s",
                        event.get("id"),
                        end_reason,
                    )
                    continue
                log.info(
                    "[AIService] agent event received id=%s reason=%s",
                    event.get("id"),
                    end_reason,
                )
                if not confirm_end or end_reason == "explicit_close":
                    return event
                candidate = event
                candidate_at = now
            if candidate is not None and now - candidate_at >= grace_sec:
                log.info(
                    "[AIService] agent event confirmed id=%s reason=%s grace=%.1fs",
                    candidate.get("id"),
                    candidate.get("end_reason"),
                    grace_sec,
                )
                return candidate
            self._stop.wait(timeout=max(self.conversation_poll_interval, 0.1))
        if candidate is not None:
            log.info(
                "[AIService] agent event confirmed by timeout id=%s reason=%s",
                candidate.get("id"),
                candidate.get("end_reason"),
            )
            return candidate
        if wait_timeout is None:
            log.warning(
                "[AIService] agent event wait timeout after %.1fs",
                timeout_sec,
            )
        else:
            log.debug(
                "[AIService] agent event poll timeout after %.1fs",
                timeout_sec,
            )
        return None

    def _is_excluded_agent_event(self, end_reason: str) -> bool:
        return str(end_reason or "").lower() in {
            "tts_preview_direct",
            "flow_reset",
            "runtime_close",
            "stream_stopped",
            "component_session_override",
        }

    def _fetch_next_agent_wake_event(self) -> dict | None:
        """Agent Events API에서 after_id 이후 새 이벤트 1건을 읽는다."""
        base_url = self.agent_events_base_url.rstrip("/")
        if not base_url:
            return None

        query = urllib.parse.urlencode({
            "after_id": self._agent_event_after_id,
            "limit": 20,
            "event": self.agent_events_event_name,
        })
        url = f"{base_url}/v1/agent/events/recent?{query}"
        try:
            req = urllib.request.Request(
                url,
                headers={"Accept": "application/json"},
                method="GET",
            )
            with urllib.request.urlopen(req, timeout=self.agent_events_timeout) as resp:
                body = resp.read().decode("utf-8", errors="replace")
            raw = json.loads(body)
            if not isinstance(raw, dict):
                raise ValueError("agent-events response is not a JSON object")
            events = raw.get("events", [])
            if not isinstance(events, list) or not events:
                return None

            normalized: list[dict] = []
            for item in events:
                if isinstance(item, dict):
                    normalized.append(item)
            if not normalized:
                return None

            normalized.sort(key=lambda item: int(item.get("id", 0) or 0))
            latest = normalized[-1]
            latest_id = int(latest.get("id", 0) or 0)
            if latest_id > self._agent_event_after_id:
                self._agent_event_after_id = latest_id
            return latest
        except (urllib.error.URLError, TimeoutError, ValueError, json.JSONDecodeError) as exc:
            log.debug("[AIService] agent-events API unavailable url=%s: %s", url, exc)
            return None

    # ── 수신 루프 ───────────────────────────────────────────────────

    def _run(self) -> None:
        """AI service background loop placeholder for future push integrations."""
        while not self._stop.is_set():
            try:
                # TODO: Jetson AI 모듈 연동 구현
                self._stop.wait(timeout=1.0)
            except Exception as e:
                log.error("[AIService] _run error: %s", e)


# 외부에서 JetsonAiService 이름으로도 import 가능하도록 alias 추가
JetsonAiService = AIService
