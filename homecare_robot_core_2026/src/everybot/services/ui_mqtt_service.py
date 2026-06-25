from __future__ import annotations

import json
import logging
import queue
import subprocess
import time
from dataclasses import dataclass

try:
    import paho.mqtt.client as paho
except ModuleNotFoundError:
    paho = None  # type: ignore

log = logging.getLogger(__name__)


@dataclass(frozen=True)
class UiMqttConfig:
    """OrangePi ↔ Jetson 로컬 MQTT 서비스 설정.

    모든 토픽/라우팅 값은 YAML에서 주입 가능 (hardcode 없음).
    """
    enabled: bool = False
    host: str = "127.0.0.1"              # Jetson 로컬 mosquitto 브로커
    port: int = 1883
    client_id: str = "Everybot-ui-client"
    keepalive: int = 30

    # Robot → UI 발행 토픽
    topic_status: str = "robot/status"
    topic_location: str = "robot/location"
    topic_move_status: str = "robot/move/status"
    topic_detection: str = "robot/detection"
    topic_patrol: str = "robot/patrol"
    topic_config_destinations: str = "robot/config/destinations"
    topic_morning_call_event: str = "robot/morning-call/event"
    topic_morning_call_door: str = "robot/morning-call/door"
    topic_config_schedule: str = "robot/config/morning-call-schedule"

    # UI → Robot 구독 토픽
    subscribe_topics: tuple = (
        "robot/cmd/move",
        "robot/cmd/pause",
        "robot/cmd/resume",
        "robot/cmd/stop",
        "robot/debug/fall",
        "robot/debug/wander",
    )
    status_publish_interval: float = 5.0


class UiMqttService:
    """OrangePi ↔ Jetson 로컬 MQTT 통신 서비스.

    WiredServiceProtocol(has_client / try_recv / send)과 동일한 인터페이스로
    WiredControlService와 병행 운영 가능.

    Topics:
        SUB: cfg.subscribe_topics      <- OrangePi 명령 수신
        PUB: cfg.topic_*               -> OrangePi 상태/이벤트 발행

    send() 자동 라우팅:
        robot_status                  -> robot/status
        location_update               -> robot/location
        move_status                   -> robot/move/status
        detection_event               -> robot/detection
        config_destinations           -> robot/config/destinations
        morning_call_event            -> robot/morning-call/event
        morning_call_door             -> robot/morning-call/door
        config_schedule               -> robot/config/morning-call-schedule
    """

    def __init__(self, cfg: UiMqttConfig) -> None:
        self._cfg = cfg
        self._connected: bool = False
        self._rx: queue.Queue[dict] = queue.Queue()
        self._client: paho.Client | None = None  # type: ignore[name-defined]

        if cfg.enabled and paho is None:
            raise RuntimeError(
                "UiMqttService requires paho-mqtt. Install: pip install paho-mqtt"
            )

    # ------------------------------------------------------------------ #
    # WiredServiceProtocol 인터페이스
    # ------------------------------------------------------------------ #

    @property
    def has_client(self) -> bool:
        """브로커에 현재 연결되어 있으면 True."""
        return self._connected

    def try_recv(self) -> dict | None:
        """수신 큐(robot/cmd/*)에서 메시지 하나를 꺼낸다. 없으면 None."""
        try:
            return self._rx.get_nowait()
        except queue.Empty:
            return None

    def send(self, msg: dict) -> None:
        """msg["type"]을 기반으로 토픽을 자동 선택하여 발행.

        라우팅 규칙은 _route()에 중앙화한다.
        """
        topic = self._route(msg)
        self._publish(topic, msg)

    def send_status(self, msg: dict) -> None:
        """로봇 상태 토픽으로 직접 발행."""
        self._publish(self._cfg.topic_status, msg)

    def send_event(self, msg: dict) -> None:
        """범용 이벤트 토픽으로 직접 발행.

        5/21 데모 스펙에는 범용 event 토픽이 없으므로 감지 이벤트 토픽을 기본으로 쓴다.
        세부 이벤트는 send()의 type 기반 라우팅을 사용하는 것이 우선이다.
        """
        self._publish(self._cfg.topic_detection, msg)

    # ------------------------------------------------------------------ #
    # 생명주기
    # ------------------------------------------------------------------ #

    def start(self) -> None:
        """브로커 연결 + loop_start + robot/cmd/* 구독.

        enabled=True 이면 로컬 mosquitto 브로커가 아직 없을 경우 자동으로 기동한다.
        """
        if not self._cfg.enabled:
            log.info("[UiMqtt] disabled — skip start")
            return

        # 로컬 브로커 자동 기동 (localhost 접속 시에만)
        if self._cfg.host in ("127.0.0.1", "localhost"):
            self._ensure_broker()

        self._client = paho.Client(client_id=self._cfg.client_id, clean_session=True)
        self._client.on_connect = self._on_connect
        self._client.on_disconnect = self._on_disconnect
        self._client.on_message = self._on_message

        try:
            self._client.connect(self._cfg.host, self._cfg.port, keepalive=self._cfg.keepalive)
            self._client.loop_start()
            log.info(
                "[UiMqtt] connecting to %s:%d (client_id=%s)",
                self._cfg.host, self._cfg.port, self._cfg.client_id,
            )
        except Exception:
            log.exception("[UiMqtt] connect failed")

    def tick(self) -> None:
        """주기적 처리 (현재는 loop_start로 백그라운드 처리 — 빈 구현)."""
        pass

    def stop(self) -> None:
        """loop_stop + disconnect."""
        if self._client is None:
            return
        try:
            self._client.loop_stop()
            self._client.disconnect()
        except Exception:
            log.exception("[UiMqtt] stop error")
        finally:
            self._connected = False
            log.info("[UiMqtt] stopped")

    # ------------------------------------------------------------------ #
    # 내부 헬퍼
    # ------------------------------------------------------------------ #

    def _ensure_broker(self) -> None:
        """로컬 mosquitto 브로커가 응답하지 않으면 자동으로 기동한다.

        이미 포트가 열려 있으면(기존 프로세스) 기동 시도 없이 반환.
        mosquitto 바이너리가 없으면 경고만 출력하고 계속 진행한다.
        """
        import socket as _socket
        # 포트 열림 여부 확인 (이미 실행 중이면 스킵)
        try:
            with _socket.create_connection((self._cfg.host, self._cfg.port), timeout=0.5):
                log.debug("[UiMqtt] broker already listening on port %d", self._cfg.port)
                return
        except OSError:
            pass  # 연결 실패 → 브로커 기동 필요

        log.info("[UiMqtt] starting mosquitto on port %d …", self._cfg.port)
        try:
            subprocess.Popen(
                ["mosquitto", "-p", str(self._cfg.port), "-d"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            # 브로커가 포트를 열 때까지 최대 2초 대기
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                try:
                    with _socket.create_connection((self._cfg.host, self._cfg.port), timeout=0.3):
                        log.info("[UiMqtt] broker ready on port %d", self._cfg.port)
                        return
                except OSError:
                    time.sleep(0.2)
            log.warning("[UiMqtt] broker did not become ready within 2s")
        except FileNotFoundError:
            log.warning(
                "[UiMqtt] mosquitto not found — install with: apt install mosquitto"
                " or start manually: mosquitto -p %d -d", self._cfg.port,
            )
        except Exception as exc:
            log.warning("[UiMqtt] broker auto-start failed: %s", exc)

    def _route(self, msg: dict) -> str:
        """msg["type"]을 보고 발행할 토픽 반환."""
        msg_type: str = str(msg.get("type", ""))
        cfg = self._cfg
        route_map = {
            "robot_status": cfg.topic_status,
            "heartbeat": cfg.topic_status,
            "location_update": cfg.topic_location,
            "move_status": cfg.topic_move_status,
            "patrol_event": cfg.topic_patrol,
            "detection_event": cfg.topic_detection,
            "config_destinations": cfg.topic_config_destinations,
            "morning_call_event": cfg.topic_morning_call_event,
            "morning_call_door": cfg.topic_morning_call_door,
            "config_schedule": cfg.topic_config_schedule,
        }
        return route_map.get(msg_type, cfg.topic_status)

    def _publish(self, topic: str, msg: dict) -> None:
        if not self._connected or self._client is None:
            log.warning("[UiMqtt] not connected — drop topic=%s type=%s", topic, msg.get("type"))
            return
        try:
            # MQTT: topic이 메시지 유형을 식별하므로 type wrapper 제거, payload만 발행
            payload_data = msg.get("payload", msg)
            payload = json.dumps(payload_data, ensure_ascii=False)
            self._client.publish(topic, payload=payload, qos=0)
            log.info("[UiMqtt] pub topic=%s type=%s", topic, msg.get("type"))
        except Exception:
            log.exception("[UiMqtt] publish error topic=%s", topic)

    # ------------------------------------------------------------------ #
    # paho 콜백
    # ------------------------------------------------------------------ #

    def _on_connect(self, client, userdata, flags, rc) -> None:
        if rc == 0:
            self._connected = True
            topics = list(self._cfg.subscribe_topics)
            for topic in topics:
                client.subscribe(topic, qos=0)
            log.info("[UiMqtt] connected — subscribed to %s", topics)
        else:
            self._connected = False
            log.warning("[UiMqtt] connect failed rc=%d", rc)

    def _on_disconnect(self, client, userdata, rc) -> None:
        self._connected = False
        if rc != 0:
            log.warning("[UiMqtt] unexpected disconnect rc=%d", rc)
        else:
            log.info("[UiMqtt] disconnected")

    def _on_message(self, client, userdata, msg) -> None:
        try:
            data = json.loads(msg.payload.decode("utf-8").strip())
            if not isinstance(data, dict):
                log.warning("[UiMqtt] non-dict message on %s — ignored", msg.topic)
                return

            topic_type_map = {
                "robot/cmd/move": "cmd_move",
                "robot/cmd/pause": "cmd_pause",
                "robot/cmd/resume": "cmd_resume",
                "robot/cmd/stop": "cmd_stop",
                "robot/cmd/release": "cmd_release",
                "robot/debug/fall": "debug_fall_detected",
                "robot/debug/wander": "debug_wander_detected",
            }
            topic_norm = str(msg.topic).lstrip("/")
            mapped_type = topic_type_map.get(topic_norm, msg.topic)
            if "type" not in data:
                data = {"type": mapped_type, "payload": data}
            elif "payload" not in data and (
                mapped_type.startswith("cmd_") or mapped_type.startswith("debug_")
            ):
                payload = {k: v for k, v in data.items() if k != "type"}
                data = {"type": data["type"], "payload": payload}

            self._rx.put_nowait(data)
            log.info("[UiMqtt] recv topic=%s type=%s", msg.topic, data.get("type"))
        except Exception:
            log.exception("[UiMqtt] message parse error topic=%s", msg.topic)
