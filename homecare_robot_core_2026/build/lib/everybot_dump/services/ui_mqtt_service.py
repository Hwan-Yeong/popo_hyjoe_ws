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

    # 레거시 토픽 설정 (YAML에서 변경 가능)
    topic_cmd: str = "ui/setup_cmd"                      # SUB: OrangePi → Jetson (legacy)
    topic_event: str = "ui/human_interactive_event"      # PUB: Jetson → OrangePi (legacy event)
    topic_status: str = "ui/status"                      # PUB: Jetson → OrangePi (legacy status)

    # UI → Robot 구독 토픽 (v2.2)
    subscribe_topics: tuple = (
        "/ui/event",
        "/ui/call",
        "/ui/setting/change",
        "/ui/picture_capture",
    )
    legacy_subscribe_topics: tuple = ("ui/setup_cmd",)

    # Robot → UI 발행 토픽 (v2.2)
    topic_robot_status: str = "/robot/status"
    topic_robot_change_ui: str = "/robot/change_ui"
    topic_robot_heartbeat: str = "/robot/heartbeat"
    topic_robot_setting_now: str = "/robot/setting/now"
    topic_robot_event: str = "/robot/event"

    # send() 자동 라우팅 규칙 (YAML 리스트로 설정)
    # type이 이 목록에 있으면 → topic_status 발행
    status_types: tuple = ("robot_status", "request_show_menu", "REG_STATUS")
    # type이 이 prefix로 시작하면 → topic_event 발행
    event_type_prefixes: tuple = ("notify_", "response_")


class UiMqttService:
    """OrangePi ↔ Jetson 로컬 MQTT 통신 서비스.

    WiredServiceProtocol(has_client / try_recv / send)과 동일한 인터페이스로
    WiredControlService와 병행 운영 가능.

    Topics:
        SUB: cfg.subscribe_topics      ← OrangePi 명령 수신
        PUB: cfg.topic_robot_*         → OrangePi 상태/이벤트 발행

    send() 자동 라우팅:
        robot_status                  → /robot/status
        heartbeat                     → /robot/heartbeat
        request_show_menu             → /robot/change_ui
        notify_settings_current       → /robot/setting/now
        REG_STATUS/notify_*/response_*→ /robot/event
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
        """수신 큐(/ui/* 또는 legacy ui/setup_cmd)에서 메시지 하나를 꺼낸다. 없으면 None."""
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
        self._publish(self._cfg.topic_robot_status, msg)

    def send_event(self, msg: dict) -> None:
        """로봇 이벤트 토픽으로 직접 발행."""
        self._publish(self._cfg.topic_robot_event, msg)

    # ------------------------------------------------------------------ #
    # 생명주기
    # ------------------------------------------------------------------ #

    def start(self) -> None:
        """브로커 연결 + loop_start + /ui/* 구독.

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

        if msg_type == "robot_status":
            return self._cfg.topic_robot_status

        if msg_type == "request_show_menu":
            return self._cfg.topic_robot_change_ui

        if msg_type == "heartbeat":
            return self._cfg.topic_robot_heartbeat

        if msg_type == "notify_settings_current":
            return self._cfg.topic_robot_setting_now

        if msg_type == "REG_STATUS":
            return self._cfg.topic_robot_event

        for prefix in self._cfg.event_type_prefixes:
            if msg_type.startswith(prefix):
                return self._cfg.topic_robot_event

        return self._cfg.topic_robot_event  # 기본

    def _publish(self, topic: str, msg: dict) -> None:
        if not self._connected or self._client is None:
            log.debug("[UiMqtt] not connected — drop publish topic=%s type=%s", topic, msg.get("type"))
            return
        try:
            payload = json.dumps(msg, ensure_ascii=False)
            self._client.publish(topic, payload=payload, qos=0)
            log.debug("[UiMqtt] pub topic=%s type=%s", topic, msg.get("type"))
        except Exception:
            log.exception("[UiMqtt] publish error topic=%s", topic)

    # ------------------------------------------------------------------ #
    # paho 콜백
    # ------------------------------------------------------------------ #

    def _on_connect(self, client, userdata, flags, rc) -> None:
        if rc == 0:
            self._connected = True
            topics = list(self._cfg.subscribe_topics)
            for topic in self._cfg.legacy_subscribe_topics:
                if topic and topic not in topics:
                    topics.append(topic)
            if self._cfg.topic_cmd and self._cfg.topic_cmd not in topics:
                topics.append(self._cfg.topic_cmd)

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
            self._rx.put_nowait(data)
            log.info("[UiMqtt] recv topic=%s type=%s", msg.topic, data.get("type"))
        except Exception:
            log.exception("[UiMqtt] message parse error topic=%s", msg.topic)
