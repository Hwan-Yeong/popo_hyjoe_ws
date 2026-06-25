from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional

try:
    import paho.mqtt.client as paho
except ModuleNotFoundError:
    paho = None


@dataclass(frozen=True)
class MqttConfig:
    enabled: bool = False
    host: str = "192.168.20.83"
    port: int = 1883
    client_id: str = "Everybot-robot"
    username: str | None = None
    password: str | None = None
    keepalive: int = 30
    topic_prefix: str = "robots/"


class MqttClient:
    def __init__(self, cfg: MqttConfig):
        if paho is None:
            raise RuntimeError("paho-mqtt is not installed. Install: pip install paho-mqtt")
        self._cfg = cfg
        self._client = paho.Client(client_id=cfg.client_id, clean_session=True)
        if cfg.username:
            self._client.username_pw_set(cfg.username, cfg.password)

        self._on_message_cb: Optional[Callable[[str, bytes], None]] = None

        def _on_message(client, userdata, msg):
            if self._on_message_cb:
                self._on_message_cb(msg.topic, msg.payload)

        self._client.on_message = _on_message

    def set_on_message(self, cb: Callable[[str, bytes], None]) -> None:
        self._on_message_cb = cb

    def connect(self) -> None:
        self._client.connect(self._cfg.host, self._cfg.port, keepalive=self._cfg.keepalive)

    def loop_start(self) -> None:
        self._client.loop_start()

    def loop_stop(self) -> None:
        self._client.loop_stop()

    def disconnect(self) -> None:
        self._client.disconnect()

    def subscribe(self, topic: str, qos: int = 0) -> None:
        self._client.subscribe(topic, qos=qos)

    def publish(self, topic: str, payload: str | bytes, qos: int = 0, retain: bool = False) -> None:
        self._client.publish(topic, payload=payload, qos=qos, retain=retain)
