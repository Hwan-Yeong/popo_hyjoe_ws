from __future__ import annotations

import json
import logging
import threading
import time
from dataclasses import dataclass
from queue import Queue, Empty

from ..interfaces.jsonl import loads_line, dumps_line
from ..interfaces.mobile_provision_server import MobileProvisionServer, MobileServerConfig

log = logging.getLogger(__name__)


@dataclass
class MobileProvisionService:
    cfg: MobileServerConfig

    def __post_init__(self) -> None:
        self._srv = MobileProvisionServer(self.cfg)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._rx: Queue[dict] = Queue()
        self._tx: Queue[dict] = Queue()
        self._running = False

    @property
    def running(self) -> bool:
        return self._running

    @property
    def has_client(self) -> bool:
        return self._srv.has_client

    def send(self, msg: dict) -> None:
        self._tx.put(msg)

    def try_recv(self) -> dict | None:
        try:
            return self._rx.get_nowait()
        except Empty:
            return None

    def start(self) -> None:
        if self._running:
            return
        self._srv.open()
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="mobile-provision", daemon=True)
        self._thread.start()
        self._running = True
        log.info("[mobile] server listening :%d", self.cfg.port)

    def tick(self) -> None:
        pass

    def stop(self) -> None:
        if not self._running:
            return
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        self._srv.close()
        self._running = False
        log.info("[mobile] stopped")

    def _run(self) -> None:
        while not self._stop.is_set():
            if not self._srv.has_client:
                accepted = self._srv.accept_once()
                if accepted:
                    self._send_obj({"type": "hello", "payload": {"msg": "connected"}})

            # TX
            if self._srv.has_client:
                try:
                    while True:
                        msg = self._tx.get_nowait()
                        self._srv.send_raw(dumps_line(msg))
                except Empty:
                    pass
                except Exception as e:
                    log.warning("[mobile] send failed: %s", e)

            # RX
            line = self._srv.recv_line()
            if line is not None:
                try:
                    msg = loads_line(line)
                    self._rx.put(msg)
                except Exception as e:
                    log.warning("[mobile] bad json: %s", e)

            time.sleep(0.01)

    def _send_obj(self, obj: dict) -> None:
        data = (json.dumps(obj, ensure_ascii=False) + "\n").encode("utf-8")
        try:
            self._srv.send_raw(data)
            log.info("[mobile] send: %s", obj.get("type"))
        except Exception as e:
            log.info("[mobile] send failed(drop client): %s", e)