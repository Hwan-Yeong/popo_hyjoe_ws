from __future__ import annotations

import socket
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass(frozen=True)
class WiredServerConfig:
    bind_host: str = "0.0.0.0"
    port: int = 10001
    accept_timeout_sec: float = 1.0
    io_timeout_sec: float = 1.0


class WiredControlServer:
    def __init__(self, cfg: WiredServerConfig):
        self._cfg = cfg
        self._srv: socket.socket | None = None
        self._conn: socket.socket | None = None
        self._conn_addr: Optional[Tuple[str, int]] = None
        self._rx_buf = bytearray()

    def open(self) -> None:
        self.close()
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((self._cfg.bind_host, self._cfg.port))
        srv.listen(1)
        srv.settimeout(self._cfg.accept_timeout_sec)
        self._srv = srv

    def close(self) -> None:
        if self._conn is not None:
            try:
                self._conn.close()
            finally:
                self._conn = None
                self._conn_addr = None
        if self._srv is not None:
            try:
                self._srv.close()
            finally:
                self._srv = None
        self._rx_buf.clear()

    @property
    def has_client(self) -> bool:
        return self._conn is not None

    @property
    def client_addr(self) -> Optional[Tuple[str, int]]:
        return self._conn_addr

    def accept_once(self) -> bool:
        if not self._srv:
            raise RuntimeError("server not open")
        try:
            conn, addr = self._srv.accept()
            conn.settimeout(self._cfg.io_timeout_sec)
            # 기존 연결이 있으면 교체
            if self._conn:
                self._conn.close()
            self._conn = conn
            self._conn_addr = addr
            self._rx_buf.clear()
            return True
        except socket.timeout:
            return False

    def send_raw(self, data: bytes) -> None:
        if not self._conn:
            raise RuntimeError("no client")
        self._conn.sendall(data)

    def recv_line(self) -> bytes | None:
        """
        한 줄(\\n) 기준 수신. 타임아웃이면 None.
        """
        if not self._conn:
            return None
        try:
            chunk = self._conn.recv(4096)
            if not chunk:
                # peer closed
                self._conn.close()
                self._conn = None
                self._conn_addr = None
                return None
            self._rx_buf.extend(chunk)
        except socket.timeout:
            return None
        except OSError:
            self._conn = None
            self._conn_addr = None
            return None

        nl = self._rx_buf.find(b"\n")
        if nl < 0:
            return None
        line = bytes(self._rx_buf[:nl])
        del self._rx_buf[: nl + 1]
        return line
