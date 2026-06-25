from __future__ import annotations

import logging
import os
import socket
import subprocess
import time
from dataclasses import dataclass
from typing import List, Optional

log = logging.getLogger(__name__)


@dataclass(frozen=True)
class SoftApConfig:
    # 실제 AP를 올릴 Wi-Fi 인터페이스 이름 (예: wlP1p1s0)
    wifi_ifname: str

    # AP 프로필 이름(nmcli connection name)
    ap_con_name: str = "Everybot-robot-softap"

    # (선택) 홈 Wi-Fi 프로필 이름(필요 시 AP 끄고 STA로 복귀할 때)
    home_con_name: str = "Everybot-robot-home"

    # SoftAP SSID/비번, IP 대역
    ssid: str = "Everybot-robot-ssid"
    psk: str = "12345678"
    ip_cidr: str = "192.168.0.1/24"

    # 공유/NAT를 쓰려면 shared(간단), 아니면 manual
    ipv4_method: str = "shared"  # "shared" or "manual"

    # nmcli 명령 타임아웃
    cmd_timeout_sec: float = 10.0

    # (검증용) 모바일이 붙어야 하는 TCP 포트(서버는 다른 모듈이 열어야 함)
    verify_listen_port: int = 9990

    # (검증용) AP 올라온 뒤 IP/포트 확인 리트라이
    verify_retries: int = 10
    verify_sleep_sec: float = 0.3


class SoftApManager:
    """
    NetworkManager(nmcli)를 이용해 SoftAP 프로필을 올리고/내린다.

    추가된 검증:
    - AP 올린 후: wifi_ifname에 192.168.0.1(=ip_cidr의 host)이 실제로 설정됐는지 확인
    - (선택) 9990 포트가 0.0.0.0 또는 192.168.0.1로 LISTEN 중인지 확인(없으면 경고)
      => 이 포트는 softap_manager가 열 수 없음. 다른 서비스가 열어야 함.
    """

    def __init__(self, cfg: SoftApConfig):
        self._cfg = cfg
        self._enabled = False

    @property
    def enabled(self) -> bool:
        return self._enabled

    def enable(self) -> None:
        try:
            self._ensure_nmcli_available()
            self._ensure_connection_exists_and_is_ap()
            self._ensure_iface_binding()

            # 이미 떠있던 프로필이면 내려야 수정값이 확실히 반영되는 경우가 있음
            self._run(["nmcli", "con", "down", self._cfg.ap_con_name], allow_fail=True)

            # SSID/PSK/IP를 config 값으로 강제(프로필 오염 방지)
            self._run(["nmcli", "con", "modify", self._cfg.ap_con_name, "802-11-wireless.ssid", self._cfg.ssid])
            self._run(["nmcli", "con", "modify", self._cfg.ap_con_name, "wifi-sec.key-mgmt", "wpa-psk"])
            self._run(["nmcli", "con", "modify", self._cfg.ap_con_name, "wifi-sec.psk", self._cfg.psk])
            self._run(["nmcli", "con", "modify", self._cfg.ap_con_name, "ipv4.method", self._cfg.ipv4_method])
            self._run(["nmcli", "con", "modify", self._cfg.ap_con_name, "ipv4.addresses", self._cfg.ip_cidr])
            self._run(["nmcli", "con", "modify", self._cfg.ap_con_name, "connection.autoconnect", "no"])

            # AP 올리기
            self._run(["nmcli", "con", "up", self._cfg.ap_con_name])
            self._enabled = True

            # ✅ 검증: IP가 실제로 올라왔는지
            self._verify_ap_ip()

            # ✅ 검증: 서버 포트가 열려 있는지(경고용)
            self._verify_listen_port_warn_only()

            log.info("[softap] enabled: con=%s if=%s ssid=%s ip=%s (psk_len=%d)",
                     self._cfg.ap_con_name, self._cfg.wifi_ifname, self._cfg.ssid, self._cfg.ip_cidr, len(self._cfg.psk))
        except Exception:
            self._enabled = False
            raise

    def disable(self) -> None:
        try:
            self._ensure_nmcli_available()
            self._run(["nmcli", "con", "down", self._cfg.ap_con_name], allow_fail=True)
        finally:
            self._enabled = False
            log.info("[softap] disabled")

    def switch_to_home(self) -> None:
        self.disable()
        if self._cfg.home_con_name:
            self._run(["nmcli", "con", "up", self._cfg.home_con_name], allow_fail=True)

    # --------------------------
    # Internal: checks / ensure
    # --------------------------

    def _ensure_nmcli_available(self) -> None:
        self._run(["nmcli", "-v"])

    def _ensure_connection_exists_and_is_ap(self) -> None:
        if not self._connection_exists(self._cfg.ap_con_name):
            self._create_ap_connection()

        con_type = self._run_capture(["nmcli", "-g", "connection.type", "con", "show", self._cfg.ap_con_name]).strip()
        if con_type and con_type != "802-11-wireless":
            raise RuntimeError(
                f"Connection '{self._cfg.ap_con_name}' type is '{con_type}', expected '802-11-wireless'"
            )

        mode = self._run_capture(
            ["nmcli", "-g", "802-11-wireless.mode", "con", "show", self._cfg.ap_con_name]
        ).strip()
        if mode != "ap":
            self._run(["nmcli", "con", "modify", self._cfg.ap_con_name, "802-11-wireless.mode", "ap"])

    def _ensure_iface_binding(self) -> None:
        want = (self._cfg.wifi_ifname or "").strip()
        if not want:
            raise RuntimeError("SoftAP wifi_ifname is empty. Set it to actual Wi-Fi interface (e.g., wlP1p1s0).")

        # 장치 존재/타입 확인(없으면 명확하게 실패시키는 게 디버깅에 유리)
        dev_type = self._run_capture(["nmcli", "-g", "GENERAL.TYPE", "dev", "show", want], allow_fail=True).strip()
        if dev_type != "wifi":
            # dev_type이 비어있으면 장치 자체가 없을 가능성
            raise RuntimeError(f"SoftAP wifi_ifname '{want}' is not a wifi device (nmcli type='{dev_type}').")

        cur = self._run_capture(
            ["nmcli", "-g", "connection.interface-name", "con", "show", self._cfg.ap_con_name]
        ).strip()

        if cur != want:
            log.info("[softap] fixing interface binding: %s -> %s (con=%s)",
                     cur if cur else "(unset)", want, self._cfg.ap_con_name)
            self._run(["nmcli", "con", "modify", self._cfg.ap_con_name, "connection.interface-name", want])

    def _verify_ap_ip(self) -> None:
        # ip_cidr에서 host ip만 추출("192.168.0.1/24" -> "192.168.0.1")
        ap_ip = self._cfg.ip_cidr.split("/", 1)[0].strip()
        if not ap_ip:
            return

        # nmcli con up 직후 바로 안 잡히는 경우가 있어 리트라이
        for _ in range(max(1, self._cfg.verify_retries)):
            out = self._run_capture(["ip", "-4", "addr", "show", "dev", self._cfg.wifi_ifname], allow_fail=True)
            if out and ap_ip in out:
                return
            time.sleep(self._cfg.verify_sleep_sec)

        raise RuntimeError(f"[softap] AP IP '{ap_ip}' not found on interface '{self._cfg.wifi_ifname}' after enable.")

    def _verify_listen_port_warn_only(self) -> None:
        """
        9990 포트는 softap_manager가 열지 않는다.
        다만 모바일 접속 문제를 빨리 잡기 위해:
        - 0.0.0.0:9990 또는 192.168.0.1:9990 LISTEN 여부를 확인하고
        - 아니면 경고 로그를 남긴다.
        """
        port = int(self._cfg.verify_listen_port)
        ap_ip = self._cfg.ip_cidr.split("/", 1)[0].strip()

        # 가장 확실: ss로 확인 (권한 필요할 수 있음)
        out = self._run_capture(["ss", "-lnt"], allow_fail=True)
        if out:
            # 0.0.0.0:9990 또는 192.168.0.1:9990 또는 [::]:9990
            if f":{port} " in out and ("0.0.0.0" in out or ap_ip in out or "[::]" in out or "::" in out):
                return

        # 보조: localhost에서 connect 시도(서버가 127.0.0.1만 열었을 수도 있으니 참고용)
        # 여기선 경고만
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(0.2)
            s.connect((ap_ip, port))
            s.close()
            return
        except Exception:
            pass

        log.warning(
            "[softap] AP is up but port %d does not appear LISTEN on %s/0.0.0.0. "
            "Check your TCP server bind address (must be 0.0.0.0 or %s, not 127.0.0.1).",
            port, ap_ip, ap_ip
        )

    # --------------------------
    # Connection creation / query
    # --------------------------

    def _connection_exists(self, name: str) -> bool:
        out = self._run_capture(["nmcli", "-t", "-f", "NAME", "con", "show"], allow_fail=True)
        if not out:
            return False
        names = {line.strip() for line in out.splitlines() if line.strip()}
        return name in names

    def _create_ap_connection(self) -> None:
        want = (self._cfg.wifi_ifname or "").strip()
        if not want:
            raise RuntimeError("SoftAP wifi_ifname is empty. Set it to actual Wi-Fi interface.")

        self._run([
            "nmcli", "con", "add",
            "type", "wifi",
            "ifname", want,
            "con-name", self._cfg.ap_con_name,
            "autoconnect", "no",
            "ssid", self._cfg.ssid,
        ])

        self._run(["nmcli", "con", "modify", self._cfg.ap_con_name, "802-11-wireless.mode", "ap"])
        self._run(["nmcli", "con", "modify", self._cfg.ap_con_name, "wifi-sec.key-mgmt", "wpa-psk"])
        self._run(["nmcli", "con", "modify", self._cfg.ap_con_name, "wifi-sec.psk", self._cfg.psk])
        self._run(["nmcli", "con", "modify", self._cfg.ap_con_name, "ipv4.method", self._cfg.ipv4_method])
        self._run(["nmcli", "con", "modify", self._cfg.ap_con_name, "ipv4.addresses", self._cfg.ip_cidr])
        self._run(["nmcli", "con", "modify", self._cfg.ap_con_name, "connection.interface-name", want])

        log.info("[softap] created connection: con=%s if=%s ssid=%s ip=%s",
                 self._cfg.ap_con_name, want, self._cfg.ssid, self._cfg.ip_cidr)

    # --------------------------
    # Command runners
    # --------------------------

    def _run(self, args: List[str], allow_fail: bool = False) -> None:
        try:
            p = subprocess.run(
                args,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=self._cfg.cmd_timeout_sec,
                check=False,
            )
        except subprocess.TimeoutExpired as e:
            raise RuntimeError(f"Command timeout: {' '.join(args)}") from e

        if p.returncode != 0 and not allow_fail:
            raise RuntimeError(
                f"Command failed: {' '.join(args)}\n"
                f"stdout: {p.stdout}\n"
                f"stderr: {p.stderr}"
            )

    def _run_capture(self, args: List[str], allow_fail: bool = False) -> str:
        try:
            p = subprocess.run(
                args,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=self._cfg.cmd_timeout_sec,
                check=False,
            )
        except subprocess.TimeoutExpired as e:
            raise RuntimeError(f"Command timeout: {' '.join(args)}") from e

        if p.returncode != 0 and not allow_fail:
            raise RuntimeError(
                f"Command failed: {' '.join(args)}\n"
                f"stdout: {p.stdout}\n"
                f"stderr: {p.stderr}"
            )
        return (p.stdout or "").strip()