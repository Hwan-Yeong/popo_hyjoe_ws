from __future__ import annotations

import logging
import re
import subprocess
from dataclasses import dataclass

log = logging.getLogger(__name__)


@dataclass(frozen=True)
class HomeWifiConfig:
    wifi_ifname: str = "wlP1p1s0"
    con_name: str = "Everybot-robot-home"
    autoconnect: bool = True


class WifiManager:
    """
    Home Wi-Fi(STA) 연결 (DHCP).
    - connection을 삭제하지 않고 유지: 등록 이후 재부팅 자동 연결을 위해 필수
    """
    def __init__(self, cfg: HomeWifiConfig):
        self._cfg = cfg

    def connect(self, ssid: str, psk: str) -> None:
        self._run(["nmcli", "radio", "wifi", "on"])

        if self._connection_exists(self._cfg.con_name):
            # 기존 connection 재사용: SSID/PSK 갱신 후 up
            self._run(["nmcli", "con", "modify", self._cfg.con_name,
                       "802-11-wireless.ssid", ssid,
                       "wifi-sec.key-mgmt", "wpa-psk",
                       "wifi-sec.psk", psk,
                       "ipv4.method", "auto",
                       "ipv6.method", "ignore",
                       "connection.autoconnect", "yes" if self._cfg.autoconnect else "no"])
            self._run(["nmcli", "con", "up", self._cfg.con_name])
        else:
            # 처음 등록 시: connection 생성 + DHCP(auto)
            self._run([
                "nmcli", "dev", "wifi", "connect", ssid,
                "password", psk,
                "ifname", self._cfg.wifi_ifname,
                "name", self._cfg.con_name
            ])
            # autoconnect 강제 설정
            self._run(["nmcli", "con", "modify", self._cfg.con_name,
                       "connection.autoconnect", "yes" if self._cfg.autoconnect else "no",
                       "ipv4.method", "auto",
                       "ipv6.method", "ignore"])

        log.info("[wifi] connected(dhcp) ssid=%s con=%s", ssid, self._cfg.con_name)

    def reconnect(self) -> None:
        """저장된 home WiFi 프로파일로 재연결 (SSID/PSK 불필요).
        Force SoftAP 해제 후 home WiFi 복귀에 사용.
        프로파일 미존재 시 RuntimeError → 호출부에서 catch 처리.
        """
        self._run(["nmcli", "radio", "wifi", "on"])
        self._run(["nmcli", "con", "up", self._cfg.con_name])
        log.info("[wifi] reconnected via profile con=%s", self._cfg.con_name)

    def disconnect(self) -> None:
        self._run(["nmcli", "con", "down", self._cfg.con_name], check=False)
        log.info("[wifi] disconnected con=%s", self._cfg.con_name)

    def current_ssid(self) -> str | None:
        """현재 연결된 WiFi SSID. 없으면 None."""
        out = self._run_capture(
            ["nmcli", "-t", "-f", "ACTIVE,SSID", "dev", "wifi"],
            allow_fail=True,
        )
        if not out:
            return None
        for line in out.splitlines():
            parts = line.split(":")
            if len(parts) >= 2 and parts[0].strip() == "yes":
                ssid = ":".join(parts[1:]).strip()
                return ssid or None
        return None

    def current_ip(self) -> str | None:
        """현재 WiFi 인터페이스 IPv4 주소. 없으면 None."""
        out = self._run_capture(
            ["ip", "-4", "addr", "show", "dev", self._cfg.wifi_ifname],
            allow_fail=True,
        )
        if not out:
            return None
        match = re.search(r"inet\s+(\d+\.\d+\.\d+\.\d+)", out)
        return match.group(1) if match else None

    def is_connected(self) -> bool:
        """WiFi IP 할당 여부 기준 연결 확인."""
        return self.current_ip() is not None

    def _connection_exists(self, con_name: str) -> bool:
        res = subprocess.run(["nmcli", "-t", "-f", "NAME", "con", "show"],
                             capture_output=True, text=True)
        if res.returncode != 0:
            return False
        names = [line.strip() for line in res.stdout.splitlines() if line.strip()]
        return con_name in names

    def _run(self, cmd: list[str], check: bool = True) -> None:
        log.debug("cmd: %s", " ".join(cmd))
        res = subprocess.run(cmd, capture_output=True, text=True)
        if check and res.returncode != 0:
            raise RuntimeError(
                f"Command failed: {' '.join(cmd)}\n"
                f"stdout: {res.stdout}\n"
                f"stderr: {res.stderr}"
            )

    def _run_capture(self, cmd: list[str], allow_fail: bool = False) -> str:
        log.debug("cmd: %s", " ".join(cmd))
        res = subprocess.run(cmd, capture_output=True, text=True)
        if res.returncode != 0 and not allow_fail:
            raise RuntimeError(
                f"Command failed: {' '.join(cmd)}\n"
                f"stdout: {res.stdout}\n"
                f"stderr: {res.stderr}"
            )
        return (res.stdout or "").strip()
