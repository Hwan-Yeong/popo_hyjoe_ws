from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..bt.blackboard import RobotBlackboard
    from ..drivers.softap_manager import SoftApManager
    from ..drivers.wifi_manager import WifiManager

log = logging.getLogger(__name__)


@dataclass
class NetworkMonitorService:
    wifi_ifname: str = "wlP1p1s0"
    auto_softap: bool = False
    disconnect_grace: float = 30.0

    def __post_init__(self) -> None:
        self._wifi_ready = False
        self._wifi_connected = False
        self._wifi_ssid: str | None = None
        self._last_check = 0.0
        self._disconnect_at = 0.0
        self._wifi: WifiManager | None = None
        self._softap: SoftApManager | None = None
        self._bb: RobotBlackboard | None = None

    @property
    def wifi_ready(self) -> bool:
        return self._wifi_ready

    @property
    def wifi_connected(self) -> bool:
        return self._wifi_connected

    @property
    def wifi_ssid(self) -> str | None:
        return self._wifi_ssid

    @property
    def softap_active(self) -> bool:
        return self._softap.enabled if self._softap else False

    @property
    def netstat(self) -> int:
        return 0 if self.softap_active else 1

    def set_wifi(self, wifi: "WifiManager") -> None:
        self._wifi = wifi

    def set_softap(self, softap: "SoftApManager") -> None:
        self._softap = softap

    def set_bb(self, bb: "RobotBlackboard") -> None:
        self._bb = bb

    def start(self) -> None:
        log.info(
            "[net] monitor started (wifi_if=%s, auto_softap=%s)",
            self.wifi_ifname,
            self.auto_softap,
        )

    def tick(self) -> None:
        now = time.monotonic()
        if now - self._last_check < 0.5:
            return
        self._last_check = now

        self._wifi_ready = self._is_iface_up_with_ip(self.wifi_ifname)
        if self._wifi is not None:
            self._wifi_connected = self._wifi.is_connected()
            self._wifi_ssid = self._wifi.current_ssid()
        else:
            self._wifi_connected = self._wifi_ready
            self._wifi_ssid = None

        if (
            self.auto_softap
            and self._bb is not None
            and self._bb.wifi_registered
            and not self._wifi_connected
        ):
            if self._disconnect_at == 0.0:
                self._disconnect_at = now
                log.warning("[net] WiFi disconnected - SoftAP fallback in %.0fs", self.disconnect_grace)
            elif now - self._disconnect_at >= self.disconnect_grace:
                log.warning("[net] WiFi disconnected for %.0fs - enabling SoftAP", self.disconnect_grace)
                self._trigger_softap()
                self._disconnect_at = 0.0
        else:
            self._disconnect_at = 0.0

    def stop(self) -> None:
        log.info("[net] monitor stopped")

    def _trigger_softap(self) -> None:
        if self._softap is None or self._softap.enabled:
            return
        try:
            self._softap.enable()
            log.info("[net] SoftAP fallback enabled")
        except Exception as exc:
            log.error("[net] SoftAP fallback failed: %s", exc)

    def _is_iface_up_with_ip(self, ifname: str) -> bool:
        oper = Path(f"/sys/class/net/{ifname}/operstate")
        if not oper.exists():
            return False
        try:
            state = oper.read_text(encoding="utf-8").strip()
        except Exception:
            return False
        if state != "up":
            return False

        return True
