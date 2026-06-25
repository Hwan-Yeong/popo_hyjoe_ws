from __future__ import annotations

from dataclasses import dataclass, field

from ..interfaces.mqtt_client import MqttConfig
from ..interfaces.wired_control_server import WiredServerConfig
from ..interfaces.mobile_provision_server import MobileServerConfig
from ..drivers.softap_manager import SoftApConfig
from ..drivers.wifi_manager import HomeWifiConfig
from ..services.ui_mqtt_service import UiMqttConfig


# ── 신규 Config 섹션 ─────────────────────────────────────────────

@dataclass(frozen=True)
class MapConfig:
    """맵 파일 경로 설정."""
    pgm_path:             str = "configs/map/map.pgm"
    yaml_path:            str = "configs/map/map.yaml"
    forbidden_zones_path: str = "configs/map/forbidden_zones.json"
    roi_zones_path:       str = "configs/map/roi_zones.json"


@dataclass(frozen=True)
class WaypointsConfig:
    """Waypoint 파일 경로 설정."""
    file_path: str = "configs/waypoints.json"


@dataclass(frozen=True)
class RobotSettingsConfig:
    """로봇 운영 설정 파일 경로."""
    file_path: str = "configs/robot_settings.json"


@dataclass(frozen=True)
class TtsConfig:
    """TTS 모드 설정."""
    mode:      str = "file"        # "ai" | "file"
    file_root: str = "assets/tts/"


@dataclass(frozen=True)
class MapCreationConfig:
    """맵 생성 웹서버 설정."""
    server_port: int = 8080


@dataclass(frozen=True)
class AmrConfig:
    """AMR UDP 연결 파라미터."""
    ip:            str = "192.168.60.206"
    port:          int = 10000
    recv_buf_size: int = 262144


@dataclass(frozen=True)
class HwApiConfig:
    """HW-Components HTTP API base URL."""
    speaker_url: str = "http://localhost:8083"
    rf_url:      str = "http://localhost:8084"
    camera_url:  str = "http://localhost:8081"
    mic_url:     str = "http://localhost:8082"


@dataclass(frozen=True)
class BtConfig:
    """BT 디버그 설정."""
    debug_mode:        str = "snapshot"
    tick_log_interval: int = 100


@dataclass(frozen=True)
class BgmConfig:
    """BGM 파일 루트 경로."""
    file_root: str = "/opt/everybot/bgm/"


@dataclass(frozen=True)
class StateConfig:
    """상태 파일 저장 경로."""
    path: str = "/var/lib/everybot/state.json"


@dataclass(frozen=True)
class NetworkConfig:
    """런타임 네트워크 정책 설정."""
    wifi_ifname:               str = "wlP1p1s0"
    auto_softap_on_disconnect: bool = False
    disconnect_grace_sec:      float = 30.0


@dataclass(frozen=True)
class RobotConfig:
    name: str = "Everybot-robot"
    tick_hz: float = 20.0

    # (1) Orangepi <-> Jetson 유선 TCP 서버: 192.168.10.1:10001
    # IP(192.168.10.1)은 OS 네트워크 설정 영역이지만, 포트는 여기서 확정.
    wired_server: WiredServerConfig = WiredServerConfig(port=10001)

    # (2) SoftAP에서 모바일 앱이 붙는 서버: 192.168.0.1:9990
    mobile_server: MobileServerConfig = MobileServerConfig(port=9990)

    # SoftAP IP 고정: 192.168.0.1/24
    softap: SoftApConfig = SoftApConfig(
        wifi_ifname="wlP1p1s0",
        ssid="Everybot-robot-ssid",
        psk="12345678",
        ip_cidr="192.168.0.1/24",
        ap_con_name="Everybot-robot-softap",
        home_con_name="Everybot-robot-home",
    )

    # (3) 홈 Wi-Fi는 DHCP (nmcli 기본 동작)
    # (4) 등록 이후에도 유지하려면 connection을 유지 + autoconnect ON
    home_wifi: HomeWifiConfig = HomeWifiConfig(
        wifi_ifname="wlP1p1s0",
        con_name="robot-home",
        autoconnect=True,
    )

    mqtt: MqttConfig = MqttConfig(enabled=False)

    # OrangePi ↔ Jetson 로컬 MQTT (Jetson mosquitto 브로커)
    ui_mqtt: UiMqttConfig = UiMqttConfig(enabled=False)

    # ── v2 신규 설정 섹션 ──────────────────────────────────────
    map:            MapConfig           = field(default_factory=MapConfig)
    waypoints:      WaypointsConfig     = field(default_factory=WaypointsConfig)
    robot_settings: RobotSettingsConfig = field(default_factory=RobotSettingsConfig)
    tts:            TtsConfig           = field(default_factory=TtsConfig)
    map_creation:   MapCreationConfig   = field(default_factory=MapCreationConfig)
    amr:            AmrConfig           = field(default_factory=AmrConfig)
    hw_api:         HwApiConfig         = field(default_factory=HwApiConfig)
    bt:             BtConfig            = field(default_factory=BtConfig)
    bgm:            BgmConfig           = field(default_factory=BgmConfig)
    state:          StateConfig         = field(default_factory=StateConfig)
    network:        NetworkConfig       = field(default_factory=NetworkConfig)

