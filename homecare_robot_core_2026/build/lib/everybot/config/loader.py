from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict

from .schema import (
    RobotConfig, MapConfig, WaypointsConfig,
    RobotSettingsConfig, FeatureSettingsConfig, ProgramScheduleConfig,
    TtsConfig, MapCreationConfig,
    AmrConfig, HwApiConfig, BtConfig, BgmConfig, StateConfig, NetworkConfig,
    HaConfig,
)
from ..interfaces.mqtt_client import MqttConfig
from ..interfaces.wired_control_server import WiredServerConfig
from ..interfaces.mobile_provision_server import MobileServerConfig
from ..drivers.softap_manager import SoftApConfig
from ..drivers.wifi_manager import HomeWifiConfig
from ..services.ui_mqtt_service import UiMqttConfig


def _load_yaml(path: Path) -> Dict[str, Any]:
    try:
        import yaml  # type: ignore
    except ModuleNotFoundError as e:
        raise RuntimeError("YAML config requires PyYAML. Install: pip install pyyaml") from e

    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ValueError("YAML root must be a mapping/object.")
    return data


def _load_json(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        data = json.load(f)
    if not isinstance(data, dict):
        raise ValueError("JSON root must be an object.")
    return data


def load_config_dict(path: Path) -> Dict[str, Any]:
    if not path.exists():
        raise FileNotFoundError(f"Config not found: {path}")
    suf = path.suffix.lower()
    if suf in (".yaml", ".yml"):
        return _load_yaml(path)
    if suf == ".json":
        return _load_json(path)
    raise ValueError(f"Unsupported config extension: {suf}")


def load_robot_config(path: Path) -> RobotConfig:
    raw = load_config_dict(path)

    wired = raw.get("wired_server", raw.get("wired", {})) or {}
    mobile = raw.get("mobile_server", raw.get("mobile", {})) or {}
    softap = raw.get("softap", {}) or {}
    home = raw.get("home_wifi", {}) or {}
    mqtt = raw.get("mqtt", {}) or {}
    ui_mqtt = raw.get("ui_mqtt", {}) or {}
    network = raw.get("network", {}) or {}
    amr_raw = raw.get("amr", {}) or {}
    hw_api_raw = raw.get("hw_api", {}) or {}
    bt_raw = raw.get("bt", {}) or {}
    bgm_raw = raw.get("bgm", {}) or {}
    state_raw = raw.get("state", {}) or {}
    map_raw = raw.get("map", {}) or {}
    waypoints_raw = raw.get("waypoints", {}) or {}
    robot_settings_raw = raw.get("robot_settings", {}) or {}
    feature_settings_raw = raw.get("feature_settings", {}) or {}
    program_schedule_raw = raw.get("program_schedule", {}) or {}
    tts_raw = raw.get("tts", {}) or {}
    map_creation_raw = raw.get("map_creation", {}) or {}
    ha_raw = raw.get("ha", {}) or {}
    wifi_ifname = str(network.get("wifi_ifname", home.get("wifi_ifname", softap.get("wifi_ifname", "wlP1p1s0"))))

    return RobotConfig(
        name=str(raw.get("name", "Everybot-robot")),
        tick_hz=float(raw.get("tick_hz", 20.0)),

        wired_server=WiredServerConfig(
            bind_host=str(wired.get("bind_host", "0.0.0.0")),
            port=int(wired.get("port", 10001)),
            accept_timeout_sec=float(wired.get("accept_timeout_sec", 1.0)),
            io_timeout_sec=float(wired.get("io_timeout_sec", 1.0)),
        ),
        mobile_server=MobileServerConfig(
            bind_host=str(mobile.get("bind_host", "0.0.0.0")),
            port=int(mobile.get("port", 9990)),
            accept_timeout_sec=float(mobile.get("accept_timeout_sec", 1.0)),
            io_timeout_sec=float(mobile.get("io_timeout_sec", 1.0)),
        ),
        softap=SoftApConfig(
            wifi_ifname=str(softap.get("wifi_ifname", wifi_ifname)),
            ssid=str(softap.get("ssid", "Everybot-robot-ssid")),
            psk=str(softap.get("psk", "12345678")),
            ip_cidr=str(softap.get("ip_cidr", "192.168.0.1/24")),
            ap_con_name=str(softap.get("ap_con_name", "Everybot-robot-softap")),
            home_con_name=str(softap.get("home_con_name", "Everybot-robot-home")),
        ),
        home_wifi=HomeWifiConfig(
            wifi_ifname=str(home.get("wifi_ifname", wifi_ifname)),
            con_name=str(home.get("con_name", "robot-home")),
            autoconnect=bool(home.get("autoconnect", True)),
        ),
        mqtt=MqttConfig(
            enabled=bool(mqtt.get("enabled", False)),
            host=str(mqtt.get("host", "192.168.20.83")),
            port=int(mqtt.get("port", 1883)),
            client_id=str(mqtt.get("client_id", raw.get("name", "Everybot-robot"))),
            username=mqtt.get("username", None),
            password=mqtt.get("password", None),
            keepalive=int(mqtt.get("keepalive", 30)),
            topic_prefix=str(mqtt.get("topic_prefix", f"robots/{raw.get('name','Everybot-robot')}")),
        ),
        ui_mqtt=UiMqttConfig(
            enabled=bool(ui_mqtt.get("enabled", False)),
            host=str(ui_mqtt.get("host", "127.0.0.1")),
            port=int(ui_mqtt.get("port", 1883)),
            client_id=str(ui_mqtt.get("client_id", "Everybot-ui-client")),
            keepalive=int(ui_mqtt.get("keepalive", 30)),
            topic_status=str(ui_mqtt.get("topic_status", "robot/status")),
            topic_location=str(ui_mqtt.get("topic_location", "robot/location")),
            topic_move_status=str(ui_mqtt.get("topic_move_status", "robot/move/status")),
            topic_patrol=str(ui_mqtt.get("topic_patrol", "robot/patrol")),
            topic_detection=str(ui_mqtt.get("topic_detection", "robot/detection")),
            topic_config_destinations=str(ui_mqtt.get("topic_config_destinations", "robot/config/destinations")),
            topic_morning_call_event=str(ui_mqtt.get("topic_morning_call_event", "robot/morning-call/event")),
            topic_morning_call_door=str(ui_mqtt.get("topic_morning_call_door", "robot/morning-call/door")),
            topic_config_schedule=str(ui_mqtt.get("topic_config_schedule", "robot/config/morning-call-schedule")),
            subscribe_topics=tuple(ui_mqtt.get(
                "subscribe_topics",
                [
                    "robot/cmd/move",
                    "robot/cmd/pause",
                    "robot/cmd/resume",
                    "robot/cmd/stop",
                    "robot/cmd/release",
                    "robot/debug/fall",
                    "robot/debug/wander",
                ],
            )),
            status_publish_interval=float(ui_mqtt.get("status_publish_interval", 5.0)),
        ),
        map=MapConfig(
            pgm_path=str(map_raw.get("pgm_path", "configs/map/map.pgm")),
            yaml_path=str(map_raw.get("yaml_path", "configs/map/map.yaml")),
            forbidden_zones_path=str(map_raw.get("forbidden_zones_path", "configs/map/forbidden_zones.json")),
            roi_zones_path=str(map_raw.get("roi_zones_path", "configs/map/roi_zones.json")),
        ),
        waypoints=WaypointsConfig(
            file_path=str(waypoints_raw.get("file_path", "configs/waypoints.json")),
        ),
        robot_settings=RobotSettingsConfig(
            file_path=str(robot_settings_raw.get("file_path", "configs/robot_settings.json")),
        ),
        feature_settings=FeatureSettingsConfig(
            file_path=str(feature_settings_raw.get("file_path", "configs/feature_settings.json")),
        ),
        program_schedule=ProgramScheduleConfig(
            file_path=str(program_schedule_raw.get("file_path", "configs/program_schedule.json")),
        ),
        tts=TtsConfig(
            mode=str(tts_raw.get("mode", "file")),
            file_root=str(tts_raw.get("file_root", "assets/tts/")),
        ),
        map_creation=MapCreationConfig(
            server_port=int(map_creation_raw.get("server_port", 8080)),
        ),
        amr=AmrConfig(
            ip=str(amr_raw.get("ip", "192.168.60.206")),
            port=int(amr_raw.get("port", 10000)),
            recv_buf_size=int(amr_raw.get("recv_buf_size", 262144)),
        ),
        hw_api=HwApiConfig(
            speaker_url=str(hw_api_raw.get("speaker_url", "http://localhost:8083")),
            rf_url=str(hw_api_raw.get("rf_url", "http://localhost:8084")),
            camera_url=str(hw_api_raw.get("camera_url", "http://localhost:8081")),
            mic_url=str(hw_api_raw.get("mic_url", "http://localhost:8082")),
            tts_url=str(hw_api_raw.get("tts_url", "http://127.0.0.1:8085")),
            fall_status_url=str(hw_api_raw.get("fall_status_url", "http://192.168.31.167:8008/api/fall-status")),
            fall_status_timeout=float(hw_api_raw.get("fall_status_timeout", 0.25)),
            agent_events_url=str(hw_api_raw.get("agent_events_url", "http://127.0.0.1:8086")),
            agent_events_timeout=float(hw_api_raw.get("agent_events_timeout", 2.0)),
            conversation_wait_timeout=float(hw_api_raw.get("conversation_wait_timeout", 45.0)),
            face_api_url=str(hw_api_raw.get("face_api_url", "http://127.0.0.1:8087")),
            face_api_timeout=float(hw_api_raw.get("face_api_timeout", 1.0)),
        ),
        bt=BtConfig(
            debug_mode=str(bt_raw.get("debug_mode", "snapshot")),
            tick_log_interval=int(bt_raw.get("tick_log_interval", 100)),
        ),
        bgm=BgmConfig(
            file_root=str(bgm_raw.get("file_root", "/opt/everybot/bgm/")),
        ),
        state=StateConfig(
            path=str(state_raw.get("path", "/var/lib/everybot/state.json")),
        ),
        network=NetworkConfig(
            wifi_ifname=wifi_ifname,
            auto_softap_on_disconnect=bool(network.get("auto_softap_on_disconnect", False)),
            disconnect_grace_sec=float(network.get("disconnect_grace_sec", 30.0)),
        ),
        ha=HaConfig(
            enabled=bool(ha_raw.get("enabled", False)),
        ),
    )
