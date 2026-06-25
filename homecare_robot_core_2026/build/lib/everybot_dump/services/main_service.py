from __future__ import annotations

import logging
import secrets
import string
import time
from dataclasses import dataclass, field
from enum import Enum

from everybot.interfaces.ha_client import HomeAssistantConfig
from everybot.services.amr_service import AmrService

from ..drivers.softap_manager import SoftApManager, SoftApConfig
from ..drivers.wifi_manager import WifiManager, HomeWifiConfig
from ..interfaces.mqtt_client import MqttConfig
from .mqtt_service import MqttService
from .wired_control_service import WiredControlService
from .mobile_provision_service import MobileProvisionService
from .ui_mqtt_service import UiMqttService

from ..utils.beam_projector_util import BeamProjectorUtil

from ..utils.state_store import StateStore

log = logging.getLogger(__name__)

STATE_PATH_DEFAULT = "/var/lib/everybot/state.json"

class RegState(str, Enum):
    IDLE = "IDLE"
    SOFTAP_UP = "SOFTAP_UP"
    HOME_WIFI_CONNECTING = "HOME_WIFI_CONNECTING"
    VERIFIED = "VERIFIED"
    DONE = "DONE"
    REGISTED = "REGISTED"


def _rand_psk(n: int = 12) -> str:
    alphabet = string.ascii_letters + string.digits
    return "".join(secrets.choice(alphabet) for _ in range(n))

@dataclass
class RegistrationContext:
    robot_sn: str | None = None
    robot_model: str | None = None
    softap_ssid: str | None = None
    softap_psk: str | None = None

    home_wifi_ssid: str | None = None
    home_wifi_psk: str | None = None
    smarthome_ip: str | None = None
    smarthome_token: str | None = None
    #smarthome: dict = field(default_factory=dict)
    
    uuid_for_mqtt : str | None = None
    verify_ok: bool | None = None
    verify_reason: str = ""
    registered_robot_id: str | None = None

    reg_state: RegState = RegState.IDLE

    def is_done(self) -> bool:
        return self.reg_state == RegState.DONE

    def has_home_wifi(self) -> bool:
        return bool(self.home_wifi_ssid) and bool(self.home_wifi_psk)

@dataclass
class MainService:
    robot_name: str
    wired: WiredControlService
    mobile: MobileProvisionService
    amr: AmrService

    softap_cfg: SoftApConfig
    home_wifi_cfg: HomeWifiConfig
    mqtt_cfg: MqttConfig
    ui_mqtt: UiMqttService

    projector: BeamProjectorUtil

    test: int
    state_path: str = STATE_PATH_DEFAULT

    def _load_persisted(self) -> None:
        data = self._state_store.load()
        try:
            self._ctx.reg_state = RegState(data.get("reg_state", RegState.IDLE.value))
        except Exception:
            self._ctx.reg_state = RegState.IDLE

        self._ctx.home_wifi_ssid = data.get("home_ssid")
        self._ctx.home_wifi_psk = data.get("home_password")
        self._ctx.uuid_for_mqtt = data.get("uuid")
        self._ctx.smarthome_ip = data.get("ha_ip")
        self._ctx.smarthome_token = data.get("ha_token")

    def _save_persisted(self) -> None:
        data = {
            "reg_state": self._ctx.reg_state.value,
            "home_ssid": self._ctx.home_wifi_ssid,
            "home_password": self._ctx.home_wifi_psk,  
            "uuid": self._ctx.uuid_for_mqtt,
            "ha_ip": self._ctx.smarthome_ip,
            "ha_token": self._ctx.smarthome_token
        }
        self._state_store.save(data)

    def __post_init__(self) -> None:
        self._state_store = StateStore(self.state_path)
        self._ctx = RegistrationContext()
        self._softap = SoftApManager(self.softap_cfg)
        self._wifi = WifiManager(self.home_wifi_cfg)
        self._mqtt = MqttService(self.mqtt_cfg, self.amr, self.wired)
        self.test = 0
        self._last_status_push = 0.0
        self._load_persisted()

        if self._ctx.is_done():
            print("[boot] registration state: DONE")
        else:
            print(f"[boot] registration state: {self._ctx.reg_state.value}")

        if self._ctx.has_home_wifi():
            print(f"[boot] home wifi persisted: ssid= {self._ctx.home_wifi_ssid}")

    @property
    def state(self) -> RegState:
        return self._ctx.reg_state

    def is_wifi_registered(self) -> bool:
        """home WiFi 자격증명이 ctx에 존재하면 True.
        Bridge의 wifi_reg_fn 콜백으로 사용.
        _load_persisted() 호출 후 부팅 즉시 반영되며,
        _handle_mobile('provision') 완료 직후에도 즉시 True 반환.
        """
        return bool(self._ctx.home_wifi_ssid) and bool(self._ctx.home_wifi_psk)

    def start(self) -> None:
        print(f"[main] main service started (state= {self._ctx.reg_state})")

    def _send_to_ui(self, msg: dict) -> None:
        """wired(TCP) + ui_mqtt 두 채널에 동시 전송."""
        if self.wired.has_client:
            self.wired.send(msg)
        if self.ui_mqtt.has_client:
            self.ui_mqtt.send(msg)

    def tick(self) -> None:
        # wired messages (TCP) — BT 명령은 Bridge(BtLayer)가 처리, 레거시만 여기서 처리
        msg = self.wired.try_recv()
        while msg is not None:
            self._handle_wired(msg)
            msg = self.wired.try_recv()

        # ui_mqtt 메시지는 Bridge(BlackboardBridge)가 전담 처리.
        # MainService에서 try_recv()하면 Bridge보다 먼저 큐를 비워 BT 명령이 무시됨.
        # → MainService에서는 ui_mqtt.try_recv() 호출 금지.

        if (self._ctx.reg_state == RegState.REGISTED or self._ctx.reg_state == RegState.DONE) and self._mqtt.started:
            self._mqtt.tick()

        if self._ctx.is_done() == False:
            # mobile messages
            mmsg = self.mobile.try_recv()
            while mmsg is not None:
                self._handle_mobile(mmsg)
                mmsg = self.mobile.try_recv()

            # status push to orangepi (wired + ui_mqtt 동시 전송)
            now = time.monotonic()
            if now - self._last_status_push > 1.0:
                self._last_status_push = now
                self._send_to_ui({
                    "type": "REG_STATUS",
                    "payload": {
                        "state": self._ctx.reg_state.value,
                        "verify_ok": self._ctx.verify_ok,
                    },
                })

        # mqtt start
        if (self._ctx.reg_state == RegState.REGISTED or self._ctx.reg_state == RegState.DONE) and self.mqtt_cfg.enabled and not self._mqtt.started:
            self._mqtt.start(HomeAssistantConfig(ha_ip=self._ctx.smarthome_ip, token=self._ctx.smarthome_token, timeout_sec=5.0), self._state_store, self.projector)
            self.amr._pub = self._mqtt

            if self._ctx.reg_state == RegState.DONE and self._mqtt.started:
                self._mqtt.publish("robots/regist-result", {})
                self._ctx.reg_state = RegState.REGISTED
                self._save_persisted()
                if self.wired.has_client:
                    self.wired.send({"type": "response_robot_regist", "payload" : {
                    "sub_type": "REGIST_DONE"}})


    def stop(self) -> None:
        # 서비스 종료 시: mqtt stop, softap down 시도
        try:
            if self._mqtt.started:
                self._mqtt.stop()
        finally:
            try:
                self._softap.disable()
            except Exception:
                pass
        print("[main] main service stopped")

    # ---------------- handlers ----------------
    # Orangepi <-> Jetson 
    def _handle_wired(self, msg: dict) -> None:
        self.test += 1
        print("[main handle wired] received data :", msg.get("type"))
        requestType = msg.get("type")

        if requestType == "request_robot_regist":
            # (1) orangepi -> jetson 등록 시작
            # robot_sn = str(msg.get("robot_sn", "unknown"))
            self._ctx.robot_sn = self.get_jetson_serial_number()
            self._ctx.robot_model = self.robot_name
            print(f"[wired] Robot SerialNumber: {self._ctx.robot_sn}, Model: {self._ctx.robot_model}")
            print("[wired] REG_START from Orangepi")

            # (2) softap 전환 + 접속 정보(192.168.0.1:9990) 전달
            ssid = f"{self.softap_cfg.ssid}"
            psk = self.softap_cfg.psk

            self._ctx.softap_ssid = ssid
            self._ctx.softap_psk = psk

            # softap config 갱신 (IP는 요구사항: 192.168.0.1/24 고정)
            self.softap_cfg = SoftApConfig(
                wifi_ifname=self.softap_cfg.wifi_ifname,
                ssid=ssid,
                psk=psk,
                ip_cidr="192.168.0.1/24",
                ap_con_name=self.softap_cfg.ap_con_name,
                home_con_name=self.softap_cfg.home_con_name,
            )

            # orangepi에 softap 접속 정보 전달 (QR 표시용)
            self.wired.send({"type": "response_robot_regist", "payload" : {
                "sub_type": "SOFTAP_INFO",
                "softap_ip": "192.168.0.1",
                "softap_port": self.mobile.cfg.port,   # 9990
                "ssid": ssid,
                "psk": psk,
            }})

            self._softap = SoftApManager(self.softap_cfg)
            
            self._softap.enable()
            if not self.mobile.running:
                self.mobile.start()

            self._ctx.reg_state = RegState.SOFTAP_UP
            self._ctx.verify_ok = None
            self._ctx.verify_reason = ""
            self._ctx.registered_robot_id = None
            return

        if requestType == "request_projector_control":
            subType = msg.get("payload", {}).get("subType")

            if subType:
                if subType == "beam_start":
                    self.projector.beam_open()
                    self.projector.projector_off()
                    self.projector.projector_on()
                elif subType == "beam_off":
                    status = self.projector.video_status()
                    if status.data.get('state') == "PLAYING":
                        self.projector.video_stop()
                    self.projector.projector_off()
                    time.sleep(1.0)
                    self.projector.beam_close()
                    time.sleep(1.0)
                    self.projector.servo_off()
                elif subType == "beam_angle_top":
                    self.projector.servo_set_angle(angle_deg=25)
                elif subType == "video_play":
                    filePath = msg.get("payload", {}).get("video_file_path")
                    print(f"{filePath}")
                    if filePath:
                        self.projector.video_init()
                        time.sleep(1.0)
                        self.projector.video_play(file_path= filePath)
                    else:
                        print(f"/home/everybot/test.mp4")
                        self.projector.video_init()
                        time.sleep(1.0)
                        self.projector.video_play(file_path= "/home/everybot/test.mp4")
                elif subType == "video_stop":
                    self.projector.video_stop()
                elif subType == "video_status":
                    self.projector.video_status()
            
        if requestType == "request_servo_control":
            subType = msg.get("payload", {}).get("subType")

            if subType:
                if subType == "beam_open":
                    self.projector.beam_open()
                elif subType == "beam_close":
                    self.projector.beam_close()
                elif subType == "servo_set_angle":
                    angle = msg.get("payload", {}).get("angle")
                    print(f"{angle}")
                    if angle:
                        self.projector.servo_set_angle(angle_deg=angle)
                elif subType == "servo_off":
                    self.projector.servo_off()
        
        #로봇(orangepi) STT를 통한 스마트홈 기기 제어
        if (requestType == "request_iot_control"):
            print("[main handle wired] received iot command~~")
            if self._mqtt.started:
                self._mqtt._handle_iot_control(self._mqtt._uuid, msg)

                try:
                    time.sleep(0.6)
                    devices = self._mqtt._build_save_iot_devices_payload()
                    self._mqtt._publish_save_iot_devices(self._mqtt._uuid, devices)
                    log.info("[main handle wired][iot] published save-iot-devices count=%d", len(devices))
                    self.wired._send_obj({"type": "report_device_info", "payload": devices})
                except Exception as e:
                    log.exception("[main handle wired][iot] save-iot-devices publish failed: %s", e)
        
        if (requestType == "request_get_iot_device_info"):
            print("[main handle wired] received refresh iot device info~~")
            try:
                if self._mqtt.started:
                    devices = self._mqtt._build_save_iot_devices_payload()
                    self._mqtt._publish_save_iot_devices(self._mqtt._uuid, devices)
                    log.info("[main handle wired][iot] published save-iot-devices count=%d", len(devices))
                    self.wired._send_obj({"type": "report_device_info", "payload": devices})
            except Exception as e:
                log.exception("[main handle wired][iot] save-iot-devices publish failed: %s", e)


        # if t == "heartbeat":
        #     self.wired.send({"type": "PONG", "ts": time.time()})
        #     return

    # Mobile App <-> Jetson
    def _handle_mobile(self, msg: dict) -> None:
        t = msg.get("type")

        if t == "hello":
            self.mobile.send({
                "type": "robot_info",
                "state": self._ctx.reg_state.value,
                "model": self._ctx.robot_model,
                "robotId": self._ctx.robot_sn,
            })
            return

        if t == "provision_start":
            # (5) 모바일 -> jetson: 홈 wifi + 스마트홈 정보
            home = msg.get("home_wifi", {}) or {}
            smarthome = msg.get("smarthome", {}) or {}

            self._ctx.home_wifi_ssid = str(home.get("ssid", "")).strip()
            self._ctx.home_wifi_psk = str(home.get("psk", "")).strip()
            self._ctx.smarthome_ip = str(smarthome.get("ip", "")).strip()
            self._ctx.smarthome_token = str(smarthome.get("token", "")).strip()

            if not self._ctx.home_wifi_ssid or not self._ctx.home_wifi_psk:
                self.mobile.send({"type": "ERROR", "code": "INVALID_WIFI", "message": "home_wifi.ssid/psk required"})
                return

            self._ctx.reg_state = RegState.HOME_WIFI_CONNECTING

            # softap -> home wifi 전환은 연결이 끊길 수 있으니 ACK 먼저
            self.mobile.send({
                "type": "provision_start_ack",
                "next": "SWITCH_TO_HOME_WIFI",
                "hint": "Device will switch to home Wi-Fi (DHCP). Reconnect and call QUERY_STATUS."})
            
        if t == "provision":
            registInfo = msg.get("regist_info", {}) or {}
            self._ctx.uuid_for_mqtt = str(registInfo.get("uuid", "")).strip()

            if self.wired.has_client:
                self.wired.send({"type": "response_robot_regist", "payload" : {
                "sub_type": "TRY_CONNECT_HOME_WIFI", "home_ssid" : self._ctx.home_wifi_ssid}})

            # softap 내려 + home wifi 연결(DHCP). connection은 유지/재부팅 자동연결 목적
            try:
                try:
                    self._softap.disable()
                except Exception:
                    pass

                self._wifi.connect(self._ctx.home_wifi_ssid, self._ctx.home_wifi_psk)

            except Exception as e:
                self._ctx.verify_ok = False
                self._ctx.verify_reason = f"HOME_WIFI_CONNECT_FAILED: {e}"
                self._ctx.reg_state = RegState.HOME_WIFI_CONNECTING

                if self.wired.has_client:
                    self.wired.send(
                        {"type": "response_robot_regist", 
                         "payload" : 
                            {"sub_type": "FAIL_CONNECT_HOME_WIFI", "home_ssid" : self._ctx.home_wifi_ssid, "reson": self._ctx.verify_reason}})
                    return
                
            if self.wired.has_client:
                self.wired.send({"type": "response_robot_regist", "payload" : {
                "sub_type": "SUCCESS_CONNECT_HOME_WIFI", "home_ssid" : self._ctx.home_wifi_ssid}})
                self._ctx.verify_ok = True
                self._ctx.reg_state = RegState.DONE
                self._save_persisted()
                print(f"[Regist Done] {self._ctx.home_wifi_ssid}")
            return
    
    def get_jetson_serial_number(self) -> str:
        try:
            with open("/proc/device-tree/serial-number", "r") as f:
                # Read the content and strip any leading/trailing whitespace or null bytes
                serial_number = f.read().strip().strip('\x00')
            return serial_number
        except FileNotFoundError:
            print("Serial number file not found at /proc/device-tree/serial-number.")
            return None
        except Exception as e:
            print(f"An error occurred: {e}")
            return None
