from __future__ import annotations

import datetime
import json
import logging
import struct
import time
from dataclasses import dataclass
from typing import Any, Callable, Optional, TYPE_CHECKING

from everybot.services.wired_control_service import WiredControlService
from everybot.utils.beam_projector_util import BeamProjectorUtil
from everybot.utils.state_store import StateStore
from everybot.services.amr_service import AmrService

from ..interfaces.mqtt_client import MqttClient, MqttConfig
from ..interfaces.ha_client import HomeAssistantClient, HomeAssistantConfig

if TYPE_CHECKING:
    from ..bt.blackboard import RobotBlackboard
    from .ui_mqtt_service import UiMqttService

log = logging.getLogger(__name__)


def _utc_now() -> str:
    return datetime.datetime.utcnow().strftime("%Y-%m-%dT%H:%M:%SZ")


@dataclass
class MqttService:
    cfg: MqttConfig
    amr: AmrService
    wired: WiredControlService
    
    def __post_init__(self) -> None:
        self._client: MqttClient | None = None
        self._started = False
        self._uuid = ""
        self._userName = ""
        self._save_iot_domains = {"vacuum", "sensor", "switch"}  
        self._save_iot_entity_ids = set()          
        self._ha: HomeAssistantClient | None = None
        self._haCfg: HomeAssistantConfig | None = None
        self._store : StateStore | None = None
        self._projector : BeamProjectorUtil | None = None
        self._last_pub = time.monotonic()
        self._bb: RobotBlackboard | None = None
        self._ui_mqtt: UiMqttService | None = None
        self._status_fn: Callable[[], dict] | None = None

    def getUserName(self) -> str:
        return self._userName

    @property
    def started(self) -> bool:
        return self._started

    def set_bb(self, bb: "RobotBlackboard") -> None:
        self._bb = bb

    def set_ui_mqtt(self, ui_mqtt: "UiMqttService") -> None:
        self._ui_mqtt = ui_mqtt

    def set_status_fn(self, fn: Callable[[], dict]) -> None:
        self._status_fn = fn

    def start(self, haCfg : HomeAssistantConfig, stateStore : StateStore, projectorUtil : BeamProjectorUtil) -> None:
        if self._started or not self.cfg.enabled:
            return
        
        self._store = stateStore
        self._uuid = self._store.load().get("uuid")
        self._client = MqttClient(self.cfg)
        self._client.set_on_message(self._on_message)
        self._projector = projectorUtil

        self._client.connect()
        self._client.loop_start()
        self._haCfg = haCfg

        sn = self.get_jetson_serial_number()
        self._client.subscribe(f"robots/{sn}/regist")

        self._started = True
        log.info("[mqtt] started %s:%d", self.cfg.host, self.cfg.port)

        if (self._uuid != ""):
            self._client.subscribe(f"robots/{self._uuid}/cmd")
            self._ensure_ha_client()
            try:
                devices = self._build_save_iot_devices_payload()
                self._publish_save_iot_devices(self._uuid, devices)
                log.info("[mqtt][iot] published %s save-iot-devices count=%d", self._uuid, len(devices))
                if self.wired.has_client:
                    self.wired._send_obj({"type": "report_device_info", "payload": devices})
            except Exception as e:
                log.exception("[mqtt][iot] save-iot-devices publish failed: %s", e)

    def tick(self) -> None:
        """
        UI MQTT 주기적 robot_status 전송 (5초 주기).
        외부 MQTT 전송은 publish_external_status() 를 이벤트 발생 시 호출.
        """
        if not self._started:
            return
        now = time.monotonic()
        if now - self._last_pub < 5.0:
            return
        self._last_pub = now

        net_info = self._status_fn() if self._status_fn else {}
        if self._bb is not None:
            from ..utils.event_code import derive_status

            status = derive_status(self._bb)
            event_code = self._bb.current_event_code
            battery = round(self._bb.battery_percent, 1)
        else:
            status, event_code, battery = "IDLE", "Normal", 0

        timestamp = _utc_now()
        payload = {
            "robot_uuid": self._uuid,
            "timestamp": timestamp,
            "netstat": net_info.get("netstat", 1),
            "net_ssid": net_info.get("net_ssid", ""),
            "event_code": event_code,
            "status": status,
            "battery": battery,
        }

        if self._ui_mqtt is not None:
            self._ui_mqtt.send({"type": "robot_status", "payload": payload})
            self._ui_mqtt.send({
                "type": "heartbeat",
                "payload": {
                    "robot_uuid": self._uuid,
                    "timestamp": timestamp,
                    "state": "online",
                },
            })

    def stop(self) -> None:
        if not self._started or not self._client:
            return
        try:
            self._client.loop_stop()
            self._client.disconnect()
        finally:
            self._client = None
            self._started = False
        log.info("[mqtt] stopped")

    def publish(self, topic: str, payload: dict) -> None:
        if not self._started or not self._client:
            return
        self._client.publish(f"{topic}", json.dumps(payload, ensure_ascii=False))

    def publish_external_status(self, event_code: str, status: str, battery: float) -> None:
        if not self._started or not self._client or not self._uuid:
            return
        payload = {
            "robot_uuid": self._uuid,
            "timestamp": _utc_now(),
            "event_code": event_code,
            "status": status,
            "battery": round(battery, 1),
        }
        self._client.publish(
            f"robots/{self._uuid}/status",
            json.dumps(payload, separators=(",", ":")),
            qos=0,
            retain=False,
        )
        log.info("[mqtt][ext] published status event_code=%s status=%s", event_code, status)

    def subscribe(self, topic: str) -> None:
        if not self._started or not self._client:
            return
        self._client.subscribe(topic)

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
               
    def _publish_save_iot_devices(self, uuid: str, devices: dict) -> None:
        if not uuid:
            return
        topic = f"robots/{uuid}/save-iot-devices"
        self._client.publish(topic, json.dumps(devices, ensure_ascii=False))

    def publishMap(self, mapData: dict) -> None:
        topic = f"robots/{self._uuid}/rt-map"
        #print(f"rtmap ~~ {mapData}")
        self._client.publish(topic, json.dumps(mapData, ensure_ascii=False))

    def publishCurPos(self, posData: dict) -> None:
        topic = f"robots/{self._uuid}/rt-cur-pos"
        self._client.publish(topic, json.dumps(posData, ensure_ascii=False))

    def publishCurMovingStatus(self, movingStatus: int) -> None:
        topic = f"robots/{self._uuid}/rt-cur-moving-status"
        movingStatus = {
            "moving_status" : movingStatus
        }
        self._client.publish(topic, json.dumps(movingStatus, ensure_ascii=False))

    def publishCurStatus(self, statusData: int) -> None:
        topic = f"robots/{self._uuid}/heartbeat"
        status = {
            "state": "online",
            "robot_status": statusData
        }
        #print(f"rstatus ~~ {status}")
        self._client.publish(topic, json.dumps(status, ensure_ascii=False))

    def publishCurBatteryPercent(self, batteryPercent: float) -> None:
        topic = f"robots/{self._uuid}/rt-cur-battery"
        battery = {
            "battery": batteryPercent
        }
        print(f"rbattery ~~ {battery}")
        self._client.publish(topic, json.dumps(battery, ensure_ascii=False))

    def _build_save_iot_devices_payload(self) -> dict:
        self._ensure_ha_client()
        if getattr(self, "_ha", None) is None:
            raise RuntimeError("HomeAssistant client not ready")

        states = self._ha.list_states() or []

        want_ids = getattr(self, "_save_iot_entity_ids", set()) or set()
        want_domains = getattr(self, "_save_iot_domains", set()) or set()

        out: dict = {}

        for s in states:
            entity_id = str(s.get("entity_id", "") or "")
            if not entity_id:
                continue

            domain = entity_id.split(".", 1)[0] if "." in entity_id else ""
            domainTail = entity_id.split(".", 1)[1] if "." in entity_id else ""

            if want_ids:
                if entity_id not in want_ids:
                    continue
            else:
                if want_domains and domain not in want_domains:
                    continue

            if domain == "switch" and not domainTail.__contains__("plug"):
                continue

            attrs = s.get("attributes") or {}
            out[entity_id] = {
                "state": s.get("state"),
                "domain": domain,
                "last_changed": s.get("last_changed"),
                "friendly_name": attrs.get("friendly_name"),
            }

        return out

    def _publish_ack(self, uuid: str, idem_key: str) -> None:
        if not uuid or not idem_key:
            return
        topic = f"robots/{uuid}/cmd-ack"
        self._client.publish(topic, json.dumps({"idempotencyKey": idem_key}, ensure_ascii=False))

    def _publish_result(self, uuid: str, idem_key: str, ok: bool, result: Any = None, error: Optional[str] = None) -> None:
        if not uuid or not idem_key:
            return
        topic = f"robots/{uuid}/cmd-ack"
        payload: dict[str, Any] = {"idempotencyKey": idem_key, "ok": ok}
        if ok:
            payload["result"] = result
        else:
            payload["error"] = error or "unknown_error"
        self._client.publish(topic, json.dumps(payload, ensure_ascii=False))

    def _ensure_ha_client(self) -> None:
        if getattr(self, "_ha", None) is not None:
            return
        ha_cfg = getattr(self, "_haCfg", None)
        if ha_cfg is None:
            return
        # _haCfg가 HomeAssistantConfig가 아니라면 여기서 맞춰야 함
        if isinstance(ha_cfg, HomeAssistantConfig):
            self._ha = HomeAssistantClient(ha_cfg)
        else:
            # ha_cfg가 dict처럼 들어온 경우 대비
            self._ha = HomeAssistantClient(HomeAssistantConfig(**ha_cfg))

    def _handle_iot_control(self, uuid: str, json_payload: dict) -> None:
        """
        json_payload:
          {"type":"request_iot_control","payload":{"control":"turn_on","entityId":"switch.xxx","key":"idem"}}
        """
        payload = json_payload.get("payload") or {}
        control = str(payload.get("control", "")).strip()
        entity_id = str(payload.get("entityId", "")).strip()
        idem_key = json_payload.get("idempotencyKey") or {}

        if not idem_key:
            idem_key = ""
            log.info("[mqtt][iot] missing payload.key(idempotencyKey) -> drop")

        self._ensure_ha_client()
        if getattr(self, "_ha", None) is None:
            self._publish_result(uuid, idem_key, False, error="HomeAssistant client not ready (missing ha config/token?)")
            return

        try:
            if control in ("turn_on", "turn_off", "start", "pause", "stop", "return_to_base"):
                if not entity_id or "." not in entity_id:
                    self._publish_result(uuid, idem_key, False, error="entityId required (e.g. switch.xxx)")
                    return
                domain = entity_id.split(".", 1)[0]
                res = self._ha.call_service(domain, control, {"entity_id": entity_id})
                self._publish_result(uuid, idem_key, True, result={"control": control, "entityId": entity_id, "ha": res})
                return

            if control == "get_state":
                if not entity_id:
                    self._publish_result(uuid, idem_key, False, error="entityId required")
                    return
                s = self._ha.get_state(entity_id)
                if s is None:
                    self._publish_result(uuid, idem_key, False, error="state lookup failed")
                    return
                self._publish_result(uuid, idem_key, True, result=s)
                return

            if control == "list_entities":
                # optional: payload.domain
                domain = str(payload.get("domain", "") or "").strip().lower()
                states = self._ha.list_states() or []
                if domain:
                    states = [x for x in states if str(x.get("entity_id", "")).startswith(domain + ".")]

                compact = [
                    {
                        "entityId": x.get("entity_id"),
                        "state": x.get("state"),
                        "name": (x.get("attributes") or {}).get("friendly_name"),
                    }
                    for x in states
                ]
                self._publish_result(uuid, idem_key, True, result={"count": len(compact), "entities": compact})
                return

            if control == "list_entities_with_states":
                domain = str(payload.get("domain", "") or "").strip().lower()
                states = self._ha.list_states() or []
                if domain:
                    states = [x for x in states if str(x.get("entity_id", "")).startswith(domain + ".")]

                compact = []
                for x in states:
                    attrs = x.get("attributes") or {}
                    compact.append(
                        {
                            "entityId": x.get("entity_id"),
                            "state": x.get("state"),
                            "name": attrs.get("friendly_name"),
                            "deviceClass": attrs.get("device_class"),
                            "unit": attrs.get("unit_of_measurement"),
                        }
                    )
                self._publish_result(uuid, idem_key, True, result={"count": len(compact), "entities": compact})
                return

            self._publish_result(uuid, idem_key, False, error=f"unknown control: {control}")

        except Exception as e:
            log.exception("[mqtt][iot] failed: %s", e)
            self._publish_result(uuid, idem_key, False, error=str(e))

    def _on_message(self, topic: str, payload: bytes) -> None:
        try:
            sn = self.get_jetson_serial_number()
            json_payload = json.loads(payload.decode('utf-8'))
            log.info("[mqtt] rx topic=%s payload=%s", topic, payload[:200])

            if (topic == f"robots/{sn}/regist"):
                if (json_payload['type'] == "robot_regist_done"):
                    self._uuid = json_payload['payload']['uuid']
                    self._userName = json_payload['payload']['userName']
                    print("[mqtt] received robot regist result data")
                    self._client.subscribe(f"robots/{self._uuid}/cmd")
                    self._ensure_ha_client()
                    savedState = self._store.load()
                    data = {
                        "reg_state": savedState.get("reg_state"),
                        "home_ssid": savedState.get("home_ssid"),
                        "home_password": savedState.get("home_password"),  
                        "uuid": self._uuid,
                        "ha_ip": savedState.get("ha_ip"),
                        "ha_token": savedState.get("ha_token")
                    }
                    self._store.save(data)
                    try:
                        devices = self._build_save_iot_devices_payload()
                        self._publish_save_iot_devices(self._uuid, devices)
                        log.info("[mqtt][iot] published save-iot-devices count=%d", len(devices))
                        if self.wired.has_client:
                            self.wired._send_obj({"type": "report_device_info", "payload": devices})
                    except Exception as e:
                        log.exception("[mqtt][iot] save-iot-devices publish failed: %s", e)

            elif (topic == f"robots/{self._uuid}/cmd"):
                if (json_payload['type'] == "request_iot_control"):
                    print("[mqtt] received iot command~~")
                    self._handle_iot_control(self._uuid, json_payload)

                    try:
                        time.sleep(0.6)
                        devices = self._build_save_iot_devices_payload()
                        self._publish_save_iot_devices(self._uuid, devices)
                        log.info("[mqtt][iot] published save-iot-devices count=%d", len(devices))
                        if self.wired.has_client:
                            self.wired._send_obj({"type": "report_device_info", "payload": devices})
                    except Exception as e:
                        log.exception("[mqtt][iot] save-iot-devices publish failed: %s", e)

                if (json_payload['type'] == "request_amr_control"):
                    print("[mqtt] received amr command~~")
                    body = json.dumps(json_payload['payload']['args'], separators=(",", ":"), ensure_ascii=False).encode("utf-8")
                    hdr = struct.pack("!IHHBB", len(body), 1, 1, json_payload['payload']['cmd'], json_payload['payload']['type'])
                    pkt = hdr + body
                    self.amr._sock.sendto(pkt, (self.amr._amrIp, self.amr._amrPort))

                if (json_payload['type'] == "request_projector_control"):
                    print("[mqtt] received beam command~~")
                    subType = json_payload['payload']['subType']

                    if subType:
                        if subType == "beam_start":
                            self._projector.beam_open()
                            self._projector.projector_off()
                            self._projector.projector_on()
                        elif subType == "beam_off":
                            status = self._projector.video_status()
                            if status.data.get('state') == "PLAYING":
                                self._projector.video_stop()
                            self._projector.projector_off()
                            time.sleep(1.0)
                            self._projector.beam_close()
                            time.sleep(1.0)
                            self._projector.servo_off()
                        elif subType == "beam_angle_top":
                            self._projector.servo_set_angle(angle_deg=25)
                        elif subType == "video_play":
                            self._projector.video_init()
                            time.sleep(1.0)
                            self._projector.video_play(file_path= "/home/everybot/test.mp4")
                        elif subType == "video_stop":
                            self._projector.video_stop()
                        elif subType == "video_status":
                            self._projector.video_status()
            else:
                print("[mqtt] received command~~")
        except Exception as e:
            print(f"[mqtt] onMessage Exception ~~ {e}")
        
        
       

        

