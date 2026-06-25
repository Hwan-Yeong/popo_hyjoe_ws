from __future__ import annotations

import logging
import json
import math
import socket
import struct
import threading
import time
import zlib
from dataclasses import dataclass, field
from typing import Any

from ..interfaces.pubsub import Publisher
from .amr_constants import *

log = logging.getLogger(__name__)

def hexdump(b: bytes, maxlen=64):
    s = b[:maxlen]
    return " ".join(f"{x:02x}" for x in s) + (" ..." if len(b) > maxlen else "")

def find_json_span(raw: bytes):
    # Try object first, then array
    for (open_ch, close_ch) in ((ord('{'), ord('}')), (ord('['), ord(']'))):
        depth = 0
        in_string = False
        esc = False
        start = -1
        for i, x in enumerate(raw):
            if not in_string:
                if x == open_ch:
                    if depth == 0:
                        start = i
                    depth += 1
                elif x == close_ch and depth > 0:
                    depth -= 1
                    if depth == 0 and start != -1:
                        return (start, i)
                elif x == 0x22:  # "
                    in_string = True
                    esc = False
            else:
                if esc:
                    esc = False
                else:
                    if x == 0x5c:  # backslash
                        esc = True
                    elif x == 0x22:  # "
                        in_string = False
        # try next type
    return None

def extract_json(payload: bytes, json_len: int):
    # 1) header slice
    cand1 = payload[:json_len]
    span = find_json_span(cand1)
    if span:
        a, b = span
        return json.loads(cand1[a:b+1].decode("utf-8", errors="ignore"))
    # 2) whole payload
    span = find_json_span(payload)
    if span:
        a, b = span
        return json.loads(payload[a:b+1].decode("utf-8", errors="ignore"))
    # 3) utf-16 le/be attempts
    try:
        s = payload[:json_len].decode("utf-16-le", errors="ignore")
        a, b = s.find("{"), s.rfind("}")
        if a != -1 and b != -1 and b > a:
            return json.loads(s[a:b+1])
        a, b = s.find("["), s.rfind("]")
        if a != -1 and b != -1 and b > a:
            return json.loads(s[a:b+1])
    except Exception:
        pass
    try:
        s = payload[:json_len].decode("utf-16-be", errors="ignore")
        a, b = s.find("{"), s.rfind("}")
        if a != -1 and b != -1 and b > a:
            return json.loads(s[a:b+1])
        a, b = s.find("["), s.rfind("]")
        if a != -1 and b != -1 and b > a:
            return json.loads(s[a:b+1])
    except Exception:
        pass
    # 4) gzip
    if len(payload) >= 2 and payload[0] == 0x1f and payload[1] == 0x8b:
        try:
            decomp = zlib.decompress(payload, zlib.MAX_WBITS | 16)
            return extract_json(decomp, len(decomp))
        except Exception:
            pass
    raise ValueError("no JSON braces found")


def base64_map_convert_to_file(map_data_dict : dict):
    import base64
    import yaml

    map_info = map_data_dict.copy()
    width = int(map_info["width"])
    height = int(map_info["height"])
    resolution = map_info["resolution"]
    origin = [map_info["posX"], map_info["posY"], 0.0]
    
    raw_data = base64.b64decode(map_info["data"])
    
    print("decoded bytes:", len(raw_data), "expected:", width*height)
    header = f"P5\n{width} {height}\n255\n".encode("ascii")
    
    # map 파일 생성
    map_name = "map"
    with open(f"{map_name}.pgm", "wb") as f:
        f.write(header)
        f.write(raw_data)

    # YAML 파일 생성
    yaml_data = {
        "image": f"{map_name}.pgm",
        "resolution": resolution,
        "origin": origin,
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.25
    }

    with open(f"{map_name}.yaml", "w") as f:
        yaml.dump(yaml_data, f, default_flow_style=False)

    print(f"저장 완료: {map_name}.pgm, {map_name}.yaml")

@dataclass
class AmrService:
    """
    실제 AMR UDP 통신 서비스.

    AmrServiceProtocol 인터페이스를 구현하여 BT 노드에서 사용 가능.
    수신 루프(별도 스레드)가 상태를 캐시하고 BT 계층은 캐시를 읽는다.
    """

    amr_ip:   str = "192.168.60.206"
    amr_port: int = 10000
    recv_buf_size: int = 262144

    def __post_init__(self) -> None:
        self._started = False
        self._sock  = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._stop  = threading.Event()
        self._lock  = threading.Lock()
        self._thread: threading.Thread | None = None
        self._pub:    Publisher | None = None

        # ── 캐시 상태 (수신 스레드가 갱신, BT 계층이 읽기) ─────────
        self._moving_state: int  = MovingState.IDLE
        self._position:     dict = {"x": 0.0, "y": 0.0, "theta": 0.0}
        self._battery_pct:  float = 100.0
        self._arrived_flag: bool = False
        self._prev_state:   int  = MovingState.IDLE

        # ── 최신 맵 데이터 (ActionSaveMap 이 읽는다) ─────────────────
        self.latest_map_data: dict = {}

        # ── 주행 실패 플래그 (ActionNavigateTo 재시도용) ─────────────
        self._drive_failed: bool = False

        # ── 최신 RobotStatus (MapCreationServer /api/done 검증용) ────
        self._robot_status: int = 0

        # ── AllMovingInfo 유효 플래그 (Phase 2 UI 표시용) ─────────────
        self._valid_position:        bool = False
        self._valid_target_position: bool = False

    # ── 서비스 생명주기 ─────────────────────────────────────────────

    @property
    def started(self) -> bool:
        return self._started

    def start(self) -> None:
        if self._started:
            return
        # 맵 데이터(base64 PGM ~71KB)가 단일 UDP 프레임을 초과할 수 있으므로
        # OS 소켓 수신 버퍼를 256KB로 확장 (기본 ~8KB)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, self.recv_buf_size)
        # bind 먼저 완료 후 스레드 시작 → send/recv 모두 10000 포트 사용
        self._sock.bind(("0.0.0.0", self.amr_port))
        self._sock.settimeout(0.8)
        self._thread = threading.Thread(target=self._run, name="amr-service", daemon=True)
        self._thread.start()
        self._started = True

    def tick(self) -> None:
        pass

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        self._sock.close()
        self._started = False

    # ── AmrServiceProtocol ──────────────────────────────────────────

    @property
    def cached_moving_state(self) -> int:
        with self._lock:
            return self._moving_state

    @property
    def cached_position(self) -> dict:
        with self._lock:
            return dict(self._position)

    @property
    def cached_robot_status(self) -> int:
        """마지막 수신된 RobotStatus.status (0=IDLE, 1=MOVING, 7=충전스테이션 위치)."""
        with self._lock:
            return self._robot_status

    def pop_arrived_event(self) -> bool:
        """엣지 트리거: IDLE 전환 순간 1회만 True 반환."""
        with self._lock:
            if self._arrived_flag:
                self._arrived_flag = False
                return True
        return False

    def pop_drive_failed(self) -> bool:
        """주행 불가 응답 수신 시 1회만 True 반환 (ActionNavigateTo 재시도용)."""
        with self._lock:
            if self._drive_failed:
                self._drive_failed = False
                return True
        return False

    @property
    def battery_percent(self) -> float:
        with self._lock:
            return self._battery_pct

    @property
    def cached_valid_position(self) -> bool:
        """AllMovingInfo.validPosition — 현재 위치 유효 여부."""
        with self._lock:
            return self._valid_position

    @property
    def cached_valid_target_position(self) -> bool:
        """AllMovingInfo.validTargetPosition — 목적지 유효(주행 명령 활성) 여부."""
        with self._lock:
            return self._valid_target_position

    def send_target_position(self, coord: dict) -> None:
        """cmd=60 목적지 설정만 전송 (cmd=61 없음, type 필드 없음).

        Phase 2 nav_test 에서 사용 — 직접 amr_api_test.py 테스트와 동일한 포맷.
        send_nav_cmd 와 달리 Driving START(cmd=61) 을 즉시 전송하지 않는다.
        """
        x, y, theta = (coord.get(k, 0.0) for k in ("x", "y", "theta"))
        body = {"Request": {"Set": {"TargetPosition": {"x": x, "y": y, "theta": theta}}}}
        self._send(AmrCmd.TARGET_POSITION, AmrDtype.SET, body)
        log.info("[AMR] send_target_position → (%.2f, %.2f, %.2f)", x, y, theta)

    def send_nav_cmd(self, coord: dict) -> None:
        """목적지 설정(cmd=60) + 주행 시작(cmd=61) 연속 전송."""
        x, y, theta = (coord.get(k, 0.0) for k in ("x", "y", "theta"))
        self._send(AmrCmd.TARGET_POSITION, AmrDtype.SET,
                   build_nav_request(x, y, theta))
        self._send(AmrCmd.DRIVING, AmrDtype.SET,
                   build_driving_request(DrivingSet.START))
        log.info("[AMR] send_nav_cmd → (%.2f, %.2f, %.2f)", x, y, theta)

    def send_stop(self) -> None:
        """AMR 즉시 정지(cmd=61 STOP)."""
        self._send(AmrCmd.DRIVING, AmrDtype.SET,
                   build_driving_request(DrivingSet.STOP))
        log.info("[AMR] send_stop")

    def send_raw_cmd(self, cmd: int, dtype: int, args: dict) -> None:
        """저수준 직접 전송 (MapCreation, SaveMap 등 특수 명령용)."""
        self._send(cmd, dtype, args)
        log.debug("[AMR] send_raw_cmd cmd=%d dtype=%d", cmd, dtype)

    def send_driving_start(self) -> None:
        """cmd=61 Driving set=1 (주행 시작). Phase 2 NavTo 테스트 시 사용."""
        self._send(AmrCmd.DRIVING, AmrDtype.SET,
                   build_driving_request(DrivingSet.START))
        log.info("[AMR] send_driving_start")

    def send_manual_vw(self, ms: float, rads: float) -> None:
        """cmd=59 수동 조종 (MapCreation 웹서버 → 키보드 조종)."""
        self._send(AmrCmd.MOTOR_MANUAL_VW, AmrDtype.SET,
                   build_manual_vw_request(ms, rads))

    def send_mapping_start(self, manual: bool = True) -> None:
        """cmd=62 맵 작성 시작 (manual=True → set=1, False → set=2)."""
        action = MappingSet.START_MANUAL if manual else MappingSet.START_AUTO
        self._send(AmrCmd.MAPPING, AmrDtype.SET,
                   build_mapping_request(action))
        log.info("[AMR] mapping start (manual=%s)", manual)

    def send_mapping_stop(self) -> None:
        """cmd=62 맵 작성 정지 (set=4)."""
        self._send(AmrCmd.MAPPING, AmrDtype.SET,
                   build_mapping_request(MappingSet.STOP))
        log.info("[AMR] mapping stop")

    def send_save_map(self) -> None:
        """cmd=87 맵 저장 명령."""
        self._send(AmrCmd.SAVE_MAP, AmrDtype.SET, build_save_map_request())
        log.info("[AMR] save_map sent")

    def send_software_reset(self) -> None:
        """AMR 소프트웨어 리셋. 맵 저장 후 AMR 재초기화 시 사용."""
        self._send(AmrCmd.SOFTWARE_RESET, AmrDtype.SET, build_software_reset_request())
        log.info("[AMR] send_software_reset")

    def request_map_data(self) -> None:
        """cmd=15 맵 데이터 요청 (응답은 수신 루프에서 latest_map_data 갱신)."""
        self._send(AmrCmd.MAP_DATA, AmrDtype.GET,
                   build_get_request("MapData"))
        log.info("[AMR] request_map_data sent")

    def request_moving_info(self) -> None:
        """cmd=41 AllMovingInfo 요청."""
        self._send(AmrCmd.ALL_MOVING_INFO, AmrDtype.GET,
                   build_get_request("AllMovingInfo"))
        
    def send_return_charging_station(self) -> None:
        """cmd=50 충전 스테이션 복귀 명령."""
        self._send(AmrCmd.RETURN_CHARGING_STATION, AmrDtype.SET, build_return_charging_station_request())
        log.info("[AMR] return charging station sent")
    
    def send_station_repositioning(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0) -> None:
        """cmd=77 충전 스테이션 위치 재설정."""
        self._send(AmrCmd.STATION_REPOSITIONING, AmrDtype.SET, build_station_repositioning_request(x,y,theta))
        log.info("[AMR] station repositioning sent  → (%.2f, %.2f, %.2f)", x, y, theta)

    def send_bypass(
        self,
        block_areas: list[dict] | None = None,
        block_walls: list[dict] | None = None,
        charging_station: dict | None = None,
    ) -> None:
        """cmd=72 ByPass 전송."""
        body = build_bypass_request(block_areas, block_walls, charging_station)
        self.send_raw_cmd(AmrCmd.BYPASS, AmrDtype.SET, body)
        log.info(
            "[AMR] send_bypass areas=%d walls=%d stations=%d",
            len(block_areas or []),
            len(block_walls or []),
            1 if charging_station else 0,
        )

    def send_rotation(self, rot_type: int, radian: float) -> None:
        """cmd=66 Rotation 전송."""
        body = build_rotation_request(rot_type, radian)
        self.send_raw_cmd(AmrCmd.ROTATION, AmrDtype.SET, body)
        log.info("[AMR] send_rotation type=%d radian=%.3f", rot_type, radian)

    def is_in_zone(self, key: str, zone_mgr: Any) -> bool:
        """현재 로봇 위치가 key zone 내부인지 판단."""
        pos = self.cached_position
        return zone_mgr.is_point_in_zone(key, pos.get("x", 0.0), pos.get("y", 0.0))

    def estimate_object_world_pos(self, distance_m: float, angle_deg: float = 0.0) -> dict:
        """현재 로봇 pose와 거리/각도 기반으로 사물의 월드 좌표를 추정."""
        pos = self.cached_position
        theta = float(pos.get("theta", 0.0))
        obj_angle = theta + math.radians(angle_deg)
        return {
            "x": float(pos.get("x", 0.0)) + distance_m * math.cos(obj_angle),
            "y": float(pos.get("y", 0.0)) + distance_m * math.sin(obj_angle),
        }


    # ── Private ─────────────────────────────────────────────────────

    def _send(self, cmd: int, dtype: int, body: dict) -> None:
        """UDP 헤더 + JSON body 전송."""
        payload = json.dumps(body, ensure_ascii=False).encode("utf-8")
        header  = struct.pack("!IHHBB", len(payload), 1, 1, cmd, dtype)
        try:
            self._sock.sendto(header + payload, (self.amr_ip, self.amr_port))
        except OSError as e:
            log.warning("[AMR] _send failed cmd=%d: %s", cmd, e)

    def _run(self) -> None:
        HDR_FMT  = "!IHHBB"
        HDR_LEN  = struct.calcsize(HDR_FMT)   # 10
        # 맵 데이터 JSON (base64 PGM 53300px → ~71KB) 수신을 위해 65535+여유로 확장.
        # 이전 0x8000(32768)은 맵 단일 프레임(~35KB+)을 묵시적으로 절단
        RECV_MAX = 65536 + HDR_LEN

        pay = bytes()
        while not self._stop.is_set():
            try:
                dat, addr = self._sock.recvfrom(RECV_MAX)
            except socket.timeout:
                continue
            jl, f1, f2, cmd, obj = struct.unpack(HDR_FMT, dat[:HDR_LEN])

            #print(f"from {addr} len {jl} flags {f1}/{f2} cmd {cmd} obj 0x{obj:02X}")

            total = f1
            idx   = f2

            chunk = dat[HDR_LEN:HDR_LEN+jl]  # jl 만큼만
            if total <= 1 or idx in (0, 1):  # 단일패킷 또는 첫조각이면 새로 시작
                pay = b""
            pay += chunk

            # 마지막 조각이 아니면 계속 받기
            if total > 1 and idx < total:
                continue

            # if True:
            #     print("head:", hexdump(dat[:HDR_LEN]))
            #     print("payH:", hexdump(pay[:jl]))
            try:
                js = extract_json(pay, jl)
                
                # ── Set 응답 파싱 (주행 실패 감지) ─────────────────────
                resp_set = js.get("Response", {}).get("Set", {})
                if resp_set:
                    set_key = list(resp_set.keys())[0]
                    if set_key == "Driving":
                        drive_resp = resp_set.get("Driving", {})
                        result = drive_resp.get("result", True)
                        if not result:
                            with self._lock:
                                self._drive_failed = True
                            log.warning("[AMR] drive failed response: %s", drive_resp)

                # ── Get 응답 파싱 ────────────────────────────────────────
                resp_api = js.get("Response", {}).get("Get", {})

                # 빈 dict 가드 — 키가 없으면 IndexError 발생 방지
                if resp_api:

                    #req_api_key = list(req_api.keys())[0]
                    resp_api_key = list(resp_api.keys())[0]
                    
                    if resp_api_key == 'MapData':
                        map_data_dict = resp_api.get('MapData', {})
                        # BT 노드(ActionSaveMap)를 위해 캐시
                        with self._lock:
                            self.latest_map_data = map_data_dict
                        if self._pub:
                            self._pub.publishMap(map_data_dict)

                    if resp_api_key == 'AllMovingInfo':
                        moving_data_dict = resp_api.get('AllMovingInfo', {})
                        pos   = moving_data_dict.get('Position', {})
                        state = int(moving_data_dict.get('movingState', MovingState.IDLE))
                        valid_pos    = bool(moving_data_dict.get('validPosition', False))
                        valid_target = bool(moving_data_dict.get('validTargetPosition', False))
                        with self._lock:
                            prev = self._moving_state
                            self._moving_state = state
                            # x/y/theta 키를 소문자로 정규화
                            self._position = {
                                "x":     float(pos.get("x", pos.get("X", 0.0))),
                                "y":     float(pos.get("y", pos.get("Y", 0.0))),
                                "theta": float(pos.get("theta", pos.get("Theta", 0.0))),
                            }
                            self._valid_position        = valid_pos
                            self._valid_target_position = valid_target
                            # 도착 감지: MOVING→IDLE 또는 MOVING→ARRIVED(2)
                            # AMR 펌웨어가 목적지 도착 시 movingState=2 를 반환함.
                            # MOVING→IDLE 은 정지/취소도 포함하므로 양쪽 모두 처리.
                            if prev == MovingState.MOVING and state in (
                                MovingState.IDLE, MovingState.ARRIVED
                            ):
                                self._arrived_flag = True
                                label = "IDLE" if state == MovingState.IDLE else "ARRIVED"
                                log.info("[AMR] arrived event (MOVING→%s)", label)
                        if self._pub:
                            self._pub.publishCurPos(pos)
                            self._pub.publishCurMovingStatus(state)
                        log.debug("[AMR] AllMovingInfo state=%d pos=%s", state, pos)

                    if resp_api_key == "RobotStatus":
                        status_data_dict = resp_api.get('RobotStatus', {})
                        st = int(status_data_dict.get('status', 0))
                        with self._lock:
                            self._robot_status = st
                        if self._pub:
                            self._pub.publishCurStatus(st)

                    if resp_api_key == "BatteryStatus":
                        battery_data_dict = resp_api.get('BatteryStatus', {})
                        pct = float(battery_data_dict.get('batteryPercent', 0.0))
                        with self._lock:
                            self._battery_pct = pct
                        if self._pub:
                            self._pub.publishCurBatteryPercent(pct)

                    ##print(json.dumps(js, ensure_ascii=False, indent=2)[:2000]) #디버깅 로그 줄이는 용도로 2000까지만.
                    #print(f"{resp_api_key} API Complete")
                        
            except Exception as e:
                print("parse fail:", e)
                #if True:
                    #print("raw:", hexdump(pay, 256))
