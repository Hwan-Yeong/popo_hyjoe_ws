from __future__ import annotations

"""
AMR API 상수 모듈.
AMR UDP 프로토콜에서 사용하는 cmd / dtype 값과
요청 JSON 빌더 함수를 한 곳에 정의한다.
Mock / Real 서비스 모두 이 모듈의 값을 공유한다.
"""


class AmrCmd:
    # ── Get (dtype=1) ──────────────────────────────────────────
    ERROR_LIST      = 12
    MAP_STATUS      = 14
    MAP_DATA        = 15
    ROBOT_STATUS    = 21
    ACTION_STATUS   = 22
    ALL_MOVING_INFO = 43 # 41은 동작 안함
    ALL_STATUS      = 42

    # ── Set (dtype=2) ──────────────────────────────────────────
    MOTOR_MANUAL_VW = 59   # 수동 조종 (linear mS, angular radS)
    TARGET_POSITION = 60   # 목적지 설정 + 주행 명령
    DRIVING         = 61   # 주행 Start / Pause / Resume / Stop
    MAPPING         = 62   # 맵 작성 Start / Stop
    ROTATION        = 66   # 로봇 회전 명령
    BYPASS          = 72   # 금지영역/관심영역/충전위치 일괄 설정
    SAVE_MAP        = 87   # 맵 저장
    SOFTWARE_RESET  = 56   # 소프트웨어 리셋
    START_CHARGING  = 54   # 충전 시작 (왠만하면 안씀)
    STOP_CHARGING  = 55   # 충전 중지
    RETURN_CHARGING_STATION = 50 # 충전 스테이션 복귀 (이어서 자동 충전까지)
    RECOVERY_RETURN_CHARGING_STATION = 88 # 리커버리 후 충전 스테이션 복귀 (이어서 자동 충전까지)
    STATION_REPOSITIONING = 77 # 충전 스테이션 위치 재설정
    

class AmrDtype:
    GET = 1
    SET = 2


class DrivingSet:
    START  = 1
    PAUSE  = 2
    RESUME = 3
    STOP   = 4

class MappingSet:
    START_MANUAL = 1
    START_AUTO   = 2
    STOP         = 4


class RotationType:
    """cmd=66 Rotation.type 상수."""
    DIFF      = 0
    POSE      = 1
    AUTO      = 2
    LEFT_360  = 3
    RIGHT_360 = 4


# ── 로봇 상태 코드 (RobotStatus) ─────────────────
class RobotStatus:
    IDLE = 0
    AUTO_MAPPING = 1
    MANUAL_MAPPING = 2
    NAVIGATION = 3
    RETURN_CHARGER = 4
    DOCKING = 5
    UNDOCKING = 6
    ONSTATION = 7
    FACTORY_NAVIGATION = 8
    ERROR = 9
    FOLLOW_ME = 10


# ── 동작 상태 코드 (ActionStatus) ─────────────────
class ActionStatus:
    VOID = 0
    READY = 1
    START = 2
    PAUSE = 3
    RESUME = 4
    COMPLETE = 5
    FAIL = 6


# ── 이동 상태 코드 (AllMovingInfo.movingState) ─────────────────
class MovingState:
    IDLE    = 0
    MOVING  = 1
    ARRIVED = 2   # AMR 펌웨어 확장값: 목적지 도착/주행 완료
    #              MOVING(1) → ARRIVED(2) 전이가 실제 도착 신호
    #              MOVING(1) → IDLE(0)   전이는 정지/취소 경우도 포함
    PAUSE = 3
    FAIL = 4
    START_ROTATION = 5
    ROTATION_END = 6
    READY = 7
    ALTERNATIVE_GOAL = 8  # 대체 목표 도착도 주행 완료로 처리
    FOLLOW_ME = 9


# ── 요청 JSON 빌더 ─────────────────────────────────────────────

def build_nav_request(x: float, y: float,
                      theta: float = 0.0,
                      nav_type: int = 1) -> dict:
    """cmd=60 목적지 설정 요청."""
    return {"Request": {"Set": {
        "TargetPosition": {"x": x, "y": y, "theta": theta, "type": nav_type}
    }}}


def build_driving_request(action: int) -> dict:
    """cmd=61 주행 제어 요청. action = DrivingSet.* 상수."""
    return {"Request": {"Set": {"Driving": {"set": action}}}}


def build_get_request(key: str) -> dict:
    """dtype=GET 조회 요청. key = 'AllMovingInfo', 'RobotStatus' 등."""
    return {"Request": {"Get": {key: {}}}}


def build_manual_vw_request(ms: float, rads: float) -> dict:
    """cmd=59 수동 조종 속도 요청. mS=linear(m/s), radS=angular(rad/s)."""
    return {"Request": {"Set": {"MotorManual_VW": {"mS": ms, "radS": rads}}}}


def build_mapping_request(action: int) -> dict:
    """cmd=62 맵 작성 제어 요청. action = MappingSet.* 상수."""
    return {"Request": {"Set": {"Mapping": {"set": action}}}}


def build_save_map_request() -> dict:
    """cmd=87 맵 저장 요청."""
    return {"Request": {"Set": {"SaveMap": {}}}}


def build_software_reset_request() -> dict:
    """cmd=56 소프트웨어 리셋 요청."""
    return {"Request": {"Set": {"SoftwareReset": {}}}}


def build_start_charging_request() -> dict:
    """cmd=54 충전 시작 요청."""
    return {"Request": {"Set": {"StartCharging": {}}}}


def build_stop_charging_request() -> dict:
    """cmd=55 충전 중지 요청."""
    return {"Request": {"Set": {"StopCharging": {}}}}


def build_return_charging_station_request() -> dict:
    """cmd=50 충전 스테이션 복귀."""
    return {"Request": {"Set": {"ReturnToChargingStation": {}}}}


def build_recovery_return_charging_station_request() -> dict:
    """cmd=88 리커버리 후 충전 스테이션 복귀."""
    return {"Request": {"Set": {"RecoveryReturnToChargingStation": {}}}}


def build_station_repositioning_request(x: float = 0.0, y: float = 0.0,
                      theta: float = 0.0,) -> dict:
    """cmd=77 충전 스테이션 위치 재설정."""
    return {"Request": {"Set": {
        "StationRepositioning": {"x": x, "y": y, "theta": theta}
    }}}


def build_rotation_request(rot_type: int, radian: float) -> dict:
    """cmd=66 회전 명령 요청."""
    return {"Request": {"Set": {"Rotation": {"type": rot_type, "radian": radian}}}}

# Codex 작성 부분, 만약 AMR에서 Bypass가 이상하다는 로그를 보게되면, 예제 값 그대로(image관련) 사용하게 재설정
def build_bypass_request(
    block_areas: list[dict] | None = None,
    block_walls: list[dict] | None = None,
    charging_station: dict | None = None,
) -> dict:
    """
    cmd=72 ByPass 전체 설정 요청.

    image_path / image_position 은 robot 좌표와 동일값으로 채운다.
    """
    import datetime
    import uuid

    def _copy_points(robot_path: list[dict]) -> list[dict]:
        return [{"x": p["x"], "y": p["y"]} for p in robot_path]

    areas: list[dict] = []
    for area in block_areas or []:
        robot_path = area.get("robot_path", [])
        areas.append({
            "id": area.get("id", str(uuid.uuid4())),
            "robot_path": robot_path,
            "image_path": _copy_points(robot_path),
        })

    walls: list[dict] = []
    for wall in block_walls or []:
        robot_path = wall.get("robot_path", [])
        walls.append({
            "id": wall.get("id", str(uuid.uuid4())),
            "robot_path": robot_path,
            "image_path": _copy_points(robot_path),
        })

    stations: list[dict] = []
    if charging_station:
        robot_position = charging_station.get("robot_position", {"x": 0.0, "y": 0.0})
        stations.append({
            "id": charging_station.get("id", str(uuid.uuid4())),
            "robot_position": robot_position,
            "image_position": {"x": robot_position["x"], "y": robot_position["y"]},
        })

    return {
        "Request": {
            "Set": {
                "ByPass": {
                    "uid": str(uuid.uuid4())[:10],
                    "info": {
                        "version": "2.0.0",
                        "modified": datetime.datetime.utcnow().strftime("%Y-%m-%dT%H:%M:%S.000Z"),
                    },
                    "room_list": [],
                    "block_area": areas,
                    "block_wall": walls,
                    "charging_station": stations,
                    "init_position": [],
                }
            }
        }
    }
