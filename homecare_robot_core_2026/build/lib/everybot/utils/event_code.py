"""
EventCode / RobotStatusCode / NetStat 상수 정의.
"""
from __future__ import annotations


class EventCode:
    '''
    [KB라이프향 기능 정의서 기반으로 기능(Event) 정의]
    \n PATROL :  순찰상황감지 - 어르신 배회 및 낙상 등 응급상황을 감지하고, 시설 대응이 가능하도록 알림 제공
    \n MorningCall :  모닝콜(아침인사) - 세대별 기상시간에 맞춘 알림 및 안부 확인
    \n MusicPlay :  복도 음악 재생 - 공간 별 배경 음악 송출
    \n Info :  안내기능 - 1층 공간 위치별 안내멘트
    \n Hello :  안부인사 - 안면인식 시반 개인 맞춤 인사
    \n Concierge :  컨시어지 - 음성 기반 직원호출 및 요청 전달
    \n Photo :  사진촬영 - 사진촬영 및 자동 저장 서비스
    \n Contents :  컨텐츠 제공 기능 - 다양한 콘텐츠 제공
    ''' 
    NORMAL = "Normal" # 기능 수행 전의 대기상태
    STOP = "Stop" # 정지 상태 (TBD)
    EMERGENCY = "Emergency" #긴급정지
    PATROL = "PATROL" # 순찰 상황 감지
    MORNING_CALL = "MorningCall" # 아침인사
    MUSIC_PLAY = "MusicPlay" # 복도음악재생
    INFO = "Info" # 안내기능
    HELLO = "Hello" #안부인사
    CONCIERGE = "Concierge" # 음성 기반 직원호출 및 요청 전달
    PHOTO = "Photo" # 사진촬영 및 자동 저장 서비스
    CONTENTS = "Contents" # 다양한 콘텐츠 제공
    DEBUGMODE = "DebugMode" # 디버깅 모드


class RobotStatusCode:
    IDLE = "IDLE"
    BUSY = "BUSY"
    WARN = "WARN"
    ERROR = "ERROR"
    CHARGING = "CHARGING"


class NetStat:
    SOFTAP = 0
    STATION = 1


# ToDo
# 26.4.27/ 시나리오 거의 확정. 기능 정의서 기반 BT 시나리오 작성 필요
SCENARIO_EVENT_MAP: dict[str, str] = {
    # Scheduled scenarios
    "emergency": EventCode.EMERGENCY,
    "patrol_situation_check": EventCode.PATROL,
    "morning_call": EventCode.MORNING_CALL,
    "music_play": EventCode.MUSIC_PLAY,
    # User/manual scenarios
    "info_guidance": EventCode.INFO,
    "hello_greeting": EventCode.HELLO,
    "concierge": EventCode.CONCIERGE,
    "photo_service": EventCode.PHOTO,
    "content_service": EventCode.CONTENTS,
    # Internal/hidden scenarios
    "direct_move": EventCode.NORMAL,
    "move_waypoints": EventCode.NORMAL,
}


LEGACY_SCENARIO_MAP: dict[str, str] = {
    "visit_guidance": "info_guidance",
    "facility_guidance": "info_guidance",
    "care_service": "hello_greeting",
    "patrol": "patrol_situation_check",
    "emergency": "patrol_situation_check",
    "emergency_patrol": "patrol_situation_check",
}


def resolve_scenario_id(raw_id: str | None) -> str:
    """Convert legacy scenario_id values to the current v2 scenario_id."""
    scenario_id = str(raw_id or "").strip()
    return LEGACY_SCENARIO_MAP.get(scenario_id, scenario_id)


def derive_status(bb: "RobotBlackboard") -> str:
    from ..bt.blackboard import RobotBlackboard

    if bb.emergency_stop:
        return RobotStatusCode.ERROR
    if bb.amr_robot_state == 7:
        return RobotStatusCode.CHARGING
    if bb.battery_percent <= 15.0:
        return RobotStatusCode.WARN
    if bb.active_scenario is not None:
        return RobotStatusCode.BUSY
    return RobotStatusCode.IDLE
