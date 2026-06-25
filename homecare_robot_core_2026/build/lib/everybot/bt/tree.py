"""
bt/tree.py — 로봇 BT 전체 트리 조립.

build_robot_tree() 단일 진입점.
모든 시나리오와 인터럽트 핸들러가 이 함수 안에 인라인으로 조립되므로
전체 구조를 한눈에 파악할 수 있다.

트리 개요 (v2):
  root [Selector, memory=False]          ← 매 tick 우선순위 재평가
    ├── factory_reset [Sequence, memory=True]  P=110  공장초기화 (최우선)
    ├── emergency     [Sequence, memory=True]  P=100  UI 긴급정지
    ├── init          [Sequence, memory=True]  P=90   부팅 초기화 (1회)
    │     ├── wifi_sel   [Selector, memory=False]  WiFi 등록 확인
    │     └── map_sel    [Selector, memory=False]  맵 확인 + 없으면 생성
    ├── battery_return[Sequence, memory=True]  P=80   배터리 부족 복귀 + 충전 대기
    ├── scheduled_runner [Selector, memory=False] P=60 스케줄 기반 자동 시나리오
    │     ├── emergency
    │     ├── morning_call
    │     └── music_play
    └── scenario_runner [Selector, memory=False] P=50 사용자 입력 기반 수동 시나리오
          ├── welcome_mode      [Sequence, memory=True]  ← Idle 상태 → 환영 TTS 1회
          ├── direct_move       [Sequence, memory=True]  ← UI cmd_move → BT 이동
          ├── move_waypoints    [Sequence, memory=True]  ← 히든: 단일/복수 waypoint 이동
          ├── info_guidance     [Sequence, memory=True]
          ├── hello_greeting    [Sequence, memory=True]  ← 음성 대화 포함
          ├── concierge         [Sequence, memory=True]
          ├── photo_service     [Sequence, memory=True]
          └── content_service   [Sequence, memory=True]
"""
from __future__ import annotations

import datetime
import logging

import py_trees
import py_trees.composites

from .blackboard import RobotBlackboard
from .bridge import ServiceBundle
from ..utils.event_code import EventCode, SCENARIO_EVENT_MAP
from .nodes import (
    # ── Condition ─────────────────────────────────────
    ConditionNotInitialized,
    ConditionIdleStatus,
    ConditionEmergencyStop,
    ConditionMoveStopRequested,
    ConditionBatteryCritical,
    ConditionScenarioActive,
    ConditionPersonLyingDown,
    ConditionWifiRegistered,
    ConditionMapReady,
    ConditionFactoryReset,
    ConditionForceSoftAP,
    ConditionMapEdit,
    ConditionFallDetected,
    ConditionFallCandidate,
    ConditionFallDetectedDummy,
    ConditionWanderDetected,
    ConditionWanderDetectedDummy,
    ConditionVoiceIntentOk,
    ConditionMoreRoomsToVisit,
    
    
    # ── Action ────────────────────────────────────────
    ActionMarkInitialized,
    ActionIdleStateSet,
    ActionWifiProvision,
    ActionForceSoftAP,
    ActionIdleWaiting,
    ActionSleep,
    ActionRotation,
    ActionNavigateTo,
    ActionNavigateDirectMove,
    ActionReturntToStation,
    ActionSpeak,
    ActionSpeakFile,

    ActionFaceRecognize,
    ActionPhotoCapture,
    ActionNotifyWired,
    ActionAmrStop,
    ActionHandleMoveStop,
    ActionScenarioDone,
    ActionWaitEmergencyRelease,
    ActionWaitBatteryReady,
    ActionFactoryReset,
    ActionStartMapCreationServer,
    ActionWaitMapCreationDone,
    ActionStopMapCreationServer,
    ActionMapEdit,

    ActionNavigateWaypoints,
    ActionLoadPatrolConfig,
    ActionSavePatrolState,
    ActionRestorePatrolState,
    ActionClearFallDetection,
    ActionClearWanderDetection,
    ActionApproachPerson,
    ActionVoiceStatusCheck,
    ActionTakePhoto,
    ActionDetermineLocation,
    ActionNotifyManager,
    ActionPublishDetection,
    ActionLoadMorningCallSchedule,
    ActionNavigateToRoom,
    ActionRingBell,
    ActionWaitDoorOpen,
    ActionWaitDoorOpenDummy,
    ActionGreetResident,
    ActionFreeConversation,
    ActionAnnounceSchedule,
    ActionPublishMorningCallSchedule,
    ActionPublishMorningCallEvent,
    ActionPublishDoorStatus,
    ActionAdvanceVisitIndex,
    ActionRFBellNotify,
    ActionPublishPatrolEvent,
    ActionWaitSpeakerIdle,
    ActionSetMicMute,
    ActionPlayBGM,
    ActionPlayTTS,
    ActionStopAudio,
)

# py_trees composites 단축 alias (타이핑 편의)
_Seq  = py_trees.composites.Sequence
_Sel  = py_trees.composites.Selector
_Par  = py_trees.composites.Parallel
log = logging.getLogger(__name__)


def build_robot_tree(
    bb: RobotBlackboard,
    bundle: ServiceBundle,
    *,
    map_pgm_path:     str = "configs/map/map.pgm",
    map_yaml_path:    str = "configs/map/map.yaml",
    waypoints_path:   str = "configs/waypoints.json",
    forbidden_zones_path: str = "configs/map/forbidden_zones.json",
    roi_zones_path:   str = "configs/map/roi_zones.json",
    map_creation_port: int = 8080,
    tts_root:         str = "assets/tts/",
    bgm_root:         str = "assets/bgm/",
) -> py_trees.behaviour.Behaviour:
    """
    전체 BT 트리를 조립하여 root 노드를 반환한다.
    반환값을 RobotBTDebugger(root) 에 전달한다.

    Parameters
    ----------
    bb                : 모든 노드가 공유하는 Blackboard
    bundle            : AMR / AI / Wired 서비스 + waypoint 맵
    map_pgm_path      : PGM 파일 경로 (MapCreationServer / ActionFactoryReset)
    map_yaml_path     : YAML 파일 경로 (ActionFactoryReset)
    waypoints_path    : waypoints JSON 파일 경로 (ActionFactoryReset)
    map_creation_port : 맵 생성 웹서버 포트 (ActionStartMapCreationServer)
    """

    # ── 공장초기화 ────────────────────────────────────────────────────
    # memory=True: FactoryReset 1회 실행 후 bb.factory_reset=False 로 복귀
    # 이후 ConditionFactoryReset → FAILURE → 이 Sequence는 영구 FAILURE
    factory_reset_seq = _Seq("factory_reset", memory=True)
    factory_reset_seq.add_children([
        ConditionFactoryReset(bb),
        ActionFactoryReset(
            bb, bundle,
            map_pgm_path=map_pgm_path,
            map_yaml_path=map_yaml_path,
            waypoints_path=waypoints_path,
        ),
    ])

    # ── 긴급정지 ────────────────────────────────────────────────────
    # memory=True: 긴급정지 감지 후 해제까지 Sequence 내에서 대기
    # ConditionEmergencyStop → 해제 시 FAILURE → Sequence 재진입 불가 (SUCCESS 직후)
    emergency = _Seq("emergency", memory=True)
    emergency.add_children([
        ConditionEmergencyStop(bb),
        ActionAmrStop(bundle),
        ActionScenarioDone(bb, bundle),         # 시나리오 진행 중이면 상태 정리
        ActionNotifyWired(
            "notify_emergency_stop",
            {},
            bundle,
        ),
        ActionWaitEmergencyRelease(bb),         # 해제까지 RUNNING
        ActionIdleWaiting(bb, bundle, one_shot=True),  # 해제 후 메뉴 1회 표시
    ])

    # ── 이동 정지 ──────────────────────────────────────────────────
    # robot/cmd/stop은 데모 UI에서 이동 취소 용도로 사용한다.
    # emergency_stop과 분리하여 BT가 해제 대기에 빠지지 않게 한다.
    move_stop = _Seq("move_stop", memory=True)
    move_stop.add_children([
        ConditionMoveStopRequested(bb),
        ActionHandleMoveStop(bb, bundle),
    ])

    # ── 배터리 부족 복귀 ──────────────────────────────────────────────
    # memory=True: 배터리 부족 감지 → dock 이동 → 충전 완료까지 중단 없이 진행
    battery_return = _Seq("battery_return", memory=True)
    battery_return.add_children([
        ConditionBatteryCritical("[C] Battery Critical?", bundle),
        ActionAmrStop(bundle),
        ActionNotifyWired("notify_battery_low", {}, bundle),
        ActionSpeak("배터리가 부족합니다. 충전 위치로 이동합니다.", bb, bundle),
        ActionNavigateTo("dock", bb, bundle, timeout=300.0),
        ActionNotifyWired("notify_charging", {}, bundle),
        ActionWaitBatteryReady(bb, bundle),     # 충전 완료까지 RUNNING
        ActionNotifyWired("notify_charging_done", {}, bundle),  # 충전 완료 OrangePi 알림
        ActionIdleWaiting(bb, bundle, one_shot=True),  # 충전 완료 후 메뉴 1회 표시
    ])

    # ── 초기화 (부팅 후 1회) ─────────────────────────────────────────
    # memory=True: 맵 확인/생성 → WiFi 등록 순서
    #
    # 순서 근거:
    #   부팅 시 SoftAP가 켜진 상태(192.168.0.1)에서 맵 생성 웹UI(8080) 접근 가능.
    #   맵 생성 완료 후 WiFi 프로비저닝 진행 → Home WiFi 연결 → SoftAP 해제.
    #   WiFi 먼저 연결하면 SoftAP가 꺼져 맵 생성 중 OrangePi 통신 불가.
    init = _Seq("init", memory=True)
    init.add_children([
        ConditionNotInitialized(bb),
        _build_map_check(                    # ① 맵 확인 + 없으면 생성 (SoftAP ON 상태)
            bb, bundle,
            map_pgm_path=map_pgm_path,
            waypoints_path=waypoints_path,
            forbidden_zones_path=forbidden_zones_path,
            roi_zones_path=roi_zones_path,
            map_creation_port=map_creation_port,
        ),
        _build_wifi_check(bb, bundle),       # ② 맵 완료 후 WiFi 등록 확인
        ActionMarkInitialized(bb),
        ActionIdleStateSet(bb,bundle,change_state=True),

    ])

    scheduled_runner = _build_scheduled_runner(bb, bundle, tts_root=tts_root, bgm_root=bgm_root)
    scenario_runner = _build_scenario_runner(bb, bundle, tts_root=tts_root, bgm_root=bgm_root)

    # ── Root ────────────────────────────────────────────────────────────────
    # memory=False: 매 tick 처음부터 재평가 → 인터럽트 선점 보장
    root = _Sel("root", memory=False)
    root.add_children([
        factory_reset_seq,                    # P=110
        emergency,                            # P=100
        move_stop,                            # P=95   이동 정지(비-emergency)
        init,                                 # P=90
        battery_return,                       # P=80
        _build_force_softap(bb, bundle),      # P=75  디버깅용 강제 SoftAP
        _build_map_edit(                      # P=70  운영 점검용 맵 편집 모드
            bb, bundle,
            map_pgm_path=map_pgm_path,
            waypoints_path=waypoints_path,
            forbidden_zones_path=forbidden_zones_path,
            roi_zones_path=roi_zones_path,
            map_creation_port=map_creation_port,
        ),
        #scheduled_runner,                     # P=60  스케줄 기반 자동 #데모에는 동작 안함
        scenario_runner,                      # P=50
    ])

    return root


def _build_scheduled_runner(
    bb: RobotBlackboard,
    bundle: ServiceBundle,
    *,
    tts_root: str,
    bgm_root: str,
) -> py_trees.composites.Selector:
    """
    스케줄 기반 자동 시나리오 실행기.

    Phase 8에서는 신규 분기 구조와 scenario_id 정렬을 먼저 적용한다.
    실제 시간/센서 Guard는 Phase 9에서 bb.schedule_table 기반으로 확장한다.
    """
    runner = _Sel("scheduled_runner", memory=False)
    runner.add_children([
        _build_scheduled_sequence(
            bb,
            "patrol_situation_check",
            _build_patrol_situation_check(bb, bundle),
        ),
        _build_scheduled_sequence(
            bb,
            "morning_call",
            _build_morning_call_sequence(bb, bundle, bgm_root=bgm_root),
        ),
    ])
    return runner


def _build_scenario_runner(
    bb: RobotBlackboard,
    bundle: ServiceBundle,
    *,
    tts_root: str,
    bgm_root: str,
) -> py_trees.composites.Selector:
    """사용자 입력 기반 수동 시나리오 실행기."""
    runner = _Sel("scenario_runner", memory=False)
    runner.add_children([
        _build_direct_move(bb, bundle, bgm_root=bgm_root),
        _build_welcome(bb, bundle, tts_root=tts_root),
        _build_rotation(bb,bundle), # Debugging
        _build_move_waypoints(bb, bundle), # Debugging

        _build_morning_call_sequence(bb, bundle, bgm_root=bgm_root), ##데모용 임시 수동 진입 선택
        _build_patrol_situation_check(bb, bundle), ##데모용 임시 수동 진입 선택

        ########## 아직 미적용 ###########
        #_build_info_guidance_sequence(bb, bundle, tts_root=tts_root),
        #_build_concierge_sequence(bb, bundle),
        #_build_photo_service(bb, bundle),
        #_build_content_service_sequence(bb, bundle),
    ])
    return runner


def _build_direct_move(
    bb: RobotBlackboard,
    bundle: ServiceBundle,
    bgm_root: str = "assets/bgm/",
) -> py_trees.composites.Sequence:
    """UI cmd_move를 BT 중심 direct_move 시나리오로 실행한다."""
    
    bgm_file = bgm_root + "moving.wav"
    
    seq = _Seq("direct_move", memory=True)
    seq.add_children([
        ConditionScenarioActive("direct_move", bb),
        ActionSpeak(" 이동을 시작합니다.", bb, bundle),
        ActionPlayBGM(bgm_file, bundle),
        ActionNavigateDirectMove(bb, bundle, timeout=300.0),
        ActionStopAudio(bundle, stop_type="bgm"),
        
    ])
    return seq


def _build_scheduled_sequence(
    bb: RobotBlackboard,
    scenario_id: str,
    child: py_trees.behaviour.Behaviour,
) -> py_trees.composites.Sequence:
    seq = _Seq(f"scheduled_{scenario_id}", memory=True)
    seq.add_children([
        ConditionScheduleDue(bb, scenario_id),
        child,
    ])
    return seq


class ConditionScheduleDue(py_trees.behaviour.Behaviour):
    """schedule_table/센서 조건이 만족되면 자동 시나리오를 활성화한다."""

    def __init__(self, bb: RobotBlackboard, scenario_id: str) -> None:
        super().__init__(name=f"[C] ScheduleDue({scenario_id})?")
        self._bb = bb
        self._scenario_id = scenario_id

    def update(self) -> py_trees.common.Status:
        if self._bb.active_scenario == self._scenario_id:
            return py_trees.common.Status.SUCCESS
        if self._bb.active_scenario is not None:
            return py_trees.common.Status.FAILURE

        entry = self._find_enabled_entry()
        if entry is None:
            return py_trees.common.Status.FAILURE

        now = datetime.datetime.now()
        current_day = now.strftime("%a").lower()[:3]
        if current_day not in entry.days:
            return py_trees.common.Status.FAILURE

        if entry.operating_start and entry.operating_end:
            if not self._in_operating_hours(now.strftime("%H:%M"), entry.operating_start, entry.operating_end):
                return py_trees.common.Status.FAILURE
            trigger_key = self._trigger_key(f"{entry.operating_start}-{entry.operating_end}")
        else:
            current_hhmm = now.strftime("%H:%M")
            if not entry.trigger_time or entry.trigger_time != current_hhmm:
                return py_trees.common.Status.FAILURE
            trigger_key = self._trigger_key(current_hhmm)

        if trigger_key in self._bb.schedule_trigger_history:
            return py_trees.common.Status.FAILURE

        self._bb.schedule_trigger_history.add(trigger_key)
        self._bb.active_scenario = self._scenario_id
        self._bb.current_event_code = SCENARIO_EVENT_MAP.get(
            self._scenario_id, EventCode.NORMAL
        )
        self._bb.scenario_params = self._build_params(entry)
        self._bb.clear_ctx()
        log.info(
            "[Schedule] activated scenario=%s event=%s key=%s params=%s",
            self._scenario_id,
            self._bb.current_event_code,
            trigger_key,
            self._bb.scenario_params,
        )
        return py_trees.common.Status.SUCCESS

    def _find_enabled_entry(self):
        for entry in self._bb.schedule_table:
            if entry.scenario_id == self._scenario_id and entry.enabled:
                return entry
        return None

    def _is_emergency_detected(self) -> bool:
        if self._bb.emergency_sensor_triggered:
            return True
        return (
            self._bb.has_ai_event("emergency_detected")
            or self._bb.has_ai_event("person_lying_down")
            or self._bb.has_ai_event("fall_detected")
        )

    def _in_operating_hours(self, current: str, start: str, end: str) -> bool:
        if start <= end:
            return start <= current <= end
        return current >= start or current <= end

    def _trigger_key(self, trigger: str) -> str:
        today = datetime.datetime.now().strftime("%Y-%m-%d")
        return f"{today}:{trigger}:{self._scenario_id}"

    def _build_params(self, entry) -> dict:
        params: dict = {}
        if entry.zone_id:
            params["zone_id"] = entry.zone_id
        return params


class ConditionCallType(py_trees.behaviour.Behaviour):
    """컨시어지 호출 유형이 기대값과 일치하면 SUCCESS."""

    def __init__(self, bb: RobotBlackboard, call_type: str) -> None:
        super().__init__(name=f"[C] CallType({call_type})?")
        self._bb = bb
        self._call_type = call_type

    def update(self) -> py_trees.common.Status:
        current = str(self._bb.scenario_params.get("call_type", "general") or "general")
        return (
            py_trees.common.Status.SUCCESS
            if current == self._call_type
            else py_trees.common.Status.FAILURE
        )


# =============================================================================
# Init 서브트리 빌더
# =============================================================================

def _build_wifi_check(
    bb: RobotBlackboard, bundle: ServiceBundle
) -> py_trees.composites.Selector:
    """
    WiFi 등록 확인 서브트리.

    Selector (memory=False):
      ├── ConditionWifiRegistered  ← 이미 등록 → SUCCESS (통과)
      └── wifi_wait [Seq, memory=True]  ← 미등록
            ├── ActionNotifyWired("notify_wifi_setup_required")  ← 1회 알림
            └── ActionWifiProvision(bb)  ← 등록 완료까지 RUNNING

    Bridge가 wifi_reg_fn()으로 bb.wifi_registered를 매 tick 동기화.
    등록 완료 시 다음 tick에서 ConditionWifiRegistered → SUCCESS 통과.
    """
    wifi_wait = _Seq("wifi_wait", memory=True)
    wifi_wait.add_children([
        ActionNotifyWired(
            "notify_wifi_setup_required",
            {},
            bundle,
        ),
        ActionWifiProvision(bb),   # 등록 완료까지 RUNNING
    ])

    wifi_sel = _Sel("wifi_check", memory=False)
    wifi_sel.add_children([
        ConditionWifiRegistered(bb),
        wifi_wait,
    ])
    return wifi_sel


def _build_force_softap(
    bb: RobotBlackboard, bundle: ServiceBundle
) -> py_trees.composites.Sequence:
    """
    Force SoftAP 서브트리 (root P=75).

    Sequence (memory=True):
      ├── ConditionForceSoftAP  ← bb.force_softap=True 일 때만 진입
      └── ActionForceSoftAP     ← SoftAP ON → 해제 명령 수신 시 SUCCESS

    Force SoftAP 활성 중 시나리오 실행 중단.
    해제 시 ActionForceSoftAP.terminate()에서 home WiFi 자동 복귀.
    """
    seq = _Seq("force_softap", memory=True)
    seq.add_children([
        ConditionForceSoftAP(bb),
        ActionForceSoftAP(bb, bundle),
    ])
    return seq


def _build_map_edit(
    bb: RobotBlackboard, bundle: ServiceBundle,
    *,
    map_pgm_path: str,
    waypoints_path: str,
    forbidden_zones_path: str,
    roi_zones_path: str,
    map_creation_port: int,
) -> py_trees.composites.Sequence:
    """
    맵 편집 서브트리 (root P=70). cmd_map_edit enabled=true 수신 시 진입.

    기존 맵/waypoints/금지영역/ROI를 불러와 편집(추가·수정·삭제)하는 운영 점검 모드.
    활성 중 시나리오 실행 중단. cmd_map_edit enabled=false 수신 시 서버 종료.

    Sequence (memory=True):
      ├── ConditionMapEdit  ← bb.map_edit=True 일 때만 진입
      └── ActionMapEdit     ← 맵서버 기동 → 비활성화 명령 대기
    """
    seq = _Seq("map_edit", memory=True)
    seq.add_children([
        ConditionMapEdit(bb),
        ActionMapEdit(
            bb,
            bundle,
            port=map_creation_port,
            map_pgm_path=map_pgm_path,
            waypoints_path=waypoints_path,
            forbidden_zones_path=forbidden_zones_path,
            roi_zones_path=roi_zones_path,
        ),
    ])
    return seq


def _build_map_check(
    bb: RobotBlackboard, bundle: ServiceBundle,
    *,
    map_pgm_path: str,
    waypoints_path: str,
    forbidden_zones_path: str,
    roi_zones_path: str,
    map_creation_port: int,
) -> py_trees.composites.Selector:
    """
    맵 확인 서브트리. 맵이 없으면 웹서버 기반 맵 생성 서브시나리오 진행.

    Selector (memory=False):
      ├── ConditionMapReady  ← 맵 존재 → SUCCESS (통과)
      └── map_creation [Seq, memory=True]  ← 맵 없음
            ├── ActionStartMapCreationServer(port)  ← 웹서버 기동
            ├── ActionWaitMapCreationDone           ← Phase 5 ROI 저장까지 대기
            └── ActionStopMapCreationServer         ← 웹서버 종료
    """
    map_creation = _Seq("map_creation", memory=True)
    map_creation.add_children([
        ActionStartMapCreationServer(
            bb,
            bundle,
            port=map_creation_port,
            map_pgm_path=map_pgm_path,
            waypoints_path=waypoints_path,
            forbidden_zones_path=forbidden_zones_path,
            roi_zones_path=roi_zones_path,
        ),
        ActionWaitMapCreationDone(bb),
        ActionStopMapCreationServer(bb, bundle),
    ])

    map_sel = _Sel("map_check", memory=False)
    map_sel.add_children([
        ConditionMapReady(bb),
        map_creation,
    ])
    return map_sel


# =============================================================================
# 시나리오 서브트리 빌더
# 각 시나리오는 독립된 함수로 정의하되,
# 모두 build_robot_tree() 안에서 호출되어 전체 구조는 위에서 파악 가능
# =============================================================================
def _build_scenario_done(    
                         bb: RobotBlackboard, bundle: ServiceBundle
) -> py_trees.composites.Sequence:
    seq = _Seq("scenario_done", memory=True)
    seq.add_children([
        # 임시 마이크 음소거
        ActionSetMicMute(bundle, True, 1.0),
        # 
        
        ActionScenarioDone(bb, bundle),
        ActionIdleStateSet(bb,bundle,change_state=True),
        ActionSleep(bb,bundle,sec=5.0),
    ])
    return seq


def _build_welcome(
    bb: RobotBlackboard, bundle: ServiceBundle, tts_root: str = "assets/tts/"
) -> py_trees.composites.Sequence:
    """
    시나리오 선택 화면 전환 및 환영
    """
    seq = _Seq("welcome_mode", memory=True)
    seq.add_children([
        ConditionIdleStatus(bb),
        # 임시 마이크 음소거
        ActionSetMicMute(bundle, True, 1.0),
        # 
        ActionIdleWaiting(bb, bundle, one_shot=True),
        #ActionSpeakFile(tts_root + "1.hello.wav", bundle),
        ActionIdleStateSet(bb,bundle,change_state=False),
    ])
    return seq


def _build_move_waypoints(
    bb: RobotBlackboard, bundle: ServiceBundle
) -> py_trees.composites.Sequence:
    """
    히든 시나리오 : waypoint 이동

    """
    seq = _Seq("move_waypoints", memory=True)
    seq.add_children([
        ConditionScenarioActive("move_waypoints", bb),
        ActionNavigateWaypoints(bb, bundle, timeout_per_wp=300.0),
        ActionRFBellNotify(bb,bundle, destination_ctx_key="bell_point"),
        _build_scenario_done(bb, bundle),
    ])
    return seq


def _build_rotation(
    bb: RobotBlackboard, bundle: ServiceBundle
) -> py_trees.composites.Sequence:
    """
    히든 시나리오 : 로봇 회전

    """
    seq = _Seq("rotation", memory=True)
    seq.add_children([
        ConditionScenarioActive("rotation", bb),
        ActionRotation(bb, bundle, "{rotation_type}", "{radian}"),
        ActionScenarioDone(bb, bundle),
    ])
    return seq


def _build_info_guidance_sequence(
    bb: RobotBlackboard, bundle: ServiceBundle, tts_root: str = "assets/tts/"
) -> py_trees.composites.Sequence:
    """
    안내기능 시나리오.

    레거시 visit_guidance/facility_guidance는 bridge에서 info_guidance로 정규화된다.
    """
    seq = _Seq("info_guidance", memory=True)
    seq.add_children([
        ConditionScenarioActive("info_guidance", bb),
        ActionSpeakFile(tts_root + "2.guide_start.wav", bundle),
        ActionNavigateTo("{target_pos}", bb, bundle, timeout=120.0),
        ActionSpeakFile(tts_root + "2.guide_demo.wav", bundle),
        _build_scenario_done(bb, bundle),
    ])
    return seq


# =============================================================================
# 야간 순찰 (긴급상황, 이상상황) Tree
# =============================================================================

def _build_patrol_situation_check(
    bb: RobotBlackboard, bundle: ServiceBundle,
    bgm_root: str = "assets/bgm/",
) -> py_trees.composites.Sequence:
    """긴급상황감지 시나리오: 순찰 이동 + 낙상/배회 병렬 감지."""
    bgm_file = bgm_root + "patrol.wav"
    patrol_cfg = bundle.feature_cfg.get("patrol", {}) if isinstance(bundle.feature_cfg, dict) else {}
    fall_detection_enabled = bool(patrol_cfg.get("fall_detection_enabled", True))
    fall_candidate_enabled = bool(patrol_cfg.get("fall_candidate_enabled", True))
    fall_dummy_enabled = bool(patrol_cfg.get("fall_dummy_enabled", False))
    fall_detected_first = bool(patrol_cfg.get("fall_detected_first", False))
    wander_detection_enabled = bool(patrol_cfg.get("wander_detection_enabled", True))
    wander_dummy_enabled = bool(patrol_cfg.get("wander_dummy_enabled", False))

    patrol_nav = ActionNavigateWaypoints(
        bb,
        bundle,
        timeout_per_wp=300.0,
        dwell_sec=float(bb.patrol_dwell_sec),
    )
    patrol_parallel = _Par(
        "patrol_parallel",
        policy=py_trees.common.ParallelPolicy.SuccessOnSelected(
            children=[patrol_nav],
            synchronise=False,
        ),
    )


    ########상황감지 -> 낙상감지 하위트리
    fall_watch = _Sel("fall_watch", memory=False)
    fall_monitor = _Seq("fall_monitor", memory=True)
    fall_trigger = _Sel("fall_trigger", memory=False)
    fall_trigger_children = []
    if fall_detection_enabled:
        if fall_detected_first:
            fall_trigger_children.append(ConditionFallDetected(bb, bundle))
            if fall_candidate_enabled:
                fall_trigger_children.append(ConditionFallCandidate(bb, bundle))
        else:
            if fall_candidate_enabled:
                fall_trigger_children.append(ConditionFallCandidate(bb, bundle))
            fall_trigger_children.append(ConditionFallDetected(bb, bundle))
    if fall_dummy_enabled:
        fall_trigger_children.append(ConditionFallDetectedDummy(bb))
    fall_trigger.add_children(fall_trigger_children)
    fall_response = _Sel("fall_response", memory=False)
    fall_ok = _Seq("fall_ok", memory=True)

    fall_ok.add_children([
        ConditionVoiceIntentOk(bb),
    ])
    fall_help = _Seq("fall_help", memory=True)
    fall_help.add_children([
        #ActionTakePhoto(bb, bundle), # 아직 구현 및 연동 안됨 (6월 예정)
        #ActionNotifyManager(bb, bundle), # 아직 구현 및 연동 안됨 (6월 예정)
    ])
    fall_response.add_children([fall_ok, fall_help])
    fall_monitor.add_children([
        fall_trigger,
        
        # BGM 재생 중단
        ActionStopAudio(bundle, stop_type="bgm"),
        
        ActionSavePatrolState(bb, bundle),
        ActionPublishDetection(bb, bundle, "fall"), # 낙상 감지 화면 전환
        ActionAmrStop(bundle),
        ActionSpeak(" 위험 상황이 감지 되었습니다!", bb, bundle),
        
        #ActionSleep(bb, bundle, sec=1.0),
        
        #ActionApproachPerson(bb, bundle), #데모에서는 사람한테 안감
        ActionVoiceStatusCheck(bb, bundle),
        #ActionSleep(bb, bundle, sec=1.0),
        
        #fall_response, # API 연동 및 동작이 확실하지 않음
        
        # UI에 Detection 결과 전달
        ActionSpeak(" 응답이 없어 위험 상황으로 감지되어 관리자에게 메시지를 전송했습니다.", bb, bundle),
        ActionWaitSpeakerIdle(bundle, timeout=45.0),
        ActionPublishDetection(bb, bundle, "fall-confirm"), # 낙상 감지 알림 전송 화면 전환
        ##ActionSpeak(" 현재 위치로 호출이 완료되었습니다.", bb, bundle),
        ActionRestorePatrolState(bb, bundle),

        # 감지 상태 초기화
        ActionClearFallDetection(bb, bundle),
        
        #ActionSleep(bb, bundle, sec=60.0),
        ActionSpeak(" 순찰을 계속 진행합니다.", bb, bundle),
        ActionPlayBGM(bgm_file, bundle), # 순찰 시작 BGM

    ])
    fall_watch.add_children([
        fall_monitor,
        ActionSleep(bb, bundle, sec=86400.0),
    ])

    ########상황감지 -> 배회감지 하위트리
    wander_watch = _Sel("wander_watch", memory=False)
    wander_monitor = _Seq("wander_monitor", memory=True)
    wander_trigger = _Sel("wander_trigger", memory=False)
    wander_trigger_children = []
    if wander_detection_enabled:
        wander_trigger_children.append(ConditionWanderDetected(bb, bundle))
    if wander_dummy_enabled:
        wander_trigger_children.append(ConditionWanderDetectedDummy(bb))
    wander_trigger.add_children(wander_trigger_children)
    wander_monitor.add_children([
        wander_trigger,

        # 사진 촬영 -> 아직 검증 안됨
        #ActionTakePhoto(bb, bundle),

        # Location은 이미 보내고 있지만 나중에 할 예정
        #ActionDetermineLocation(bb, bundle),

        # 알림 메세지 전송. but Demo는 없음
        #ActionNotifyManager(bb, bundle),

        # Publish 하면서 일단 UI에서 같이 MMS 보낼 예정
        ActionPublishDetection(bb, bundle, "wander"), # 배회 감지 확인 화면 전환
        ActionSleep(bb, bundle, sec=3.0),
        ActionPublishDetection(bb, bundle, "wander-confirm"), # 배회 감지 알림 전송 화면 전환

        # 감지 상태 초기화
        ActionClearWanderDetection(bb, bundle),
    ])
    wander_watch.add_children([
        wander_monitor,
        ActionSleep(bb, bundle, sec=86400.0),
    ])

    # 순찰 트리 구성
    patrol_children = [patrol_nav] # 목적지 순회
    if fall_trigger_children:
        patrol_children.append(fall_watch) # 낙상 감지
    if wander_trigger_children:
        patrol_children.append(wander_watch) # 배회 감지
    patrol_parallel.add_children(patrol_children)

    ######## 순찰 중 긴급상황 감지 메인 시나리오 #######
    seq = _Seq("patrol_situation_check", memory=True)
    seq.add_children([
        ConditionScenarioActive("patrol_situation_check", bb),
        ActionLoadPatrolConfig(bb, bundle), #순찰 정보 확인
        ActionPublishPatrolEvent(bb,bundle,"start"),
        ActionSpeak(" 야간 순찰을 시작하겠습니다.", bb, bundle),
        
        # 순찰 시작 전 BGM 셋팅. 추후 음량 조절 필요
        ActionPlayBGM(bgm_file, bundle), # 순찰 시작 BGM
        
        patrol_parallel, # 순찰 시작
        
        # 순찰 종료
        ActionSpeak(" 순찰이 종료되었습니다.", bb, bundle),
        ActionNavigateTo("home",bb, bundle), # 대기장소(home) 이동
        ActionPublishPatrolEvent(bb,bundle,"end"),
        _build_scenario_done(bb, bundle),
    ])
    return seq

# =============================================================================
# 모닝 콜(아침 안부인사) Tree
# =============================================================================
def _build_morning_call_sequence(
    bb: RobotBlackboard, bundle: ServiceBundle,
    bgm_root: str = "assets/bgm/",
    tts_root: str = "assets/tts/",
) -> py_trees.composites.Sequence:
    """모닝콜 시나리오: 데모용 2세대 순회 + 문열림/인사/일정 안내."""
    bgm_file = bgm_root + "moving.wav"
    cfg = bundle.feature_cfg.get("morning_call", {}) if isinstance(bundle.feature_cfg, dict) else {}
    bell_duration = float(cfg.get("bell_duration_sec", 20) or 20)
    door_wait = float(cfg.get("door_wait_sec", 300) or 300)
    max_retry = int(cfg.get("max_door_retry", 2) or 2)
    door_open_mode = str(cfg.get("door_open_mode", "face_api") or "face_api").lower()
    door_dummy_delay = float(cfg.get("door_dummy_delay_sec", 3.0) or 3.0)
    
    # Visit 트리 정의
    def _build_visit(name: str) -> py_trees.composites.Sequence:
        visit = _Seq(name, memory=True)
        visit.add_children([
            
            # 기능 활성화 확인
            ConditionMoreRoomsToVisit(bb),
            ActionSetMicMute(bundle, True, 1.0),

            # 이동 시작
            ActionPublishMorningCallEvent(bb, bundle, "moving"), # UI Mqtt 발송 : 이동 시작
            ActionSpeak(" 어르신 안부 인사를 드리러 출발합니다.", bb,bundle),
            ActionPlayBGM(bgm_file, bundle),
            ActionNavigateToRoom(bb, bundle, timeout=120.0), # 호실 이동 시작 (feature_settings.json 파일에 있는 리스트 참조)
            ActionStopAudio(bundle, stop_type="bgm"),
            ActionPublishMorningCallEvent(bb, bundle, "arrived"), # UI Mqtt 발송 : 도착 알림
            
            # 도착 후 벨울림
            ActionSleep(bb,bundle, 4.0),
            ActionRingBell(bb, bundle, duration_sec=bell_duration), # 벨 울림 (waypoints.json의 목적지에 저장 된 Bell ID로 호출)
            ActionPublishMorningCallEvent(bb, bundle, "waiting_door"), # UI Mqtt 발송 : 문열림 대기
            
            ActionSpeak(" 안부인사 드리러 왔습니다!", bb,bundle),
            #ActionSpeakFile(tts_root+"8.morning_check.wav",bundle),
            ActionSleep(bb,bundle,5.0),
            
            # 문열림 대기
            (
                ActionWaitDoorOpenDummy(bb, bundle, delay_sec=door_dummy_delay)
                if door_open_mode == "dummy"
                else ActionWaitDoorOpen(bb, bundle, wait_sec=door_wait, max_retry=max_retry)
            ),
            ActionPublishDoorStatus(bb, bundle), # UI Mqtt 발송 : 문 열림 결과 
            ActionPublishMorningCallEvent(bb, bundle, "talking"), # UI Mqtt 발송 : 대화 시작
            
            #ActionSpeakFile(tts_root+"10.condition_check_2.wav",bundle),
            #ActionSleep(bb,bundle,1.0),

            # AI 얼굴인식 기반 인사 대화 (변경 필요?, Voice Agent에서 대화 종료했다라는 결과 알리면 대기하는걸로?)
            ActionGreetResident(bb, bundle), #안면 인식 후 인식한 사람 저장 및 기본 인사멘트 실행
            ActionWaitSpeakerIdle(bundle, timeout=45.0),
            
            # 데모용 미사여구 멘트
            ActionSpeak(" 식사는 꼭 챙겨 드시고, 무리하지 마시고 천천히 하루 보내세요.", bb,bundle),
            ActionWaitSpeakerIdle(bundle, timeout=45.0),
            ActionSpeak("저는 언제든 {recognized_name}님 곁에서 도움을 드릴 준비가 되어 있습니다.", bb,bundle),
            ActionWaitSpeakerIdle(bundle, timeout=45.0),
            
            # Wake Word 호출 대기
            ActionSpeak(" 필요한 정보나 도움이 필요하시면, 제 이름을 불러 편하게 말씀해주세요.", bb,bundle),
            ActionSleep(bb,bundle,2.0),
            ActionWaitSpeakerIdle(bundle, timeout=45.0),
            ActionSetMicMute(bundle, False),

            # AI TTS/STT등 기능 기반 자유 대화
            ActionFreeConversation(bb, bundle),
            ActionSetMicMute(bundle, True),
            ActionSleep(bb,bundle,1.0),
            
            # AI TTS 기능 기반 스케쥴 안내 대화
            #ActionAnnounceSchedule(bb, bundle),
            #ActionSleep(bb,bundle,2.0),
            
            # 다음 장소 이동
            ActionSpeak(" 오늘 함께 이야기 나눌 수 있어 즐거웠습니다. 남은 시간도 편안하고 건강하게 보내세요.", bb,bundle),
            #ActionSleep(bb,bundle,3.0),
            
            ActionSpeak(" 다음 장소로 이동하겠습니다.", bb,bundle,wait_until_done=True),
            #ActionSleep(bb,bundle,3.0),
            
            ActionPublishMorningCallEvent(bb, bundle, "moving_next"), # UI Mqtt 발송 : 다음 콜 장소 이동 
            ActionAdvanceVisitIndex(bb, bundle),
        ])
        return visit
    
    def _build_visit_dummy(name: str) -> py_trees.composites.Sequence:
        visit = _Seq(name, memory=True)
        visit.add_children([
            
            # 기능 활성화 확인
            ConditionMoreRoomsToVisit(bb),
            ActionSetMicMute(bundle, True, 1.0),

            # 이동 시작
            ActionPublishMorningCallEvent(bb, bundle, "moving"), # UI Mqtt 발송 : 이동 시작
            ActionSpeak(" 어르신 안부 인사를 드리러 출발합니다.", bb,bundle),
            ActionPlayBGM(bgm_file, bundle),
            ActionNavigateToRoom(bb, bundle, timeout=120.0), # 호실 이동 시작 (feature_settings.json 파일에 있는 리스트 참조)
            ActionStopAudio(bundle, stop_type="bgm"),
            ActionPublishMorningCallEvent(bb, bundle, "arrived"), # UI Mqtt 발송 : 도착 알림
            
            # 도착 후 벨울림
            ActionSleep(bb,bundle, 4.0),
            ActionRingBell(bb, bundle, duration_sec=bell_duration), # 벨 울림 (waypoints.json의 목적지에 저장 된 Bell ID로 호출)
            ActionPublishMorningCallEvent(bb, bundle, "waiting_door"), # UI Mqtt 발송 : 문열림 대기
            
            ActionSpeak(" 안부인사 드리러 왔습니다!", bb,bundle),
            #ActionSpeakFile(tts_root+"8.morning_check.wav",bundle),
            ActionSleep(bb,bundle,15.0),
            
            ActionSpeak(" 응답이 없어 다음 장소로 이동하겠습니다.", bb,bundle),
            
            ActionPublishMorningCallEvent(bb, bundle, "moving_next"), # UI Mqtt 발송 : 다음 콜 장소 이동 
            ActionAdvanceVisitIndex(bb, bundle),
        ])
        return visit

    ## 모닝콜 기능 메인 트리
    seq = _Seq("morning_call", memory=True)
    seq.add_children([
        # 모닝콜 시작 확인
        ConditionScenarioActive("morning_call", bb),
        # 모닝콜 스케쥴 로드 및 Noti
        ActionLoadMorningCallSchedule(bb, bundle),
        ActionPublishMorningCallSchedule(bb, bundle),
        
        # 방문 시작
        _build_visit("morning_visit_1"),
        _build_visit_dummy("morning_visit_2"),
        
        # 모닝콜 종료 후 복귀
        ActionPublishMorningCallEvent(bb, bundle, "completed"), # UI Mqtt 발송 : 모닝콜 완료

        ActionSetMicMute(bundle, True),
        ActionSpeak(" 이동을 시작합니다.", bb,bundle),
            
        #ActionSpeakFile(tts_root+"2.moving_start.wav",bundle),
        ActionSleep(bb,bundle,3.0),
        
        ActionPlayBGM(bgm_file, bundle),
        ####ActionReturntToStation(bb, bundle), # 충전기 이동(시연에는 안함)
        ActionNavigateTo("home",bb, bundle), # 대기장소(home) 이동
        ActionSleep(bb,bundle,3.0),
        ActionStopAudio(bundle, stop_type="bgm"),
        ActionWaitSpeakerIdle(bundle, timeout=20.0),
        ActionSetMicMute(bundle, False),
                
        # 모닝 콜 기능 종료
        ActionScenarioDone(bb, bundle),
        ActionIdleStateSet(bb, bundle, change_state=True),
    ])
    return seq


def _build_concierge_sequence(
    bb: RobotBlackboard, bundle: ServiceBundle
) -> py_trees.composites.Sequence:
    """컨시어지 시나리오: 일반 호출과 긴급 호출을 분기해 UI에 전달한다."""
    call_branch = _Sel("concierge_call_type_branch", memory=False)
    call_branch.add_children([
        _build_concierge_emergency_call(bb, bundle),
        _build_concierge_general_call(bb, bundle),
    ])

    seq = _Seq("concierge", memory=True)
    seq.add_children([
        ConditionScenarioActive("concierge", bb),
        ActionSpeak("요청을 담당자에게 전달하겠습니다.", bb, bundle),
        call_branch,
        _build_scenario_done(bb, bundle),
    ])
    return seq


def _build_concierge_emergency_call(
    bb: RobotBlackboard, bundle: ServiceBundle
) -> py_trees.composites.Sequence:
    seq = _Seq("concierge_emergency_call", memory=True)
    seq.add_children([
        ConditionCallType(bb, "emergency"),
        ActionNotifyWired(
            "request_show_menu",
            lambda: {"screen": "emergency_call_alert"},
            bundle,
        ),
        ActionNotifyWired(
            "notify_concierge_request",
            lambda: {
                "call_type": "emergency",
                "priority": "high",
                "message": bb.scenario_params.get("message", ""),
                "room_id": bb.scenario_params.get("room_id", ""),
            },
            bundle,
        ),
    ])
    return seq


def _build_concierge_general_call(
    bb: RobotBlackboard, bundle: ServiceBundle
) -> py_trees.composites.Sequence:
    seq = _Seq("concierge_general_call", memory=True)
    seq.add_children([
        ActionNotifyWired(
            "notify_concierge_request",
            lambda: {
                "call_type": bb.scenario_params.get("call_type", "general"),
                "priority": "normal",
                "message": bb.scenario_params.get("message", ""),
                "room_id": bb.scenario_params.get("room_id", ""),
            },
            bundle,
        ),
    ])
    return seq

'''
def _build_content_service_sequence(
    bb: RobotBlackboard, bundle: ServiceBundle
) -> py_trees.composites.Sequence:
    """콘텐츠 제공 시나리오 골격."""
    seq = _Seq("content_service", memory=True)
    seq.add_children([
        ConditionScenarioActive("content_service", bb),
        ActionSpeak("콘텐츠를 준비하겠습니다.", bb, bundle),
        _build_scenario_done(bb, bundle),
    ])
    return seq
'''
'''
def _build_photo_service(
    bb: RobotBlackboard, bundle: ServiceBundle
) -> py_trees.composites.Sequence:
    """
    시나리오 5: 사진 촬영 및 전송
      안내 발화 → 얼굴 인식 → 카운트다운 → 촬영 → 전송 → 완료
    """
    seq = _Seq("photo_service", memory=True)
    seq.add_children([
        ConditionScenarioActive("photo_service", bb),
        ActionSpeak("사진 촬영을 도와드릴까요?", bb, bundle),
        ActionFaceRecognize(bb, bundle, timeout=8.0),
        ActionSpeak("화면을 보고 포즈를 취해주세요. 5, 4, 3, 2, 1. 찰칵!", bb, bundle),
        ActionPhotoCapture(bb, bundle, timeout=5.0),
        ActionSpeak("촬영 완료! 전송해 드릴게요.", bb, bundle),
        ActionNotifyWired(
            "notify_photo",
            lambda: {
                "path": bb.ctx.get("photo_path"),
                "name": bb.ctx.get("recognized_name"),
            },
            bundle,
        ),
        _build_scenario_done(bb, bundle),
    ])
    return seq
'''
