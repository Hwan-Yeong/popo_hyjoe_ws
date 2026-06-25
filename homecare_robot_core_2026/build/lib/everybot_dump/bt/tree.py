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
    ConditionBatteryCritical,
    ConditionScenarioActive,
    ConditionPersonLyingDown,
    ConditionWifiRegistered,
    ConditionMapReady,
    ConditionFactoryReset,
    ConditionForceSoftAP,
    
    
    # ── Action ────────────────────────────────────────
    ActionMarkInitialized,
    ActionIdleStateSet,
    ActionWifiProvision,
    ActionForceSoftAP,
    ActionIdleWaiting,
    ActionSleep,
    ActionRotation,
    ActionNavigateTo,
    ActionSpeak,
    ActionSpeakFile,
    ActionWaitPersonDetected,
    ActionFaceRecognize,
    ActionPhotoCapture,
    ActionNotifyWired,
    ActionAmrStop,
    ActionScenarioDone,
    ActionWaitEmergencyRelease,
    ActionWaitBatteryReady,
    ActionFactoryReset,
    ActionStartMapCreationServer,
    ActionWaitMapCreationDone,
    ActionStopMapCreationServer,
    ActionWaitVoiceReply,
    ActionNavigateWaypoints,
    ActionAnomalyWatch,
    ActionRFBellNotify,
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
        # 모든 시나리오의 시작은 충전 위치에서 시작해야 함. home 이동이 필요 없어서 주석
        #ActionNavigateTo("home", bb, bundle, timeout=30.0),
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
        init,                                 # P=90
        battery_return,                       # P=80
        _build_force_softap(bb, bundle),      # P=75  디버깅용 강제 SoftAP
        scheduled_runner,                     # P=60  스케줄 기반 자동
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
            "emergency",
            _build_emergency_sequence(bb, bundle),
        ),
        _build_scheduled_sequence(
            bb,
            "morning_call",
            _build_morning_call_sequence(bb, bundle, bgm_root=bgm_root),
        ),
        _build_scheduled_sequence(
            bb,
            "music_play",
            _build_music_play_sequence(bb, bundle, bgm_root=bgm_root),
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
        _build_welcome(bb, bundle, tts_root=tts_root),
        _build_rotation(bb,bundle), # Debugging
        _build_move_waypoints(bb, bundle), # Debugging
        _build_info_guidance_sequence(bb, bundle, tts_root=tts_root),
        _build_hello_greeting_sequence(bb, bundle, bgm_root=bgm_root),
        _build_concierge_sequence(bb, bundle),
        _build_photo_service(bb, bundle),
        _build_content_service_sequence(bb, bundle),
    ])
    return runner


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
        if self._bb.active_scenario is not None:
            return py_trees.common.Status.FAILURE

        entry = self._find_enabled_entry()
        if entry is None:
            return py_trees.common.Status.FAILURE

        if self._scenario_id == "emergency":
            if not self._is_emergency_detected():
                return py_trees.common.Status.FAILURE
            trigger_key = self._trigger_key("sensor")
        else:
            now = datetime.datetime.now()
            current_hhmm = now.strftime("%H:%M")
            current_day = now.strftime("%a").lower()[:3]
            if current_day not in entry.days or entry.trigger_time != current_hhmm:
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
        ActionNavigateTo("home",bb,bundle),
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
        ActionIdleWaiting(bb, bundle, one_shot=True),
        ActionSpeakFile(tts_root + "1.hello.wav", bundle),
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


def _build_hello_greeting_sequence(
    bb: RobotBlackboard, bundle: ServiceBundle,
    bgm_root: str = "assets/bgm/",
) -> py_trees.composites.Sequence:
    """안부인사 시나리오. 기존 care_service 흐름을 신규 scenario_id로 이관한다."""
    bgm_file = bgm_root + "moving.wav"
    seq = _Seq("hello_greeting", memory=True)
    seq.add_children([
        ConditionScenarioActive("hello_greeting", bb),
        ActionPlayBGM(bgm_file, bundle),
        ActionNavigateTo("{target_unit}", bb, bundle, timeout=90.0),
        ActionStopAudio(bundle, stop_type="bgm"),
        ActionRFBellNotify(bb, bundle, destination_ctx_key="target_unit"),
        ActionWaitPersonDetected(bb, timeout=30.0, skip_on_timeout=True),
        ActionSpeak("안녕히 주무셨어요? 오늘 컨디션은 어떠세요?", bb, bundle),
        ActionWaitVoiceReply(bb, bundle, timeout=10.0),
        ActionSpeak("필요하신 것이 있으면 말씀해 주세요.", bb, bundle),
        ActionWaitVoiceReply(bb, bundle, timeout=10.0),
        _build_scenario_done(bb, bundle),
    ])
    return seq


def _build_emergency_sequence(
    bb: RobotBlackboard, bundle: ServiceBundle
) -> py_trees.composites.Sequence:
    """긴급상황감지 시나리오. 기존 patrol 흐름을 신규 scenario_id로 이관한다."""
    patrol_parallel = _Par(
        "emergency_parallel",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne(),
    )
    patrol_parallel.add_children([
        ActionNavigateWaypoints(bb, bundle, timeout_per_wp=300.0),
        ActionAnomalyWatch(bb, bundle),
    ])

    seq = _Seq("emergency", memory=True)
    seq.add_children([
        ConditionScenarioActive("emergency", bb),
        patrol_parallel,
        _build_scenario_done(bb, bundle),
    ])
    return seq


def _build_morning_call_sequence(
    bb: RobotBlackboard, bundle: ServiceBundle,
    bgm_root: str = "assets/bgm/",
) -> py_trees.composites.Sequence:
    """모닝콜 시나리오. Phase 9에서 스케줄 Guard가 진입 조건을 설정한다."""
    bgm_file = bgm_root + "moving.wav"
    seq = _Seq("morning_call", memory=True)
    seq.add_children([
        ConditionScenarioActive("morning_call", bb),
        ActionPlayBGM(bgm_file, bundle),
        ActionNavigateTo("{target_unit}", bb, bundle, timeout=90.0),
        ActionStopAudio(bundle, stop_type="bgm"),
        ActionRFBellNotify(bb, bundle, destination_ctx_key="target_unit"),
        ActionSpeak("좋은 아침입니다. 기상 시간입니다.", bb, bundle),
        ActionWaitVoiceReply(bb, bundle, timeout=10.0),
        _build_scenario_done(bb, bundle),
    ])
    return seq


def _build_music_play_sequence(
    bb: RobotBlackboard, bundle: ServiceBundle,
    bgm_root: str = "assets/bgm/",
) -> py_trees.composites.Sequence:
    """복도 음악 재생 시나리오. Phase 9에서 시간대/공간 Guard를 추가한다."""
    seq = _Seq("music_play", memory=True)
    seq.add_children([
        ConditionScenarioActive("music_play", bb),
        ActionNavigateTo("{zone_id}", bb, bundle, timeout=120.0),
        ActionPlayBGM(bgm_root + "moving.wav", bundle),
        ActionSleep(bb, bundle, sec=5.0),
        ActionStopAudio(bundle, stop_type="bgm"),
        _build_scenario_done(bb, bundle),
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
