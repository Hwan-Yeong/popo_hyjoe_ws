"""
bt/runner.py — BT 시나리오 Mock 테스트 실행기.

전체 흐름:
  1. BT 시작 → Init Seq (홈 이동 → initialized=True)
  2. IdleWaiting 진입 → request_show_menu 전송
  3. --scenario 지정 시 nav_duration+0.5초 뒤 자동으로 scenario_start 주입
  4. 시나리오 실행 → ScenarioDone → IdleWaiting 복귀
  5. --inject 이벤트 스케줄에 따라 중간 AI 이벤트 주입

사용법:
  python -m everybot.bt.runner --scenario visit_guidance
  python -m everybot.bt.runner --scenario patrol --inject "5.0:person_lying_down"
  python -m everybot.bt.runner --scenario visit_guidance --debug verbose --dot
  python -m everybot.bt.runner --scenario care_service --inject-emergency 4.0
  python -m everybot.bt.runner --scenario patrol --inject-battery 5
  
[Interactive Mode]
v : "visit_guidance"
f : "facility_guidance"
c : "care_service"
p : "patrol"
s : "photo_service"
e : 긴급정지, r : 긴급정지 해제
b : 배터리 10%, n : 배터리 100%
q : 종료
"""
from __future__ import annotations

import argparse
import logging
import sys
import time
from dataclasses import dataclass, field

import py_trees

from .blackboard import RobotBlackboard
from .bridge import BlackboardBridge, ServiceBundle
from .debug import DebugMode, RobotBTDebugger
from .tree import build_robot_tree
from ..services.mock import MockAmrService, MockAiService, MockWiredService
from ..services.interfaces import WiredServiceProtocol
from ..services.ui_mqtt_service import UiMqttConfig, UiMqttService

log = logging.getLogger("everybot.bt.runner")


class MockUiMqttService:
    """UiMqttService의 Mock 구현 — MQTT 브로커 없이 동작.

    WiredServiceProtocol duck-typing 준수.
    전송된 메시지는 로그로만 출력한다.
    """

    @property
    def has_client(self) -> bool:
        return True

    def try_recv(self) -> dict | None:
        return None

    def send(self, msg: dict) -> None:
        log.info("[MockUiMqtt] send type=%s payload=%s",
                 msg.get("type"), msg.get("payload"))

    def start(self) -> None:
        log.info("[MockUiMqtt] started (no broker)")

    def tick(self) -> None:
        pass

    def stop(self) -> None:
        log.info("[MockUiMqtt] stopped")

# ── 기본 웨이포인트 맵 ─────────────────────────────────────────────────────────
DEFAULT_WAYPOINTS: dict[str, dict] = {
    "home":     {"x":  0.0, "y":  0.0, "theta": 0.0},
    "dock":     {"x":  0.5, "y":  0.0, "theta": 0.0},
    # 방문자 안내
    "entrance": {"x":  3.0, "y":  0.0, "theta": 0.0},
    "lobby":    {"x":  4.0, "y":  1.0, "theta": 0.0},
    # 순찰
    "patrol_1": {"x":  5.0, "y":  0.0, "theta": 0.0},
    "patrol_2": {"x":  5.0, "y":  5.0, "theta": 0.0},
    "patrol_3": {"x":  0.0, "y":  5.0, "theta": 0.0},
    # 케어 (세대 대표 좌표 — scenario_params.target_unit 로 override 가능)
    "101":      {"x":  6.0, "y":  1.0, "theta": 0.0},
    "102":      {"x":  6.0, "y":  2.0, "theta": 0.0},
    # 시설
    "dining":   {"x":  8.0, "y":  0.0, "theta": 0.0},
    "gym":      {"x":  8.0, "y":  3.0, "theta": 0.0},
}

# ── 시나리오별 기본 파라미터 ───────────────────────────────────────────────────
SCENARIO_DEFAULT_PARAMS: dict[str, dict] = {
    "visit_guidance":    {"target_pos": "entrance"},
    "facility_guidance": {"target_facility": "dining",
                          "facility_description": "식당입니다. 운영 시간은 오전 7시부터 오후 7시입니다."},
    "care_service":      {"target_unit": "101", "unit": "101"},
    "patrol":            {},
    "photo_service":     {},
}


@dataclass
class ScheduledEvent:
    """--inject 로 등록된 시간 기반 이벤트."""
    fire_t: float
    data:   dict
    fired:  bool = False


def parse_inject_arg(raw: str) -> ScheduledEvent:
    """
    '--inject "3.0:person_lying_down,confidence=0.9"' 파싱.
    형식: <t초>:<type>[,key=value,...]
    """
    t_str, rest = raw.split(":", 1)
    parts   = rest.split(",")
    ev_type = parts[0].strip()
    ev: dict = {"type": ev_type}
    for kv in parts[1:]:
        if "=" in kv:
            k, v = kv.split("=", 1)
            try:
                ev[k.strip()] = float(v)
            except ValueError:
                ev[k.strip()] = v.strip()
        else:
            ev[kv.strip()] = True
    return ScheduledEvent(fire_t=float(t_str), data=ev)


def _build_ui_mqtt(args: argparse.Namespace) -> WiredServiceProtocol:
    """--ui-mqtt-host 지정 시 실제 UiMqttService, 미지정 시 MockUiMqttService 반환."""
    if args.ui_mqtt_host:
        cfg = UiMqttConfig(
            enabled=True,
            host=args.ui_mqtt_host,
            port=args.ui_mqtt_port,
            client_id="Everybot-ui-client-runner",
        )
        svc = UiMqttService(cfg)
        svc.start()
        log.info("[Runner] UiMqttService → %s:%d", args.ui_mqtt_host, args.ui_mqtt_port)
        return svc
    return MockUiMqttService()


def run(args: argparse.Namespace) -> None:
    """메인 실행 함수."""

    # ── Mock 서비스 생성 ────────────────────────────────────────────────────
    amr   = MockAmrService(nav_duration=args.nav_duration)
    ai    = MockAiService()
    wired = MockWiredService()

    # --inject-battery: 즉시 배터리 설정
    if args.inject_battery is not None:
        amr.inject_battery(float(args.inject_battery))
        log.info("[Runner] battery forced to %.1f%%", float(args.inject_battery))

    # ── UiMqtt (실제 or Mock) ────────────────────────────────────────────────
    ui_mqtt = _build_ui_mqtt(args)

    # ── BT 구성 ─────────────────────────────────────────────────────────────
    bb       = RobotBlackboard()
    bundle   = ServiceBundle(
        amr=amr, ai=ai, wired=wired,
        ui_mqtt=ui_mqtt,
        waypoints=dict(DEFAULT_WAYPOINTS),
    )
    bridge = BlackboardBridge(bb, bundle)
    root   = build_robot_tree(bb, bundle)
    mode   = DebugMode(args.debug)
    dbg    = RobotBTDebugger(root, bb, mode)

    # py_trees 표준: tick loop 전에 tree.setup() 호출
    dbg.setup()

    # ── 인터랙티브 모드 준비 ─────────────────────────────────────────────────
    if args.interactive:
        import msvcrt
        print("\n[Interactive Mode] 키를 눌러 이벤트를 주입하세요:")
        print("  v, f, c, p, s : 시나리오 주입")
        print("  e : 긴급정지, r : 긴급정지 해제")
        print("  b : 배터리 10%, n : 배터리 100%")
        print("  q : 종료\n")

    # ── 이벤트 스케줄 파싱 ─────────────────────────────────────────────────
    schedule: list[ScheduledEvent] = []
    for raw in (args.inject or []):
        try:
            schedule.append(parse_inject_arg(raw))
        except Exception as exc:
            log.error("[Runner] inject 파싱 실패 '%s': %s", raw, exc)

    # --inject-emergency: 긴급정지 wired 이벤트 스케줄 추가
    if args.inject_emergency is not None:
        schedule.append(ScheduledEvent(
            fire_t=float(args.inject_emergency),
            data={"_wired": True, "type": "request_emergency_stop", "payload": {}},
        ))

    # ── 시나리오 주입 타이밍 ────────────────────────────────────────────────
    # Init NavigateTo("home") 완료 후 자동 주입
    # = nav_duration + 여유 0.5초
    scenario_inject_t = args.nav_duration + 0.5
    scenario_injected = (args.scenario is None)  # None이면 주입 건너뜀

    # ── 배너 출력 ────────────────────────────────────────────────────────────
    print(f"\n{'='*62}")
    print(f"  BT Runner  |  py_trees {py_trees.version.__version__}")
    print(f"{'='*62}")
    print(f"  scenario    : {args.scenario or '(none — IdleWaiting 확인용)'}")
    print(f"  hz          : {args.hz}")
    print(f"  nav_duration: {args.nav_duration}s")
    print(f"  debug       : {args.debug}")
    print(f"  events      : {len(schedule)} scheduled")
    print(f"  inject_t    : t={scenario_inject_t:.1f}s (Init 완료 후)")
    print(f"{'='*62}\n")

    # ── tick loop ────────────────────────────────────────────────────────────
    hz        = max(1.0, args.hz)
    tick_dt   = 1.0 / hz
    duration  = args.duration
    t: float  = 0.0
    ticks: int = 0

    try:
        while True:
            t_start = time.monotonic()

            # 1. 인터랙티브 입력 처리 (msvcrt.kbhit() 사용)
            if args.interactive and msvcrt.kbhit():
                # raw byte 를 읽어 ascii 로 안전하게 변환
                ch = msvcrt.getch()
                try:
                    key = ch.decode("ascii").lower()
                    _handle_interactive_input(key, wired, amr, log)
                    if key == "q":
                        break
                except (UnicodeDecodeError, AttributeError):
                    # 특수키(F1~F12, 화살표 등) 무시
                    pass

            # 2. 스케줄 이벤트 실행
            for ev in schedule:
                if not ev.fired and t >= ev.fire_t:
                    ev.fired = True
                    if ev.data.get("_wired"):
                        # wired 메시지 주입 (긴급정지 등)
                        msg = {k: v for k, v in ev.data.items() if k != "_wired"}
                        wired.inject_cmd(msg)
                        log.info("[Schedule] t=%.2f wired → %s", t, msg.get("type"))
                    else:
                        ai.inject_event(ev.data)
                        log.info("[Schedule] t=%.2f ai → %s", t, ev.data.get("type"))

            # 2. Init 완료 감지 → 시나리오 자동 주입
            if not scenario_injected and t >= scenario_inject_t:
                params = SCENARIO_DEFAULT_PARAMS.get(args.scenario, {})
                wired.inject_scenario_start(args.scenario, params)
                log.info("[Runner] t=%.2f inject scenario_start: %s params=%s",
                         t, args.scenario, params)
                scenario_injected = True

            # 3. Bridge update (서비스 상태 → BB 동기화)
            passthrough = bridge.update()
            for msg in passthrough:
                log.debug("[Passthrough] %s", msg.get("type"))

            # 4. BT tick (핵심)
            dbg.tick()
            if args.interactive:
                dbg.print_blackboard()
            ticks += 1

            # 5. 완료 감지 — notify_scenario_done 수신
            done_msg = wired.get_last_sent("notify_scenario_done")
            if done_msg:
                wired.clear_sent()
                print(f"\n[완료] notify_scenario_done @ t={t:.1f}s (ticks={ticks})")
                print(f"       payload={done_msg.get('payload')}\n")
                if args.duration == 0:
                    # 0 = 완료까지. 시나리오 완료 후 idle 복귀 확인하고 종료.
                    # 다음 request_show_menu 를 기다렸다가 종료
                    _wait_for_idle(wired, bridge, dbg, bb, hz)
                    #break

            # 6. 시간 제한
            if duration > 0 and t >= duration:
                print(f"\n[종료] 시간 제한 {duration:.1f}s (ticks={ticks})\n")
                break

            t += tick_dt
            elapsed = time.monotonic() - t_start
            sleep_t = max(0.0, tick_dt - elapsed)
            if sleep_t > 0:
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        print("\n[중단] Ctrl+C")

    # ── 최종 트리 상태 출력 ─────────────────────────────────────────────────
    print("\n[최종 트리 상태]")
    dbg.print_tree()

    # dot export
    if args.dot:
        dbg.export_dot("robot_bt", ".")
        print("[dot] robot_bt.dot / .png 생성 완료")

    print(f"\n총 ticks: {ticks}  |  시뮬레이션 시간: {t:.2f}s\n")


def _wait_for_idle(
    wired: MockWiredService,
    bridge: BlackboardBridge,
    dbg: RobotBTDebugger,
    bb: RobotBlackboard,
    hz: float,
) -> None:
    """
    ScenarioDone 이후 IdleWaiting으로 복귀하여 request_show_menu가
    전송되는 것을 확인하고 반환한다 (최대 3초).
    """
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        bridge.update()
        dbg.tick()
        msg = wired.get_last_sent("request_show_menu")
        if msg:
            print("[확인] Idle 복귀 → request_show_menu 전송됨 ✓\n")
            return
        time.sleep(1.0 / hz)
    print("[경고] request_show_menu 미확인 (3초 초과)\n")


def _handle_interactive_input(key: str, wired: MockWiredService, amr: MockAmrService, logger: logging.Logger) -> None:
    """키보드 입력에 따른 이벤트 주입."""
    scenarios = {
        "v": "visit_guidance",
        "f": "facility_guidance",
        "c": "care_service",
        "p": "patrol",
        "s": "photo_service",
    }

    if key in scenarios:
        sid = scenarios[key]
        params = SCENARIO_DEFAULT_PARAMS.get(sid, {})
        wired.inject_scenario_start(sid, params)
        logger.info("[Interactive] inject scenario_start: %s", sid)
    elif key == "e":
        wired.inject_cmd({"type": "request_emergency_stop", "payload": {}})
        logger.warning("[Interactive] EMERGENCY STOP triggered")
    elif key == "r":
        wired.inject_cmd({"type": "request_emergency_release", "payload": {}})
        logger.info("[Interactive] EMERGENCY RELEASED")
    elif key == "b":
        amr.inject_battery(10.0)
        logger.warning("[Interactive] battery set to 10.0%")
    elif key == "n":
        amr.inject_battery(100.0)
        logger.info("[Interactive] battery set to 100.0%")
    elif key == "q":
        logger.info("[Interactive] quitting...")


def main() -> None:
    # Windows cp949 콘솔에서 py_trees 유니코드 출력 보장
    if sys.platform == "win32" and hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
        datefmt="%H:%M:%S",
        stream=sys.stdout,
    )

    parser = argparse.ArgumentParser(
        prog="python -m everybot.bt.runner",
        description="BT 시나리오 Mock 테스트 실행기",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
예시:
  # 기본 방문자 안내
  python -m everybot.bt.runner --scenario visit_guidance

  # 순찰 + 5초 뒤 쓰러진 사람 감지
  python -m everybot.bt.runner --scenario patrol --inject "5.0:person_lying_down"

  # 긴급정지 테스트
  python -m everybot.bt.runner --scenario visit_guidance --inject-emergency 4.0

  # 배터리 부족 테스트
  python -m everybot.bt.runner --inject-battery 5 --duration 20

  # verbose 모드 + dot 출력
  python -m everybot.bt.runner --scenario visit_guidance --debug verbose --dot

[Interactive Mode]
v : "visit_guidance"
f : "facility_guidance"
c : "care_service"
p : "patrol"
s : "photo_service"
e : 긴급정지, r : 긴급정지 해제
b : 배터리 10%, n : 배터리 100%
q : 종료  
""",
    )
    parser.add_argument(
        "--scenario",
        choices=["visit_guidance", "facility_guidance",
                 "care_service", "patrol", "photo_service"],
        default=None,
        help="실행할 시나리오 (생략 시 IdleWaiting 확인 모드)",
    )
    parser.add_argument(
        "--hz", type=float, default=20.0,
        help="tick 속도 Hz (default: 20.0)",
    )
    parser.add_argument(
        "--duration", type=float, default=0.0,
        help="실행 시간(초), 0=완료까지 (default: 0)",
    )
    parser.add_argument(
        "--debug",
        choices=["snapshot", "verbose", "silent"],
        default="snapshot",
        help="디버그 출력 모드 (default: snapshot)",
    )
    parser.add_argument(
        "--dot", action="store_true",
        help="종료 시 robot_bt.dot + .png 생성 (graphviz 필요)",
    )
    parser.add_argument(
        "--inject", action="append", metavar="T:TYPE[,key=val]",
        help="AI 이벤트 주입 스케줄 (반복 가능). 예: --inject '3.0:person_lying_down'",
    )
    parser.add_argument(
        "--inject-emergency", type=float, default=None,
        dest="inject_emergency",
        metavar="T",
        help="T초 뒤 UI 긴급정지(request_emergency_stop) 주입",
    )
    parser.add_argument(
        "--inject-battery", type=float, default=None,
        dest="inject_battery",
        metavar="PCT",
        help="배터리를 즉시 PCT%%로 설정 (예: --inject-battery 5)",
    )
    parser.add_argument(
        "--nav-duration", type=float, default=3.0,
        dest="nav_duration",
        help="MockAmr 이동 시뮬레이션 시간(초) (default: 3.0)",
    )
    parser.add_argument(
        "--interactive", action="store_true",
        help="키보드 입력을 통해 실시간으로 이벤트를 주입하는 모드",
    )
    parser.add_argument(
        "--ui-mqtt-host", default=None,
        dest="ui_mqtt_host",
        metavar="HOST",
        help="실제 MQTT 브로커 호스트 (예: 127.0.0.1). 지정 시 Mock 대신 실제 UiMqttService 사용",
    )
    parser.add_argument(
        "--ui-mqtt-port", type=int, default=1883,
        dest="ui_mqtt_port",
        metavar="PORT",
        help="MQTT 브로커 포트 (default: 1883)",
    )

    args = parser.parse_args()
    run(args)


if __name__ == "__main__":
    main()
