#!/usr/bin/env python3
"""
scripts/mqtt_inject.py — UiMqtt 로컬 브로커 테스트 주입 도구.

Windows/Linux 공통 동작. JSON 따옴표 이스케이프 불필요.

사용법:
  python scripts/mqtt_inject.py <command> [options]

명령어:
  scenario <id>       시나리오 시작 (visit_guidance|facility_guidance|care_service|patrol|photo_service)
  stop                시나리오 중지
  emergency           긴급정지
  release             긴급정지 해제
  pub <type> [json]   임의 type을 setup_cmd로 발행
  sub                 ui/# 토픽 수신 모니터링 (Ctrl+C로 종료)

예시:
  python scripts/mqtt_inject.py scenario visit_guidance
  python scripts/mqtt_inject.py emergency
  python scripts/mqtt_inject.py sub
  python scripts/mqtt_inject.py pub request_scenario_start '{"scenario_id":"patrol","params":{}}'
"""
from __future__ import annotations

import argparse
import json
import sys
import time

try:
    import paho.mqtt.client as paho
    import paho.mqtt.publish as publish
except ImportError:
    print("[ERROR] paho-mqtt가 설치되지 않았습니다.")
    print("       pip install paho-mqtt")
    sys.exit(1)


HOST    = "127.0.0.1"
PORT    = 1883
TOPIC_CMD    = "ui/setup_cmd"
TOPIC_ALL    = "ui/#"

SCENARIO_IDS = [
    "visit_guidance",
    "facility_guidance",
    "care_service",
    "patrol",
    "photo_service",
]

# ── 발행 헬퍼 ─────────────────────────────────────────────────────────────────

def pub(topic: str, msg: dict, host: str = HOST, port: int = PORT) -> None:
    payload = json.dumps(msg, ensure_ascii=False)
    publish.single(topic, payload=payload, hostname=host, port=port, qos=0)
    print(f"[PUB] {topic}")
    print(f"      {payload}")


# ── 명령별 핸들러 ─────────────────────────────────────────────────────────────

def cmd_scenario(args: argparse.Namespace) -> None:
    scenario_id = args.scenario_id
    if scenario_id not in SCENARIO_IDS:
        print(f"[ERROR] 알 수 없는 시나리오: {scenario_id}")
        print(f"        사용 가능: {', '.join(SCENARIO_IDS)}")
        sys.exit(1)

    params: dict = {}
    if args.params:
        try:
            params = json.loads(args.params)
        except json.JSONDecodeError as e:
            print(f"[ERROR] params JSON 파싱 실패: {e}")
            sys.exit(1)

    pub(TOPIC_CMD, {
        "type": "request_scenario_start",
        "payload": {
            "scenario_id": scenario_id,
            "params": params,
        },
    }, host=args.host, port=args.port)


def cmd_stop(args: argparse.Namespace) -> None:
    pub(TOPIC_CMD, {
        "type": "request_scenario_stop",
        "payload": {},
    }, host=args.host, port=args.port)


def cmd_emergency(args: argparse.Namespace) -> None:
    pub(TOPIC_CMD, {
        "type": "request_emergency_stop",
        "payload": {},
    }, host=args.host, port=args.port)


def cmd_release(args: argparse.Namespace) -> None:
    pub(TOPIC_CMD, {
        "type": "request_emergency_release",
        "payload": {},
    }, host=args.host, port=args.port)


def cmd_pub(args: argparse.Namespace) -> None:
    payload: dict = {}
    if args.json:
        try:
            payload = json.loads(args.json)
        except json.JSONDecodeError as e:
            print(f"[ERROR] JSON 파싱 실패: {e}")
            sys.exit(1)

    pub(TOPIC_CMD, {
        "type": args.type,
        "payload": payload,
    }, host=args.host, port=args.port)


def cmd_sub(args: argparse.Namespace) -> None:
    print(f"[SUB] {args.host}:{args.port} → {TOPIC_ALL}")
    print("      Ctrl+C로 종료\n")

    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            client.subscribe(TOPIC_ALL)
            print("[연결됨] ui/# 수신 대기 중...\n")
        else:
            print(f"[ERROR] 연결 실패 rc={rc}")

    def on_message(client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode("utf-8"))
            pretty = json.dumps(data, ensure_ascii=False, indent=2)
            ts = time.strftime("%H:%M:%S")
            print(f"[{ts}] {msg.topic}")
            for line in pretty.splitlines():
                print(f"      {line}")
            print()
        except Exception as e:
            print(f"[PARSE ERR] {msg.topic}: {e}")
            print(f"            raw: {msg.payload}")

    client = paho.Client(client_id="mqtt-inject-sub", clean_session=True)
    client.on_connect = on_connect
    client.on_message = on_message

    try:
        client.connect(args.host, args.port, keepalive=30)
        client.loop_forever()
    except KeyboardInterrupt:
        print("\n[종료]")
    finally:
        client.disconnect()


# ── argparse ─────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        prog="python scripts/mqtt_inject.py",
        description="UiMqtt 로컬 브로커 테스트 주입 도구",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
예시:
  python scripts/mqtt_inject.py scenario visit_guidance
  python scripts/mqtt_inject.py scenario patrol --params '{}'
  python scripts/mqtt_inject.py emergency
  python scripts/mqtt_inject.py release
  python scripts/mqtt_inject.py stop
  python scripts/mqtt_inject.py sub
  python scripts/mqtt_inject.py pub request_scenario_stop
  python scripts/mqtt_inject.py --host 192.168.60.100 scenario care_service
""",
    )
    parser.add_argument("--host", default=HOST, help=f"브로커 호스트 (default: {HOST})")
    parser.add_argument("--port", type=int, default=PORT, help=f"브로커 포트 (default: {PORT})")

    sub = parser.add_subparsers(dest="command", required=True)

    # scenario
    p_scenario = sub.add_parser("scenario", help="시나리오 시작")
    p_scenario.add_argument("scenario_id", choices=SCENARIO_IDS, help="시나리오 ID")
    p_scenario.add_argument("--params", default=None, metavar="JSON", help="추가 파라미터 JSON 문자열")

    # stop
    sub.add_parser("stop", help="시나리오 중지")

    # emergency
    sub.add_parser("emergency", help="긴급정지")

    # release
    sub.add_parser("release", help="긴급정지 해제")

    # pub
    p_pub = sub.add_parser("pub", help="임의 type 발행")
    p_pub.add_argument("type", help="메시지 type")
    p_pub.add_argument("json", nargs="?", default=None, metavar="JSON", help="payload JSON 문자열")

    # sub
    sub.add_parser("sub", help="ui/# 토픽 수신 모니터링")

    args = parser.parse_args()

    dispatch = {
        "scenario":  cmd_scenario,
        "stop":      cmd_stop,
        "emergency": cmd_emergency,
        "release":   cmd_release,
        "pub":       cmd_pub,
        "sub":       cmd_sub,
    }
    dispatch[args.command](args)


if __name__ == "__main__":
    main()
