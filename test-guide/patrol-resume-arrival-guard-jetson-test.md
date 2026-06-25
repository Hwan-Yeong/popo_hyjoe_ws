# 순찰 재개 및 도착 판정 Jetson 현장 테스트

- 작성일: 2026-05-22
- 대상: Jetson Core SW + AMR + 낙상감지 API + TTS/Agent Events
- 목적: 낙상 감지 후 순찰 재개 시 `MOVING→IDLE`을 도착으로 오판하지 않고, `ARRIVED`/`ALTERNATIVE_GOAL`만 waypoint 도착으로 처리하는지 검증한다.
- 테스트 방식: Codex는 Jetson 접속, 로그 모니터링, 명령 실행, 결과 판정을 담당하고, 현장 사용자는 로봇 주변 안전 확보와 낙상 상황 재현을 담당한다.

---

## 1. 검증 범위

검증 대상:

- `ActionNavigateWaypoints`의 순찰 waypoint 진행 상태
- 낙상 감지 중 `ActionSavePatrolState -> ActionAmrStop -> VoiceStatusCheck/Notify -> ActionRestorePatrolState`
- `resume_patrol_nav` 이후 기존 목표 재전송
- AMR 도착 이벤트 판정: `MOVING→ARRIVED`, `MOVING→ALTERNATIVE_GOAL`
- AMR 비도착 이벤트 배제: `MOVING→IDLE`
- 순찰 종료 조건: 실제 마지막 waypoint 도착 후에만 `all waypoints done`

범위 밖:

- 좌표 거리 기반 도착 판정
- AMR 자체 경로계획 품질 평가
- 낙상감지 모델 정확도 평가
- 관리자 알림 서버의 외부 전달 성공률

---

## 2. PASS 핵심 기준

반드시 만족해야 한다.

| 항목 | PASS 기준 | FAIL 기준 |
|------|-----------|-----------|
| 재개 목표 전송 | 낙상 처리 후 저장된 waypoint로 `send_target_position` 재전송 | 복귀 로그는 있는데 AMR 목표 전송 없음 |
| IDLE 오판 방지 | `MOVING→IDLE` 후 `wp[n] arrived`가 바로 나오지 않음 | `MOVING→IDLE` 직후 `wp[n] arrived`, `all waypoints done` 발생 |
| 실제 도착 처리 | `MOVING→ARRIVED` 또는 `MOVING→ALTERNATIVE_GOAL` 후 `wp[n] arrived` 발생 | 도착 상태가 나왔는데 waypoint가 진행되지 않음 |
| 순찰 종료 | 마지막 waypoint 실제 도착 후에만 `all waypoints done` | 재개 직후 1~2초 내 순찰 종료 |
| 반복 안정성 | 5회 반복 중 동일 증상 0회 | 1회라도 조기 종료 발생 |

---

## 3. 선행 조건

Jetson에서 Core SW와 HW 서비스가 실행 중이어야 한다.

```bash
systemctl is-active everybot.service
systemctl is-active hw_bringup.service
curl -fsS http://127.0.0.1:8083/health
curl -fsS http://127.0.0.1:8085/healthz
curl -fsS http://127.0.0.1:8086/healthz
curl -fsS http://192.168.31.167:8008/api/health
```

필수 설정:

- `configs/feature_settings.json`의 `patrol.waypoint_sequence`가 실제 이동 가능한 경로여야 한다.
- 이번 테스트 기본 경로는 `demo -> home -> demo`를 권장한다.
- `patrol.dwell_time_sec`는 10초로 둔다.
- `patrol.repeat`는 1회 또는 테스트 목적에 맞는 고정값으로 둔다.
- 낙상 감지는 실제 API 또는 테스트 주입 중 하나로 재현 가능해야 한다.

---

## 4. Codex 터미널 구성

### Terminal A: Core SW 로그

```bash
journalctl -u everybot.service -f -o short-iso
```

확인할 주요 로그:

```text
[NavWaypoints]
[FallDetected]
[SavePatrolState]
[AmrStop]
[VoiceStatusCheck]
[RestorePatrolState]
[ClearFallDetection]
[AMR] send_target_position
[AMR] arrived event
순찰이 종료되었습니다
```

### Terminal B: AMR 상태 및 이동 이벤트만 필터링

```bash
journalctl -u everybot.service -f -o short-iso | grep -E "AMR|NavWaypoints|SavePatrolState|RestorePatrolState|FallDetected|all waypoints done"
```

### Terminal C: UI/MQTT 이벤트 관찰

```bash
mosquitto_sub -h 127.0.0.1 -p 11883 -t 'robot/#' -v
```

확인할 주요 토픽:

```text
robot/status
robot/location
robot/move/status
robot/detection
robot/config/destinations
```

---

## 5. 테스트 시작 명령

실제 운영 진입 방식에 맞춰 긴급상황 순찰 시나리오를 시작한다.

```bash
mosquitto_pub -h 127.0.0.1 -p 11883 -t "robot/cmd/release" -m '{"type":"request_scenario_start","payload":{"scenario_id":"patrol_situation_check"}}'
```

시나리오 ID가 환경에서 다르면 아래 후보 중 실제 로그에 맞는 값을 사용한다.

```text
patrol_situation_check
emergency_patrol
```

기대 로그:

```text
[Bridge] scenario_start
[LoadPatrolConfig]
[NavWaypoints] waypoints=['demo', 'home', 'demo'] repeat=1 dwell=10.0s
[AMR] send_target_position -> ...
[NavWaypoints] -> wp[0]=...
```

---

## 6. 테스트 케이스

### TC-01 정상 순찰 완료 기준선

낙상 재현 없이 `demo -> home -> demo`를 끝까지 수행한다.

PASS 로그:

```text
[NavWaypoints] wp[0] arrived
[NavWaypoints] dwell 10.0s before wp[1]
[NavWaypoints] wp[1] arrived
[NavWaypoints] dwell 10.0s before wp[2]
[NavWaypoints] wp[2] arrived
[NavWaypoints] all waypoints done
```

FAIL 로그:

```text
[NavWaypoints] wp[2] timeout
```

또는 실제 이동 없이 마지막 waypoint 완료가 발생하면 FAIL로 기록한다.

### TC-02 이동 중 낙상 감지 후 원래 목표 재개

`home -> demo` 이동 중 낙상을 재현한다.

현장 사용자 행동:

1. `wp[1] arrived`와 `dwell 10.0s before wp[2]` 이후 `wp[2]` 이동이 시작될 때까지 기다린다.
2. 로봇이 `home -> demo` 이동 중일 때 낙상 상황을 재현한다.
3. 로봇이 정지하고 상태확인/관리자 알림 흐름을 완료할 때까지 기다린다.
4. `순찰을 계속 진행합니다` TTS 이후 로봇이 다시 `demo`로 이동하는지 관찰한다.

PASS 로그:

```text
[FallDetected] status=confirmed
[SavePatrolState] target=demo idx=2 dwell_remaining=0.0
[AMR] send_stop
[AmrStop] stop sent
[RestorePatrolState] restored target=demo
[AMR] send_target_position -> ...
[NavWaypoints] -> wp[2]='demo'
[NavWaypoints] resume patrol navigation wp[2] target=demo
```

이후 실제 도착 시 아래 둘 중 하나가 나와야 한다.

```text
[AMR] arrived event (MOVING→ARRIVED)
[NavWaypoints] wp[2] arrived state=2
```

```text
[AMR] arrived event (MOVING→ALTERNATIVE_GOAL)
[NavWaypoints] wp[2] arrived state=8
```

금지 로그:

```text
[AMR] arrived event (MOVING→IDLE)
[NavWaypoints] wp[2] arrived
[NavWaypoints] all waypoints done
```

`MOVING→IDLE` 직후 위 waypoint 완료 로그가 나오면 FAIL이다.

### TC-03 dwell 중 낙상 감지 후 남은 대기 또는 다음 목표 유지

`wp[1] arrived` 후 dwell 10초 대기 중 낙상을 재현한다.

PASS 로그:

```text
[NavWaypoints] wp[1] arrived
[NavWaypoints] dwell 10.0s before wp[2]
[FallDetected] status=confirmed
[SavePatrolState] target=demo idx=2 dwell_remaining=...
[RestorePatrolState] restored target=demo
```

이후 기대 동작:

- 저장된 dwell이 남아 있으면 남은 dwell 후 `wp[2]`로 이동한다.
- dwell이 끝난 뒤 감지된 경우 `wp[2]`로 다시 목표를 보낸다.
- 어느 경우에도 `RestorePatrolState` 직후 `all waypoints done`이 바로 나오면 안 된다.

FAIL 로그:

```text
[RestorePatrolState] restored target=demo
[NavWaypoints] wp[2] arrived
[NavWaypoints] all waypoints done
```

단, 실제 AMR 도착 이벤트 `MOVING→ARRIVED` 또는 `MOVING→ALTERNATIVE_GOAL`가 선행된 경우는 PASS 후보로 본다.

### TC-04 stop/cancel성 IDLE 전이 방어

이동 중 수동 정지 또는 낙상 처리의 `send_stop`으로 AMR이 `IDLE`이 되는 상황을 만든다.

PASS 기준:

- `send_stop` 후 AMR이 `IDLE` 상태가 되더라도 `wp[n] arrived`가 발생하지 않는다.
- `NavWaypoints`가 계속 RUNNING 상태를 유지하거나, 이후 restore/resume 목표 전송을 기다린다.

금지 로그:

```text
[AMR] arrived event (MOVING→IDLE)
```

현재 패치 이후 위 로그 자체가 나오면 안 된다. 나오면 `amr_service.py` 변경이 배포되지 않은 상태로 판단한다.

### TC-05 ALTERNATIVE_GOAL 도착 처리

AMR이 목적지 근처 대체 목표로 도착하여 `movingState=8`을 반환하는 상황을 만든다.

PASS 로그:

```text
[AMR] arrived event (MOVING→ALTERNATIVE_GOAL)
[NavWaypoints] wp[n] arrived state=8
```

FAIL 로그:

```text
[AMR] arrived event (MOVING→ALTERNATIVE_GOAL)
[NavWaypoints] wp[n] timeout
```

단, `ALTERNATIVE_GOAL` 이벤트가 실제로 발생하지 않은 테스트에서는 SKIP으로 기록한다.

---

## 7. 반복 테스트 절차

권장 반복 횟수는 5회다.

| 회차 | 감지 시점 | 기대 결과 |
|------|-----------|-----------|
| 1 | 낙상 없음 | 정상 순찰 종료 |
| 2 | `wp[0] -> wp[1]` 이동 중 | restore 후 `wp[1]` 재개 |
| 3 | `wp[1]` dwell 중 | dwell/next 상태 보존 |
| 4 | `wp[1] -> wp[2]` 이동 중 | restore 후 `wp[2]` 재개 |
| 5 | `wp[2]` 도착 직전 | 실제 도착 이벤트 이후에만 순찰 종료 |

각 회차마다 아래 로그 구간을 결과 파일에 붙인다.

```bash
journalctl -u everybot.service --since "YYYY-MM-DD HH:MM:SS" --until "YYYY-MM-DD HH:MM:SS" -o short-iso > docs/test/results/YYYY-MM-DD-patrol-resume-run-N.log
```

---

## 8. Codex 판정 규칙

Codex는 로그를 아래 순서로 판정한다.

1. `SavePatrolState`의 `target`, `idx`, `dwell_remaining`을 확인한다.
2. `RestorePatrolState` 이후 `send_target_position`이 다시 나갔는지 확인한다.
3. `resume patrol navigation wp[n] target=...`가 저장 target과 일치하는지 확인한다.
4. `wp[n] arrived` 직전 3초 이내에 `[AMR] arrived event (MOVING→ARRIVED|MOVING→ALTERNATIVE_GOAL)`가 있는지 확인한다.
5. `MOVING→IDLE` 또는 `send_stop` 직후 `wp[n] arrived`가 있으면 FAIL로 판정한다.
6. `all waypoints done`은 마지막 waypoint의 유효 도착 이벤트 이후에만 PASS로 판정한다.

유효 도착 이벤트:

```text
[AMR] arrived event (MOVING→ARRIVED)
[AMR] arrived event (MOVING→ALTERNATIVE_GOAL)
```

비유효 도착 이벤트:

```text
[AMR] arrived event (MOVING→IDLE)
```

위 비유효 이벤트 로그는 패치 후에는 발생하지 않아야 한다.

---

## 9. 실시간 진단 명령

현재 Core SW 코드가 최신인지 확인:

```bash
grep -R "ALTERNATIVE_GOAL" -n homecare_robot_core_2026/src/everybot/services/amr_service.py
grep -R "amr_arrived_state" -n homecare_robot_core_2026/src/everybot/bt
```

AMR 상태 로그만 확인:

```bash
journalctl -u everybot.service -n 300 -o short-iso | grep -E "movingState|arrived event|send_target_position|send_stop|NavWaypoints"
```

낙상 API 상태 확인:

```bash
curl -s http://192.168.31.167:8008/api/fall-status | python3 -m json.tool
```

TTS/Agent Events 상태 확인:

```bash
curl -s http://127.0.0.1:8085/healthz
curl -s http://127.0.0.1:8086/healthz
```

---

## 10. 중지 및 복구 명령

시나리오 중지:

```bash
mosquitto_pub -h 127.0.0.1 -p 11883 -t "robot/cmd/release" -m '{"type":"request_scenario_stop","payload":{}}'
```

AMR 정지:

```bash
mosquitto_pub -h 127.0.0.1 -p 11883 -t "robot/cmd/stop" -m '{"type":"cmd_stop","payload":{"reason":"manual_test_stop"}}'
```

TTS/BGM 중지:

```bash
curl -s -X POST http://127.0.0.1:8083/stop -H 'Content-Type: application/json' -d '{"type":"all"}'
```

서비스 재시작:

```bash
sudo systemctl restart everybot.service
sudo systemctl restart hw_bringup.service
```

---

## 11. 결과 기록 양식

| 항목 | 결과 | 로그/비고 |
|------|------|-----------|
| 테스트 일시 |  |  |
| Jetson git commit |  | `git rev-parse --short HEAD` |
| Core SW 서비스 상태 | PASS / FAIL |  |
| 순찰 경로 |  | 예: `demo -> home -> demo` |
| dwell 시간 |  | 예: 10s |
| TC-01 정상 순찰 | PASS / FAIL / SKIP |  |
| TC-02 이동 중 낙상 후 재개 | PASS / FAIL / SKIP |  |
| TC-03 dwell 중 낙상 후 재개 | PASS / FAIL / SKIP |  |
| TC-04 `MOVING→IDLE` 오판 방어 | PASS / FAIL / SKIP |  |
| TC-05 `ALTERNATIVE_GOAL` 도착 처리 | PASS / FAIL / SKIP |  |
| 조기 `all waypoints done` 발생 | YES / NO | YES면 FAIL |
| `MOVING→IDLE` arrived event 발생 | YES / NO | YES면 배포 누락 |
| 최종 판정 | PASS / FAIL |  |

---

## 12. 실패 시 우선 확인 순서

1. `MOVING→IDLE` arrived event가 보이면 `amr_service.py` 최신 패치가 Jetson에 배포됐는지 확인한다.
2. `RestorePatrolState`는 보이는데 `send_target_position`이 없으면 `resume_patrol_nav` 경로를 확인한다.
3. `send_target_position` 직후 1초 내 `wp[n] arrived`가 나오면 직전 AMR event state를 확인한다.
4. `ALTERNATIVE_GOAL` 후 timeout이면 `cached_arrived_state -> bb.amr_arrived_state -> ActionNavigateWaypoints` 전달 경로를 확인한다.
5. `VoiceStatusCheck` 또는 TTS timeout이 길어지면 순찰 재개 전 단계가 늦어진 것이므로 주행 재개 판정과 분리해서 기록한다.

---

## 13. 최종 판정

PASS:

- 5회 반복 중 `MOVING→IDLE`에 의한 waypoint 완료가 한 번도 없다.
- 낙상 처리 후 저장된 waypoint로 목표가 재전송된다.
- `ARRIVED` 또는 `ALTERNATIVE_GOAL` 이벤트 후에만 waypoint 완료가 발생한다.
- 마지막 waypoint 실제 도착 전에는 `all waypoints done`이 발생하지 않는다.

FAIL:

- `send_stop` 또는 `MOVING→IDLE` 직후 `wp[n] arrived`가 발생한다.
- 낙상 처리 후 `RestorePatrolState`는 성공했지만 AMR 목표가 재전송되지 않는다.
- 재개 직후 실제 이동 없이 `순찰이 종료되었습니다`가 출력된다.
- `ALTERNATIVE_GOAL` 이벤트가 발생했는데 waypoint 완료로 처리되지 않는다.
