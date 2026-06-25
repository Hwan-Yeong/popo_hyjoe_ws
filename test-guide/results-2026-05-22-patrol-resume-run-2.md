# 순찰 재개 및 도착 판정 현장 테스트 결과

- 테스트 일시: 2026-05-22 13:37:32 ~ 13:42:09 KST
- 대상 시나리오: `patrol_situation_check`
- Jetson git commit: `b42ed33`
- 원본 로그: `test-guide/results-2026-05-22-patrol-resume-run-2.log`
- 최종 판정: PASS

## 테스트 조건

| 항목 | 값 |
|------|----|
| 순찰 경로 | `demo -> home -> demo` |
| dwell 시간 | 10초 |
| 반복 횟수 | 1 |
| 낙상 주입 | `robot/debug/fall {"fall_detected":true}` |
| Core SW 실행 | `everybot.service` active |
| 낙상 flag 설정 | 재시험 전 반영 완료 |

## 핵심 결과

| 항목 | 결과 | 로그/비고 |
|------|------|-----------|
| 시나리오 시작 | PASS | `13:37:33 [Bridge] scenario_start -> patrol_situation_check` |
| 순찰 경로 로드 | PASS | `waypoints=['demo', 'home', 'demo'] dwell=10 repeat=1` |
| TC-02 이동 중 낙상 후 재개 | PASS | `wp[0]` 이동 중 낙상 후 `idx=0 target=demo` 저장 및 동일 목표 재전송 |
| TC-04 `MOVING->IDLE` 오판 방어 | PASS | `MOVING->IDLE` arrived event 미발생 |
| TC-05 `ALTERNATIVE_GOAL` 처리 | SKIP | 이번 주행에서 `ALTERNATIVE_GOAL` 이벤트 미발생 |
| 마지막 waypoint 도착 | PASS | `13:41:22 MOVING->ARRIVED` 후 `wp[2] arrived state=2` |
| 조기 `all waypoints done` | NO | 마지막 `wp[2]` 유효 도착 이후에만 발생 |
| 종료 후 복귀 | PASS | `home` 복귀 도착 후 `ScenarioDone -> Idle` |

## 관측 타임라인

| 시각 | 이벤트 | 판정 |
|------|--------|------|
| 13:37:32 | MQTT로 `patrol_situation_check` 시작 명령 전송 | PASS |
| 13:37:38 | `wp[0]=demo` 목표 전송 | PASS |
| 13:37:54 | 낙상 주입 수신, `SavePatrolState target=demo idx=0` | PASS |
| 13:37:54 | `[AMR] send_stop`, `[AmrStop] stop sent` | PASS |
| 13:38:15 | `RestorePatrolState target=demo`, 동일 목표 재전송 | PASS |
| 13:38:55 | `MOVING->ARRIVED`, `wp[0] arrived state=2` | PASS |
| 13:39:41 | `wp[1] arrived state=2`, dwell 10초 적용 | PASS |
| 13:39:58 | 두 번째 낙상 주입, `SavePatrolState target=demo idx=2` | PASS |
| 13:40:27 | `RestorePatrolState target=demo`, `wp[2]` 재개 | PASS |
| 13:41:22 | `MOVING->ARRIVED`, `wp[2] arrived state=2`, `all waypoints done` | PASS |
| 13:41:27 | 종료 후 `NavTo home` 복귀 목표 전송 | PASS |
| 13:42:09 | 복귀 도착 후 `ScenarioDone scenario=patrol_situation_check -> Idle` | PASS |

## 특이사항

- 첫 번째 실행은 flag 설정 누락 때문에 debug fall이 emergency mode 전환까지만 되고 BT 낙상 분기로 진입하지 않았다. 해당 로그는 `test-guide/results-2026-05-22-patrol-resume-run-1.log`에 보관했다.
- 이번 재시험에서는 낙상 분기와 순찰 상태 복원이 정상 동작했다.
- `ActionSpeak` 구간에서 TTS job wait timeout 경고가 반복적으로 있었다. 순찰 재개/도착 판정에는 영향을 주지 않았지만, TTS 완료 상태 연동 개선 항목으로 별도 추적하는 것이 좋다.
- MQTT `robot/move/status`의 `target_id`가 종료 후 복귀 이동 중에도 직전 `demo`로 보이는 구간이 있었다. Core 로그상 실제 목표는 `NavTo 'home'`으로 정상 전송되었으므로 UI 상태 publish 쪽 stale target 여부를 별도 확인할 가치가 있다.

## 결론

이번 재시험 기준으로 `MOVING->IDLE` 오판에 의한 조기 waypoint 완료는 재현되지 않았다. 낙상 처리 후 저장된 waypoint로 목표가 재전송되었고, 실제 `MOVING->ARRIVED` 이후에만 `wp[n] arrived` 및 `all waypoints done`이 발생했다.
