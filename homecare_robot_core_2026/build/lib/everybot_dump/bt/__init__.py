"""
everybot.bt — BT(Behavior Tree) 기반 시나리오 스켈레톤 패키지.

모듈 구성:
  blackboard : RobotBlackboard (서비스 상태 공유 데이터 구조)
  bridge     : ServiceBundle + BlackboardBridge (서비스 → BB 동기화)
  debug      : DebugMode + RobotBTDebugger (py_trees 래퍼, Visitor 관리)
  nodes      : 전체 Condition(7) + Action(10) 노드 (단일 파일)
  tree       : build_robot_tree() — 전체 트리 조립 (단일 함수)
  runner     : CLI Mock 테스트 실행기

실행:
  python -m everybot.bt.runner --help
  python -m everybot.bt.runner --scenario visit_guidance
  python -m everybot.bt.runner --scenario patrol --inject "5.0:person_lying_down"
"""
