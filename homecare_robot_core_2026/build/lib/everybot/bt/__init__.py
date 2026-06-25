"""
everybot.bt — BT(Behavior Tree) 기반 시나리오 패키지.

모듈 구성:
  blackboard : RobotBlackboard (서비스 상태 공유 데이터 구조)
  bridge     : ServiceBundle + BlackboardBridge (서비스 → BB 동기화)
  debug      : DebugMode + RobotBTDebugger (py_trees 래퍼, Visitor 관리)
  nodes      : 전체 Condition + Action 노드 (단일 파일)
  tree       : build_robot_tree() — 전체 트리 조립 (단일 함수)
"""
