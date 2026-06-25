"""
RobotBTDebugger — py_trees BehaviourTree 래퍼.

디버그 모드를 런타임에 교체할 수 있으며,
Visitor 를 통해 트리 상태를 stdout 에 출력하거나
graphviz dot 파일을 생성한다.
"""
from __future__ import annotations

import logging
from enum import Enum

import py_trees
import py_trees.display
import py_trees.trees
import py_trees.visitors

from .blackboard import RobotBlackboard

log = logging.getLogger(__name__)


class DebugMode(str, Enum):
    SILENT   = "silent"    # 출력 없음
    SNAPSHOT = "snapshot"  # 상태 변경 시만 출력 (기본)
    VERBOSE  = "verbose"   # 매 tick 전체 트리 출력


class _ThrottledSnapshotVisitor(py_trees.visitors.VisitorBase):
    """DisplaySnapshotVisitor 를 감싸서 N tick 마다 한 번만 출력.

    py_trees DisplaySnapshotVisitor 는 상태 변경 시마다 print() 를 하므로
    20Hz 루프에서 매우 verbose 하다.  이 래퍼는 최소 interval tick 간격을
    두어 로그 양을 제한한다.
    """

    def __init__(self, interval: int = 100) -> None:
        super().__init__(full=True)
        self._inner = py_trees.visitors.DisplaySnapshotVisitor(
            display_blackboard=False
        )
        self._interval = max(interval, 1)
        self._tick_count = 0
        self._enabled = False  # 이번 tick 에서 출력 허용 여부

    def initialise(self) -> None:
        self._tick_count += 1
        self._enabled = (self._tick_count % self._interval == 0)
        if self._enabled:
            self._inner.initialise()

    def run(self, behaviour: py_trees.behaviour.Behaviour) -> None:
        if self._enabled:
            self._inner.run(behaviour)

    def finalise(self) -> None:
        if self._enabled:
            self._inner.finalise()


class RobotBTDebugger:
    """
    py_trees.trees.BehaviourTree 를 감싸는 디버그 래퍼.

    사용 예:
        debugger = RobotBTDebugger(root, bb, DebugMode.SNAPSHOT)
        debugger.tick()          # BT tick 1회
    """

    def __init__(self, root: py_trees.behaviour.Behaviour,
                 bb: RobotBlackboard,
                 mode: DebugMode = DebugMode.SNAPSHOT,
                 snapshot_interval: int = 100) -> None:
        self._tree = py_trees.trees.BehaviourTree(root)
        self._bb   = bb
        self._mode = mode
        self._snapshot_interval = snapshot_interval
        self._apply_visitors(mode)

    # ── Public API ───────────────────────────────────────────────

    def setup(self) -> None:
        """
        py_trees 표준: tick loop 전에 반드시 1회 호출.
        모든 노드의 setup() 을 재귀적으로 실행한다.
        """
        self._tree.setup()

    def tick(self) -> None:
        """BehaviourTree.tick() 위임 (Visitor 포함)."""
        self._tree.tick()

    def print_tree(self) -> None:
        """현재 트리를 즉시 unicode 형태로 stdout 출력."""
        print(py_trees.display.unicode_tree(
            self._tree.root, show_status=True
        ))

    def print_blackboard(self) -> None:
        """현재 RobotBlackboard 상태를 예쁜 표 형태로 직접 출력."""
        import dataclasses
        data = dataclasses.asdict(self._bb)
        
        # ── 표 렌더링 ──────────────────────────────────────────
        title = " [Robot Blackboard] "
        width = 50
        print(f"\n┌{title:─^{width}}┐")
        
        for k, v in data.items():
            # ai_events 같은 리스트는 요약 출력
            if isinstance(v, list):
                val_str = f"[{len(v)} items]"
            # 좌표 딕셔너리는 짧게 출력
            elif isinstance(v, dict) and k == "amr_position":
                val_str = f"x:{v.get('x',0):.1f}, y:{v.get('y',0):.1f}"
            else:
                val_str = str(v)
            
            # 텍스트가 너무 길면 잘라냄
            if len(val_str) > (width - 20):
                val_str = val_str[:width-23] + "..."
                
            print(f"│ {k:<18} : {val_str:<27} │")
            
        print(f"└{'─'*width}┘\n")

    def export_dot(self, name: str = "robot_bt",
                   directory: str = ".") -> None:
        """
        .dot 파일과 .png 파일 생성.
        py_trees 2.2.3 버전의 dot_tree() 를 사용하여 UTF-8 인코딩 문제를 해결한다.
        """
        import os
        try:
            # 1. 트리를 pydot.Dot 객체로 변환 (py_trees 2.x 표준)
            dot_graph = py_trees.display.dot_tree(self._tree.root)
            
            # 2. .dot 파일 경로 설정 및 디렉토리 생성
            os.makedirs(directory, exist_ok=True)
            dot_path = os.path.join(directory, f"{name}.dot")
            png_path = os.path.join(directory, f"{name}.png")

            # 3. UTF-8 로 .dot 파일 직접 쓰기 (CP949 에러 방지 핵심)
            # dot_graph (pydot.Dot) 의 to_string() 은 유니코드를 보존한다.
            with open(dot_path, "w", encoding="utf-8") as f:
                f.write(dot_graph.to_string())
            
            # 4. PNG 렌더링 시도 (Graphviz 설치 필요)
            try:
                # pydot 객체의 write_png 는 외부 dot 프로그램을 호출함
                dot_graph.write_png(png_path)
                log.info("[BTDebug] dot export → %s and %s (UTF-8)", dot_path, png_path)
            except Exception as e:
                # Graphviz 미설치 시 dot 파일이라도 남겨둔다
                log.warning("[BTDebug] PNG render failed (Graphviz not found): %s", e)
                log.info("[BTDebug] .dot file created successfully at %s", dot_path)

        except Exception as exc:
            log.warning("[BTDebug] dot export failed: %s", exc)

    def set_mode(self, mode: DebugMode) -> None:
        """Visitor 교체로 런타임 디버그 모드 전환."""
        self._mode = mode
        self._apply_visitors(mode)

    @property
    def root(self) -> py_trees.behaviour.Behaviour:
        return self._tree.root

    # ── Private ──────────────────────────────────────────────────

    def _apply_visitors(self, mode: DebugMode) -> None:
        """모드에 맞는 Visitor 를 BehaviourTree 에 등록."""
        # 기존 visitor 초기화
        self._tree.visitors.clear()

        if mode == DebugMode.SNAPSHOT:
            self._tree.add_visitor(
                _ThrottledSnapshotVisitor(interval=self._snapshot_interval)
            )
        elif mode == DebugMode.VERBOSE:
            self._tree.add_visitor(
                py_trees.visitors.DebugVisitor()
            )
        # SILENT: visitor 없음
