"""
MapCreationServer - 맵 생성/편집용 5-Phase 웹서버.

맵이 없으면:
  Phase 1 맵 생성 -> Phase 2 주행 테스트 -> Phase 3 Waypoint -> Phase 4 금지영역 -> Phase 5 관심영역

맵이 있으면:
  초기 메뉴에서 Phase 2/3/4/5 중 원하는 단계로 바로 진입

stdlib 전용 (http.server + threading + SSE). 외부 패키지 없음.
"""
from __future__ import annotations

import json
import logging
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import TYPE_CHECKING
from urllib.parse import parse_qs, urlparse

if TYPE_CHECKING:
    from ..bt.blackboard import RobotBlackboard
    from ..bt.bridge import ServiceBundle
    from ..utils.waypoint_manager import WaypointManager

from ..utils.zone_manager import ZoneManager

log = logging.getLogger(__name__)

# ── 임베디드 HTML ────────────────────────────────────
_HTML = r"""<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>맵 생성 마법사</title>
<style>
  * { box-sizing: border-box; }
  body { margin: 0; background: #1a1a2e; color: #eee;
         font-family: 'Segoe UI', sans-serif; padding: 12px; }
  h2   { font-size: 1.1rem; margin: 0 0 8px; color: #7eb8f7; }
  .phase { display: none; }
  .phase.active { display: block; }

  /* 진행 표시 */
  #progress { display: flex; gap: 0; margin-bottom: 14px; }
  .step { flex: 1; padding: 8px; text-align: center; font-size: 0.82rem;
          background: #2a2a4e; border: 1px solid #333; color: #888; }
  .step.done  { background: #1b5e20; color: #a5d6a7; border-color: #2e7d32; }
  .step.current { background: #0d47a1; color: #fff; border-color: #1565c0; font-weight: bold; }

  /* 캔버스 공통 */
  .canvas-wrap { position: relative; border: 2px solid #4e8ef7;
                  border-radius: 6px; overflow: hidden; display: inline-block; }
  canvas { display: block; background: #111; cursor: crosshair; }

  /* WASD 버튼 */
  #controls { display: grid; grid-template-columns: repeat(3, 52px);
              grid-template-rows: repeat(3, 52px); gap: 5px; margin-top: 10px; }
  .btn { background: #2a2a4e; border: 1px solid #4e8ef7; border-radius: 6px;
         color: #fff; font-size: 1.1rem; cursor: pointer;
         display: flex; align-items: center; justify-content: center;
         user-select: none; }
  .btn:active, .btn.active { background: #4e8ef7; }
  .btn.empty { visibility: hidden; }

  /* 공통 버튼 */
  .action-btn { padding: 10px 28px; border: none; border-radius: 7px;
                color: #fff; font-size: 1rem; cursor: pointer; margin-top: 12px; }
  .green  { background: #27ae60; } .green:hover  { background: #219a52; }
  .blue   { background: #1565c0; } .blue:hover   { background: #0d47a1; }
  .red    { background: #c0392b; } .red:hover    { background: #a93226; }
  .action-btn:disabled { background: #555; cursor: default; }

  #status-msg { margin: 8px 0; font-size: 0.88rem; color: #aaa; min-height: 1.2em; }
  .err { color: #e74c3c; } .ok { color: #2ecc71; }

  /* Phase 2 레이아웃 */
  #p2-info { margin-top: 8px; font-size: 0.85rem; color: #aaa; }
  #p2-state { font-size: 0.9rem; margin-top: 6px; }

  /* Phase 3 레이아웃 */
  #p3-layout { display: flex; gap: 14px; align-items: flex-start; flex-wrap: wrap; }
  #p3-form-panel { background: #22223a; border-radius: 8px; padding: 12px;
                   min-width: 240px; }
  #p3-form-panel h3 { margin: 0 0 10px; font-size: 0.9rem; color: #7eb8f7; }
  .form-row { margin-bottom: 8px; }
  .form-row label { display: block; font-size: 0.78rem; color: #aaa; margin-bottom: 3px; }
  .form-row input, .form-row select {
    width: 100%; padding: 5px 8px; background: #1a1a2e; color: #eee;
    border: 1px solid #444; border-radius: 4px; font-size: 0.88rem; }
  #wp-list { margin-top: 14px; width: 100%; }
  #wp-list table { width: 100%; border-collapse: collapse; font-size: 0.8rem; }
  #wp-list th { background: #2a2a4e; padding: 5px; text-align: left; color: #aaa; }
  #wp-list td { padding: 4px 5px; border-bottom: 1px solid #333; }
  .del-btn { background: #c0392b; border: none; color: #fff; border-radius: 4px;
             padding: 2px 7px; cursor: pointer; font-size: 0.78rem; }
  .edit-btn { background: #2980b9; border: none; color: #fff; border-radius: 4px;
              padding: 2px 7px; cursor: pointer; font-size: 0.78rem; margin-right: 4px; }
  .small-btn { padding: 7px 14px; margin-right: 8px; font-size: 0.86rem; }
  .menu-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(170px, 1fr));
               gap: 12px; margin-top: 16px; }
  .menu-card { background: #22223a; border: 1px solid #35508b; border-radius: 10px;
               padding: 14px; }
  .menu-card h3 { margin: 0 0 8px; color: #7eb8f7; font-size: 0.96rem; }
  .menu-card p { margin: 0 0 12px; color: #a9aec3; font-size: 0.82rem; min-height: 3.2em; }
  .zone-layout { display: flex; gap: 14px; align-items: flex-start; flex-wrap: wrap; }
  .zone-panel { background: #22223a; border-radius: 8px; padding: 12px; min-width: 240px; }
  .zone-panel h3 { margin: 0 0 10px; font-size: 0.9rem; color: #7eb8f7; }
  .zone-help { margin-top: 8px; font-size: 0.76rem; color: #888; }
  .hint { font-size: 0.82rem; color: #aaa; margin-top: 6px; }
  #help { margin-top: 10px; font-size: 0.75rem; color: #666; }
</style>
</head>
<body>

<!-- 진행 단계 표시 -->
<div id="progress">
  <div class="step" id="step1">① 맵 생성</div>
  <div class="step" id="step2">② 주행 테스트</div>
  <div class="step" id="step3">③ Waypoint 지정</div>
  <div class="step" id="step4">④ 금지영역</div>
  <div class="step" id="step5">⑤ 관심영역</div>
</div>

<!-- ═══ Phase 0: 기존 맵 진입 메뉴 ════════════════════════════════════════ -->
<div id="phase-0" class="phase">
  <h2>기존 맵 편집 메뉴</h2>
  <div id="status-msg-menu" class="hint">기존 맵 파일이 감지되었습니다. 필요한 단계로 바로 진입할 수 있습니다.</div>
  <div class="menu-grid">
    <div class="menu-card">
      <h3>주행 테스트</h3>
      <p>현재 맵을 기준으로 AMR 목표점을 전송해 주행 가능 여부를 확인합니다.</p>
      <button class="action-btn blue small-btn" onclick="goPhase(2)">Phase 2 열기</button>
    </div>
    <div class="menu-card">
      <h3>Waypoint 편집</h3>
      <p>위치와 방향을 지정해 waypoint를 추가하거나 교체합니다.</p>
      <button class="action-btn blue small-btn" onclick="goPhase(3)">Phase 3 열기</button>
    </div>
    <div class="menu-card">
      <h3>금지영역 편집</h3>
      <p>영역 좌표를 저장하고, AMR ByPass 명령으로 즉시 반영합니다.</p>
      <button class="action-btn blue small-btn" onclick="goPhase(4)">Phase 4 열기</button>
    </div>
    <div class="menu-card">
      <h3>관심영역 편집</h3>
      <p>Core SW에서만 쓰는 ROI 영역을 저장합니다.</p>
      <button class="action-btn blue small-btn" onclick="goPhase(5)">Phase 5 열기</button>
    </div>
  </div>
</div>

<!-- ═══ Phase 1: 맵 생성 ═══════════════════════════════════════════════════ -->
<div id="phase-1" class="phase">
  <h2>Phase 1 — WASD 조종으로 맵 생성</h2>
  <div class="canvas-wrap">
    <canvas id="map1" width="600" height="400"></canvas>
  </div>
  <div id="status-msg" id="msg1">SSE 연결 대기중...</div>

  <div id="controls">
    <div class="btn empty"></div>
    <div class="btn" data-dir="up">▲</div>
    <div class="btn empty"></div>
    <div class="btn" data-dir="left">◀</div>
    <div class="btn" data-dir="stop">■</div>
    <div class="btn" data-dir="right">▶</div>
    <div class="btn empty"></div>
    <div class="btn" data-dir="down">▼</div>
    <div class="btn empty"></div>
  </div>

  <br>
  <p style="font-size:0.85rem;color:#aaa;margin:8px 0">
    ① WASD로 충전기에서 이동 → ② 맵 생성 시작 → ③ 방 전체 이동 → ④ 충전기 복귀 → ⑤ 완료
  </p>
  <button id="start-btn" class="action-btn blue" onclick="startMapping()" style="margin-right:10px">
    ▶ 맵 생성 시작
  </button>
  <button id="done-btn" class="action-btn green" onclick="sendDone()" disabled>
    ✅ 맵 생성 완료 (충전 스테이션 복귀 후 클릭)
  </button>
    <button id="done-btn2" class="action-btn green" onclick="sendDone2()">
    ✅ 있는 맵으로 테스트 (충전 스테이션 복귀 후 클릭)
  </button>
  <div id="help">키보드: W/↑ 전진 | S/↓ 후진 | A/← 좌회전 | D/→ 우회전 | Space 정지</div>
</div>

<!-- ═══ Phase 2: 주행 테스트 ════════════════════════════════════════════════ -->
<div id="phase-2" class="phase">
  <h2>Phase 2 — 미니맵 클릭으로 주행 테스트</h2>
  <div class="canvas-wrap">
    <canvas id="map2" width="600" height="400"></canvas>
  </div>
  <div id="p2-info">지도를 클릭하면 해당 위치로 주행합니다.</div>
  <div id="p2-state">
    이동 상태: <span id="p2-moving">🟢 대기</span>
    &nbsp;|&nbsp; 목적지 활성: <span id="p2-target">-</span>
  </div>
  <div id="p2-pos" style="font-size:0.8rem;color:#888;margin-top:3px;">
    현재 위치: <span id="p2-pos-val">-</span>
  </div>
  <br>
  <button class="action-btn red" onclick="sendReset()" style="margin-right:10px">
    🔄 AMR 리셋 (맵 재로드)
  </button>
  <button class="action-btn blue" onclick="goPhase(3)">→ Waypoint 지정으로</button>
  <div id="reset-msg" style="margin-top:6px;font-size:0.85rem"></div>
</div>

<!-- ═══ Phase 3: Waypoint 지정 ══════════════════════════════════════════════ -->
<div id="phase-3" class="phase">
  <h2>Phase 3 — Waypoint 지정 및 저장</h2>
  <div id="p3-layout">
    <div>
      <div class="canvas-wrap">
        <canvas id="map3" width="500" height="380"></canvas>
      </div>
      <div style="font-size:0.78rem;color:#888;margin-top:4px">
        클릭: 위치 지정 &nbsp;|&nbsp; 클릭+드래그: 위치+방향(theta) 지정
      </div>
    </div>
    <div id="p3-form-panel">
      <h3>Waypoint 추가</h3>
      <div class="form-row">
        <label>Key (고유 ID)</label>
        <input id="wp-key" type="text" placeholder="예: entrance">
      </div>
      <div class="form-row">
        <label>Label (표시명)</label>
        <input id="wp-label" type="text" placeholder="예: 입구">
      </div>
      <div class="form-row">
        <label>X (m)</label>
        <input id="wp-x" type="number" step="0.01" value="0.0">
      </div>
      <div class="form-row">
        <label>Y (m)</label>
        <input id="wp-y" type="number" step="0.01" value="0.0">
      </div>
      <div class="form-row">
        <label>Theta (rad) — <span id="theta-deg" style="color:#f39c12">0.0°</span></label>
        <input id="wp-theta" type="number" step="0.01" value="0.0"
               oninput="document.getElementById('theta-deg').textContent=
                        (parseFloat(this.value||0)*180/Math.PI).toFixed(1)+'°'">
      </div>
      <div class="form-row">
        <label>Type</label>
        <select id="wp-type">
          <option value="normal">normal</option>
          <option value="dock">dock (충전기)</option>
          <option value="home">home</option>
        </select>
      </div>
      <div class="form-row">
        <label>Comment (설명)</label>
        <input id="wp-comment" type="text" placeholder="이 시설은 OO 입니다.(안내용 설명)">
      </div>
      <div class="form-row">
        <label>Bell ID 설정 (설명)</label>
        <input id="wp-bell_id" type="text" placeholder="3FA17B18 베이스에 N 더함...(3FA17B9...)">
      </div>
      <button class="action-btn blue" onclick="addWaypoint()" style="margin-top:6px;padding:7px 18px">
        + 추가
      </button>
    </div>
  </div>

  <div id="wp-list">
    <table>
      <thead><tr><th>Key</th><th>Label</th><th>X</th><th>Y</th><th>θ</th><th>Type</th><th>Comment</th><th>Bell_ID</th><th></th></tr></thead>
      <tbody id="wp-tbody"></tbody>
    </table>
  </div>

  <br>
  <button id="save-btn" class="action-btn green" onclick="saveWaypoints()">
    💾 저장 (waypoints.json 갱신)
  </button>
  <div id="save-msg" style="margin-top:8px;font-size:0.88rem"></div>
</div>

<!-- ═══ Phase 4: 금지영역 지정 ═════════════════════════════════════════════ -->
<div id="phase-4" class="phase">
  <h2>Phase 4 — 금지영역 지정 및 저장</h2>
  <div class="zone-layout">
    <div>
      <div class="canvas-wrap">
        <canvas id="map4" width="500" height="380"></canvas>
      </div>
      <div class="zone-help">
        지도를 클릭해 polygon 점을 순서대로 추가합니다. 3개 이상 점을 찍은 뒤 영역으로 등록하세요.
      </div>
    </div>
    <div class="zone-panel">
      <h3>금지영역 추가</h3>
      <div class="form-row">
        <label>Key (고유 ID)</label>
        <input id="fz-key" type="text" placeholder="예: forbidden_livingroom">
      </div>
      <div class="form-row">
        <label>Label (표시명)</label>
        <input id="fz-label" type="text" placeholder="예: 거실 접근 금지">
      </div>
      <div class="hint">현재 점 개수: <span id="fz-point-count">0</span></div>
      <button class="action-btn blue small-btn" onclick="addZone('forbidden')">+ 영역 추가</button>
      <button class="action-btn red small-btn" onclick="clearZoneDraft('forbidden')">점 지우기</button>
    </div>
  </div>

  <div id="zone-list-forbidden" style="margin-top:14px;">
    <table>
      <thead><tr><th>Key</th><th>Label</th><th>Points</th><th></th></tr></thead>
      <tbody id="fz-tbody"></tbody>
    </table>
  </div>

  <br>
  <button id="save-fz-btn" class="action-btn green" onclick="saveZones('forbidden')">
    💾 저장 + AMR ByPass 반영
  </button>
  <div id="fz-msg" style="margin-top:8px;font-size:0.88rem"></div>
</div>

<!-- ═══ Phase 5: 관심영역 지정 ═════════════════════════════════════════════ -->
<div id="phase-5" class="phase">
  <h2>Phase 5 — 관심영역 지정 및 저장</h2>
  <div class="zone-layout">
    <div>
      <div class="canvas-wrap">
        <canvas id="map5" width="500" height="380"></canvas>
      </div>
      <div class="zone-help">
        지도를 클릭해 ROI polygon 점을 추가합니다. 저장 시 Core SW 로컬 파일만 갱신됩니다.
      </div>
    </div>
    <div class="zone-panel">
      <h3>관심영역 추가</h3>
      <div class="form-row">
        <label>Key (고유 ID)</label>
        <input id="rz-key" type="text" placeholder="예: roi_livingroom">
      </div>
      <div class="form-row">
        <label>Label (표시명)</label>
        <input id="rz-label" type="text" placeholder="예: 거실 관찰 영역">
      </div>
      <div class="hint">현재 점 개수: <span id="rz-point-count">0</span></div>
      <button class="action-btn blue small-btn" onclick="addZone('roi')">+ 영역 추가</button>
      <button class="action-btn red small-btn" onclick="clearZoneDraft('roi')">점 지우기</button>
    </div>
  </div>

  <div id="zone-list-roi" style="margin-top:14px;">
    <table>
      <thead><tr><th>Key</th><th>Label</th><th>Points</th><th></th></tr></thead>
      <tbody id="rz-tbody"></tbody>
    </table>
  </div>

  <br>
  <button id="save-rz-btn" class="action-btn green" onclick="saveZones('roi')">
    💾 ROI 저장 후 완료
  </button>
  <div id="rz-msg" style="margin-top:8px;font-size:0.88rem"></div>
</div>

<script>
// ── 공유 상태 ─────────────────────────────────────────────────
const initialState = __INITIAL_STATE__;
let currentPhase = 0;
let mapMeta = null;   // { resolution, posX, posY, width, height }
let robotPos = { x: 0, y: 0, theta: 0 };
let movingState = 0;
let robotStatus = 0;
let waypoints = [];   // Phase 3 waypoint 목록
let waypointsLoaded = false;
let forbiddenZones = [];
let roiZones = [];
let zoneDraft = [];
let zonesLoaded = { forbidden: false, roi: false };

function showMenu() {
  currentPhase = 0;
  for (let i = 0; i <= 5; i++) {
    document.getElementById(`phase-${i}`)?.classList.toggle('active', i === 0);
  }
  updateProgress(0);
}

// ── Phase 전환 ─────────────────────────────────────────────────
function goPhase(n) {
  currentPhase = n;
  for (let i = 0; i <= 5; i++) {
    document.getElementById(`phase-${i}`)?.classList.toggle('active', i === n);
  }
  updateProgress(n);
  if (n === 2) drawMap(document.getElementById('map2'));
  if (n === 3) {
    ensureWaypointsLoaded();
    drawMap(document.getElementById('map3'));
  }
  if (n === 4) {
    ensureZonesLoaded('forbidden');
    drawMap(document.getElementById('map4'));
  }
  if (n === 5) {
    ensureZonesLoaded('roi');
    drawMap(document.getElementById('map5'));
  }
}

function updateProgress(n) {
  for (let i = 1; i <= 5; i++) {
    document.getElementById(`phase-${i}`).classList.toggle('active', i === n);
    const s = document.getElementById(`step${i}`);
    if (initialState.map_ready && i === 1 && n !== 1) {
      s.className = 'step done';
      continue;
    }
    s.className = 'step ' + (i < n ? 'done' : i === n ? 'current' : '');
  }
}

// ── SSE ───────────────────────────────────────────────────────
const es = new EventSource('/api/stream');
es.onerror = () => setMsg('SSE 연결 오류. 새로고침하세요.', true);
es.onmessage = async (ev) => {
  const d = JSON.parse(ev.data);

  if (d.type === 'position') {
    robotPos = { x: d.x, y: d.y, theta: d.theta };
    movingState = d.movingState;
    // movingState: 0=IDLE, 1=MOVING, 2=도착(AMR 펌웨어 확장값)
    const stateLabel = movingState === 1 ? '🟡 이동중'
                     : movingState === 2 ? '🔵 도착완료'
                     : '🟢 대기';
    const elState = document.getElementById('p2-moving');
    if (elState) elState.textContent = stateLabel;
    const elTarget = document.getElementById('p2-target');
    if (elTarget) elTarget.textContent = d.validTargetPos ? '✅ 활성' : '⬜ 없음';
    const elPos = document.getElementById('p2-pos-val');
    if (elPos) elPos.textContent = `x=${d.x.toFixed(3)}, y=${d.y.toFixed(3)}, θ=${(d.theta * 180 / Math.PI).toFixed(1)}°`;
    drawActiveMap();
  }

  if (d.type === 'robot_status') {
    robotStatus = d.status;
    if (currentPhase === 1) {
      setMsg(`RobotStatus: ${robotStatus} ${robotStatus === 7 ? '✅ 충전 스테이션' : '(7이면 완료 가능)'}`, false);
    }
  }

  if (d.type === 'map') {
    mapMeta = { resolution: d.resolution, posX: d.posX, posY: d.posY,
                width: d.width, height: d.height };
    // PGM pixel 데이터 → ImageData 저장
    const bytes = atob(d.data);
    const arr = new Uint8Array(bytes.length);
    for (let i = 0; i < bytes.length; i++) arr[i] = bytes.charCodeAt(i);
    window._mapPixels = arr;
    window._mapW = d.width;
    window._mapH = d.height;
    drawActiveMap();
    if (currentPhase === 1) setMsg(`맵 수신 (${d.width}×${d.height}px, ${d.resolution.toFixed(3)}m/px)`, false);
  }
};

// ── 캔버스 렌더링 ─────────────────────────────────────────────
function drawActiveMap() {
  const ids = { 1: 'map1', 2: 'map2', 3: 'map3', 4: 'map4', 5: 'map5' };
  const c = document.getElementById(ids[currentPhase]);
  if (c) drawMap(c);
}

function drawMap(canvas) {
  const ctx = canvas.getContext('2d');
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  if (window._mapPixels) {
    canvas.width  = Math.min(window._mapW, 600);
    canvas.height = Math.min(window._mapH, 400);
    const idata = ctx.createImageData(canvas.width, canvas.height);
    const scaleX = window._mapW / canvas.width;
    const scaleY = window._mapH / canvas.height;
    for (let y = 0; y < canvas.height; y++) {
      for (let x = 0; x < canvas.width; x++) {
        const srcIdx = Math.floor(y * scaleY) * window._mapW + Math.floor(x * scaleX);
        const v = window._mapPixels[srcIdx] ?? 127;
        const di = (y * canvas.width + x) * 4;
        idata.data[di] = idata.data[di+1] = idata.data[di+2] = v;
        idata.data[di+3] = 255;
      }
    }
    ctx.putImageData(idata, 0, 0);
  }

  // 로봇 마커
  if (mapMeta) {
    const [cx, cy] = worldToCanvas(robotPos.x, robotPos.y, canvas);
    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate(-robotPos.theta);
    ctx.beginPath(); ctx.arc(0, 0, 7, 0, Math.PI*2);
    ctx.fillStyle = '#e74c3c'; ctx.fill();
    ctx.beginPath(); ctx.moveTo(0,0); ctx.lineTo(11,0);
    ctx.strokeStyle = '#fff'; ctx.lineWidth = 2; ctx.stroke();
    ctx.restore();
  }

  // Phase 3: waypoint 마커 표시 (원 + theta 방향 화살표)
  if (currentPhase === 3 && mapMeta) {
    waypoints.forEach(wp => {
      const [cx, cy] = worldToCanvas(wp.x, wp.y, canvas);
      // 원 마커
      ctx.beginPath(); ctx.arc(cx, cy, 6, 0, Math.PI*2);
      ctx.fillStyle = '#f39c12'; ctx.fill();
      // theta 방향 화살표 (world theta → canvas 방향: dx=cos(θ), dy=-sin(θ))
      const arrowLen = 18;
      const arrowDx =  Math.cos(wp.theta) * arrowLen;
      const arrowDy = -Math.sin(wp.theta) * arrowLen;  // world y↑ → canvas y↓
      ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(cx + arrowDx, cy + arrowDy);
      ctx.strokeStyle = '#f39c12'; ctx.lineWidth = 2; ctx.stroke();
      // 화살 끝 삼각형
      _drawArrowHead(ctx, cx, cy, cx + arrowDx, cy + arrowDy, '#f39c12');
      // 라벨 (흰색 배경 가시성: stroke outline + fill)
      ctx.font = 'bold 10px sans-serif';
      ctx.strokeStyle = '#000'; ctx.lineWidth = 3;
      ctx.lineJoin = 'round';
      ctx.strokeText(wp.key, cx + 8, cy - 8);
      ctx.fillStyle = '#f39c12';
      ctx.fillText(wp.key, cx + 8, cy - 8);
    });

    // 드래그 중: 클릭 위치에서 드래그 방향으로 임시 화살표 오버레이
    if (p3DragActive && p3DragStartPx && p3DragEndPx) {
      const [sx, sy] = [p3DragStartPx.x, p3DragStartPx.y];
      const [ex, ey] = [p3DragEndPx.x,   p3DragEndPx.y];
      const dist = Math.sqrt((ex-sx)**2 + (ey-sy)**2);
      ctx.beginPath(); ctx.arc(sx, sy, 7, 0, Math.PI*2);
      ctx.fillStyle = 'rgba(255,200,0,0.7)'; ctx.fill();
      if (dist > 5) {
        ctx.beginPath(); ctx.moveTo(sx, sy); ctx.lineTo(ex, ey);
        ctx.strokeStyle = 'rgba(255,200,0,0.9)'; ctx.lineWidth = 2;
        ctx.setLineDash([4, 3]); ctx.stroke(); ctx.setLineDash([]);
        _drawArrowHead(ctx, sx, sy, ex, ey, 'rgba(255,200,0,0.9)');
      }
    }
  }

  if ((currentPhase === 4 || currentPhase === 5) && mapMeta) {
    const zones = currentPhase === 4 ? forbiddenZones : roiZones;
    zones.forEach((zone, idx) => drawZonePolygon(ctx, canvas, zone, idx, currentPhase === 4));
    drawDraftPolygon(ctx, canvas);
  }
}

// ── 화살 끝 삼각형 그리기 헬퍼 ──────────────────────────────────
function _drawArrowHead(ctx, fromX, fromY, toX, toY, color) {
  const angle = Math.atan2(toY - fromY, toX - fromX);
  const size = 7;
  ctx.beginPath();
  ctx.moveTo(toX, toY);
  ctx.lineTo(toX - size * Math.cos(angle - Math.PI/6),
             toY - size * Math.sin(angle - Math.PI/6));
  ctx.lineTo(toX - size * Math.cos(angle + Math.PI/6),
             toY - size * Math.sin(angle + Math.PI/6));
  ctx.closePath();
  ctx.fillStyle = color; ctx.fill();
}

function worldToCanvas(wx, wy, canvas) {
  if (!mapMeta) return [canvas.width/2, canvas.height/2];
  const px = (wx - mapMeta.posX) / mapMeta.resolution;
  const py = mapMeta.height - (wy - mapMeta.posY) / mapMeta.resolution;
  return [px * (canvas.width  / mapMeta.width),
          py * (canvas.height / mapMeta.height)];
}

function canvasToWorld(cx, cy, canvas) {
  if (!mapMeta) return [0, 0];
  const px = cx * (mapMeta.width  / canvas.width);
  const py = cy * (mapMeta.height / canvas.height);
  const wx = mapMeta.posX + px * mapMeta.resolution;
  const wy = mapMeta.posY + (mapMeta.height - py) * mapMeta.resolution;
  return [wx, wy];
}

function drawZonePolygon(ctx, canvas, zone, idx, forbidden) {
  const polygon = zone.polygon || [];
  if (polygon.length === 0) return;
  ctx.save();
  ctx.beginPath();
  polygon.forEach((pt, i) => {
    const [cx, cy] = worldToCanvas(pt.x, pt.y, canvas);
    if (i === 0) ctx.moveTo(cx, cy);
    else ctx.lineTo(cx, cy);
  });
  ctx.closePath();
  ctx.fillStyle = forbidden ? 'rgba(231, 76, 60, 0.22)' : 'rgba(46, 204, 113, 0.22)';
  ctx.strokeStyle = forbidden ? '#e74c3c' : '#2ecc71';
  ctx.lineWidth = 2;
  ctx.fill();
  ctx.stroke();
  polygon.forEach((pt) => {
    const [cx, cy] = worldToCanvas(pt.x, pt.y, canvas);
    ctx.beginPath();
    ctx.arc(cx, cy, 4, 0, Math.PI * 2);
    ctx.fillStyle = forbidden ? '#ff9b90' : '#9af0b7';
    ctx.fill();
  });
  const first = polygon[0];
  const [lx, ly] = worldToCanvas(first.x, first.y, canvas);
  // 라벨 (흰색 배경 가시성: stroke outline + fill)
  ctx.font = 'bold 10px sans-serif';
  ctx.strokeStyle = '#000'; ctx.lineWidth = 3;
  ctx.lineJoin = 'round';
  const zLabel = zone.key || `zone-${idx + 1}`;
  ctx.strokeText(zLabel, lx + 8, ly - 8);
  ctx.fillStyle = forbidden ? '#ff6b6b' : '#51cf66';
  ctx.fillText(zLabel, lx + 8, ly - 8);
  ctx.restore();
}

function drawDraftPolygon(ctx, canvas) {
  if (zoneDraft.length === 0) return;
  ctx.save();
  ctx.beginPath();
  zoneDraft.forEach((pt, i) => {
    const [cx, cy] = worldToCanvas(pt.x, pt.y, canvas);
    if (i === 0) ctx.moveTo(cx, cy);
    else ctx.lineTo(cx, cy);
  });
  ctx.strokeStyle = '#f1c40f';
  ctx.lineWidth = 2;
  ctx.setLineDash([5, 4]);
  ctx.stroke();
  ctx.setLineDash([]);
  zoneDraft.forEach((pt) => {
    const [cx, cy] = worldToCanvas(pt.x, pt.y, canvas);
    ctx.beginPath();
    ctx.arc(cx, cy, 4, 0, Math.PI * 2);
    ctx.fillStyle = '#f1c40f';
    ctx.fill();
  });
  ctx.restore();
}

// ── Phase 2: 캔버스 클릭 → NavTo ─────────────────────────────
document.getElementById('map2').addEventListener('click', (e) => {
  if (currentPhase !== 2 || !mapMeta) return;
  const rect = e.target.getBoundingClientRect();
  const [wx, wy] = canvasToWorld(e.clientX - rect.left, e.clientY - rect.top, e.target);
  document.getElementById('p2-info').textContent = `목적지: (${wx.toFixed(3)}, ${wy.toFixed(3)}) 전송중...`;
  fetch('/api/nav_test', {
    method: 'POST', headers: {'Content-Type':'application/json'},
    body: JSON.stringify({ x: wx, y: wy, theta: 0.0 }),
  }).then(r => r.json()).then(d => {
    document.getElementById('p2-info').textContent =
      d.ok ? `주행 명령 전송 ✅ → (${wx.toFixed(3)}, ${wy.toFixed(3)})`
           : `오류: ${d.msg || '알 수 없음'}`;
  }).catch(() => {
    document.getElementById('p2-info').textContent = '연결 오류';
  });
});

// ── Phase 3: 캔버스 클릭+드래그 → 좌표 + theta 지정 ─────────────
// 드래그 방식: pointerdown → 클릭 위치(x,y) 확정,
//              pointermove → 드래그 방향으로 theta 실시간 계산 + 화살표 오버레이,
//              pointerup   → 최종 theta 폼 반영
let p3DragActive  = false;
let p3DragStartPx = null;    // canvas 픽셀 좌표 {x, y}
let p3DragEndPx   = null;    // canvas 픽셀 좌표 {x, y}

const map3Canvas = document.getElementById('map3');

map3Canvas.addEventListener('pointerdown', (e) => {
  if (currentPhase !== 3 || !mapMeta) return;
  e.preventDefault();
  map3Canvas.setPointerCapture(e.pointerId);
  const rect = map3Canvas.getBoundingClientRect();
  p3DragStartPx = { x: e.clientX - rect.left, y: e.clientY - rect.top };
  p3DragEndPx   = { ...p3DragStartPx };
  p3DragActive  = true;

  // 클릭 위치 → x, y 폼 채우기
  const [wx, wy] = canvasToWorld(p3DragStartPx.x, p3DragStartPx.y, map3Canvas);
  document.getElementById('wp-x').value = wx.toFixed(3);
  document.getElementById('wp-y').value = wy.toFixed(3);
  // 드래그 전에는 theta 0 유지
  document.getElementById('wp-theta').value = '0.000';
  drawMap(map3Canvas);  // 화살표 미리 그리기
});

map3Canvas.addEventListener('pointermove', (e) => {
  if (!p3DragActive || currentPhase !== 3) return;
  const rect = map3Canvas.getBoundingClientRect();
  p3DragEndPx = { x: e.clientX - rect.left, y: e.clientY - rect.top };

  // canvas 드래그 벡터 → ROS theta (x오른쪽+, y위+, 반시계 양수)
  const dx =  (p3DragEndPx.x - p3DragStartPx.x);
  const dy = -(p3DragEndPx.y - p3DragStartPx.y);  // canvas y↓ → world y↑ 반전
  const dist = Math.sqrt(dx*dx + dy*dy);
  if (dist > 5) {  // 5px 이상 드래그 시에만 theta 갱신
    const theta = Math.atan2(dy, dx);
    document.getElementById('wp-theta').value = theta.toFixed(3);
    document.getElementById('theta-deg').textContent = (theta * 180 / Math.PI).toFixed(1) + '°';
  }
  drawMap(map3Canvas);  // 화살표 오버레이 리렌더링
});

map3Canvas.addEventListener('pointerup', (e) => {
  if (!p3DragActive) return;
  p3DragActive = false;
  map3Canvas.releasePointerCapture(e.pointerId);

  // 짧은 클릭(드래그 없음)이면 theta = 0 유지, key 포커스
  document.getElementById('wp-key').focus();
  drawMap(map3Canvas);
});

// ── Phase 3: Waypoint 추가 / 삭제 / 저장 ───────────────────────
function addWaypoint() {
  const key   = document.getElementById('wp-key').value.trim();
  const label = document.getElementById('wp-label').value.trim();
  const x     = parseFloat(document.getElementById('wp-x').value) || 0;
  const y     = parseFloat(document.getElementById('wp-y').value) || 0;
  const theta = parseFloat(document.getElementById('wp-theta').value) || 0;
  const type  = document.getElementById('wp-type').value;
  const comment = document.getElementById('wp-comment').value.trim();
  const bell_id = document.getElementById('wp-bell_id').value.trim();
  if (!key) { alert('Key를 입력하세요'); return; }
  waypoints = waypoints.filter(w => w.key !== key);  // 중복 key 교체
  waypoints.push({ key, label, x, y, theta, type, comment, bell_id });
  renderWpTable();
  drawMap(document.getElementById('map3'));
  // 폼 초기화
  ['wp-key','wp-label','wp-comment','wp-bell_id'].forEach(id => document.getElementById(id).value = '');
}

function renderWpTable() {
  const tbody = document.getElementById('wp-tbody');
  tbody.innerHTML = '';
  waypoints.forEach((wp, i) => {
    const tr = document.createElement('tr');
    tr.innerHTML = `<td>${wp.key}</td><td>${wp.label}</td>
      <td>${wp.x.toFixed(3)}</td><td>${wp.y.toFixed(3)}</td>
      <td>${wp.theta.toFixed(2)}</td><td>${wp.type}</td><td>${wp.comment || ''}</td><td>${wp.bell_id || ''}</td>
      <td><button class="edit-btn" onclick="editWp(${i})">수정</button>
          <button class="del-btn" onclick="delWp(${i})">삭제</button></td>`;
    tbody.appendChild(tr);
  });
}

function editWp(i) {
  const wp = waypoints[i];
  if (!wp) return;
  document.getElementById('wp-key').value = wp.key;
  document.getElementById('wp-label').value = wp.label || '';
  document.getElementById('wp-x').value = wp.x.toFixed(3);
  document.getElementById('wp-y').value = wp.y.toFixed(3);
  document.getElementById('wp-theta').value = wp.theta.toFixed(3);
  document.getElementById('theta-deg').textContent = (wp.theta * 180 / Math.PI).toFixed(1) + '°';
  document.getElementById('wp-type').value = wp.type || 'normal';
  document.getElementById('wp-comment').value = wp.comment || '';
  document.getElementById('wp-bell_id').value = wp.bell_id || '';
  document.getElementById('wp-key').focus();
}

function delWp(i) {
  waypoints.splice(i, 1);
  renderWpTable();
  drawMap(document.getElementById('map3'));
}

function saveWaypoints() {
  if (waypoints.length === 0) { alert('waypoint가 없습니다'); return; }
  const btn = document.getElementById('save-btn');
  btn.disabled = true;
  fetch('/api/save_waypoints', {
    method: 'POST', headers: {'Content-Type':'application/json'},
    body: JSON.stringify({ waypoints }),
  }).then(r => r.json()).then(d => {
    const msg = document.getElementById('save-msg');
    if (d.ok) {
      msg.className = 'ok';
      msg.textContent = `✅ ${waypoints.length}개 waypoint 저장 완료 — 금지영역 단계로 이동합니다.`;
      setTimeout(() => goPhase(4), 800);
    } else {
      msg.className = 'err';
      msg.textContent = `오류: ${d.msg || '알 수 없음'}`;
      btn.disabled = false;
    }
  }).catch(() => {
    document.getElementById('save-msg').textContent = '연결 오류';
    btn.disabled = false;
  });
}

// ── Phase 4/5: Zone 편집 ─────────────────────────────────────
function currentZoneMode() {
  return currentPhase === 4 ? 'forbidden' : currentPhase === 5 ? 'roi' : '';
}

function clearZoneDraft(mode) {
  if (mode !== currentZoneMode()) return;
  zoneDraft = [];
  updateDraftCount(mode);
  drawActiveMap();
}

function updateDraftCount(mode) {
  const el = document.getElementById(mode === 'forbidden' ? 'fz-point-count' : 'rz-point-count');
  if (el) el.textContent = String(zoneDraft.length);
}

function zoneListRef(mode) {
  return mode === 'forbidden' ? forbiddenZones : roiZones;
}

function ensureWaypointsLoaded() {
  if (waypointsLoaded) return;
  fetch('/api/get_waypoints')
    .then(r => r.json())
    .then(d => {
      if (d.ok && Array.isArray(d.waypoints)) {
        waypoints = d.waypoints;
        renderWpTable();
        drawMap(document.getElementById('map3'));
      }
      waypointsLoaded = true;
    })
    .catch(() => {});
}

function ensureZonesLoaded(mode) {
  if (zonesLoaded[mode]) return;
  const url = `/api/get_zones?zone_type=${mode}`;
  fetch(url)
    .then(r => r.json())
    .then(d => {
      if (mode === 'forbidden') {
        forbiddenZones = Array.isArray(d.zones) ? d.zones : [];
        renderZoneTable('forbidden');
      } else {
        roiZones = Array.isArray(d.zones) ? d.zones : [];
        renderZoneTable('roi');
      }
      zonesLoaded[mode] = true;
      drawActiveMap();
    })
    .catch(() => {});
}

function addZone(mode) {
  if (mode !== currentZoneMode()) return;
  if (zoneDraft.length < 3) {
    alert('영역은 최소 3개 점이 필요합니다');
    return;
  }
  const keyInput = document.getElementById(mode === 'forbidden' ? 'fz-key' : 'rz-key');
  const labelInput = document.getElementById(mode === 'forbidden' ? 'fz-label' : 'rz-label');
  const key = keyInput.value.trim();
  const label = labelInput.value.trim();
  if (!key) {
    alert('Key를 입력하세요');
    return;
  }
  const nextZones = zoneListRef(mode).filter(zone => zone.key !== key);
  nextZones.push({ key, label, polygon: zoneDraft.map(pt => ({ x: pt.x, y: pt.y })) });
  if (mode === 'forbidden') {
    forbiddenZones = nextZones;
    renderZoneTable('forbidden');
  } else {
    roiZones = nextZones;
    renderZoneTable('roi');
  }
  keyInput.value = '';
  labelInput.value = '';
  zoneDraft = [];
  updateDraftCount(mode);
  drawActiveMap();
}

function deleteZone(mode, index) {
  if (mode === 'forbidden') {
    forbiddenZones.splice(index, 1);
    renderZoneTable('forbidden');
  } else {
    roiZones.splice(index, 1);
    renderZoneTable('roi');
  }
  drawActiveMap();
}

function renderZoneTable(mode) {
  const tbody = document.getElementById(mode === 'forbidden' ? 'fz-tbody' : 'rz-tbody');
  const zones = zoneListRef(mode);
  tbody.innerHTML = '';
  zones.forEach((zone, i) => {
    const tr = document.createElement('tr');
    const pointCount = Array.isArray(zone.polygon) ? zone.polygon.length : 0;
    tr.innerHTML = `<td>${zone.key || ''}</td><td>${zone.label || ''}</td>
      <td>${pointCount}</td>
      <td><button class="edit-btn" onclick="editZone('${mode}', ${i})">수정</button>
          <button class="del-btn" onclick="deleteZone('${mode}', ${i})">삭제</button></td>`;
    tbody.appendChild(tr);
  });
}

function editZone(mode, index) {
  const zones = zoneListRef(mode);
  const zone = zones[index];
  if (!zone) return;
  const keyInput = document.getElementById(mode === 'forbidden' ? 'fz-key' : 'rz-key');
  const labelInput = document.getElementById(mode === 'forbidden' ? 'fz-label' : 'rz-label');
  keyInput.value = zone.key || '';
  labelInput.value = zone.label || '';
  // 기존 polygon 점들을 draft로 로드 → 캔버스에 표시
  zoneDraft = (zone.polygon || []).map(pt => ({ x: pt.x, y: pt.y }));
  updateDraftCount(mode);
  // 기존 zone을 리스트에서 제거 (수정 = 삭제 후 재등록)
  zones.splice(index, 1);
  renderZoneTable(mode);
  drawActiveMap();
  keyInput.focus();
}

function saveZones(mode) {
  const btn = document.getElementById(mode === 'forbidden' ? 'save-fz-btn' : 'save-rz-btn');
  const msg = document.getElementById(mode === 'forbidden' ? 'fz-msg' : 'rz-msg');
  const zones = zoneListRef(mode);
  btn.disabled = true;
  fetch(mode === 'forbidden' ? '/api/save_forbidden_zones' : '/api/save_roi_zones', {
    method: 'POST',
    headers: {'Content-Type':'application/json'},
    body: JSON.stringify({ zones }),
  }).then(r => r.json()).then(d => {
    if (!d.ok) {
      msg.className = 'err';
      msg.textContent = `오류: ${d.msg || '알 수 없음'}`;
      btn.disabled = false;
      return;
    }
    msg.className = 'ok';
    if (mode === 'forbidden') {
      msg.textContent = `✅ ${d.count || 0}개 금지영역 저장 완료 — 관심영역 단계로 이동합니다.`;
      setTimeout(() => goPhase(5), 800);
      btn.disabled = false;
    } else {
      msg.textContent = `✅ ${d.count || 0}개 관심영역 저장 완료 — BT 후속 저장을 진행합니다.`;
    }
  }).catch(() => {
    msg.className = 'err';
    msg.textContent = '연결 오류';
    btn.disabled = false;
  });
}

function onZoneCanvasClick(mode, e) {
  if ((mode === 'forbidden' && currentPhase !== 4) || (mode === 'roi' && currentPhase !== 5) || !mapMeta) {
    return;
  }
  const rect = e.target.getBoundingClientRect();
  const [wx, wy] = canvasToWorld(e.clientX - rect.left, e.clientY - rect.top, e.target);
  zoneDraft.push({ x: Number(wx.toFixed(3)), y: Number(wy.toFixed(3)) });
  updateDraftCount(mode);
  drawMap(e.target);
}

// ── Phase 1: WASD 조종 ────────────────────────────────────────
const LIN = 0.25, ANG = 0.6;
const dirMap = { up:[LIN,0], down:[-LIN,0], left:[0,ANG], right:[0,-ANG], stop:[0,0] };
const keyDir = {
  ArrowUp:'up', w:'up', W:'up',
  ArrowDown:'down', s:'down', S:'down',
  ArrowLeft:'left', a:'left', A:'left',
  ArrowRight:'right', d:'right', D:'right',
  ' ':'stop',
};

let pressedKeys = new Set(), driveInterval = null;

function sendControl(mS, radS) {
  fetch('/api/control', {
    method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({ mS, radS }),
  }).catch(()=>{});
}

function startDriving(dir) {
  const [ms, rads] = dirMap[dir] || [0,0];
  sendControl(ms, rads);
  clearInterval(driveInterval);
  driveInterval = setInterval(() => sendControl(ms, rads), 150);
}

function stopDriving() {
  clearInterval(driveInterval); driveInterval = null;
  sendControl(0, 0);
}

document.addEventListener('keydown', e => {
  if (currentPhase !== 1 || e.repeat) return;
  const dir = keyDir[e.key];
  if (!dir) return;
  e.preventDefault();
  pressedKeys.add(e.key);
  document.querySelector(`[data-dir="${dir}"]`)?.classList.add('active');
  startDriving(dir);
});

document.addEventListener('keyup', e => {
  if (currentPhase !== 1) return;
  pressedKeys.delete(e.key);
  const dir = keyDir[e.key];
  if (dir) document.querySelector(`[data-dir="${dir}"]`)?.classList.remove('active');
  if (pressedKeys.size === 0) stopDriving();
});

document.querySelectorAll('[data-dir]').forEach(btn => {
  const dir = btn.dataset.dir;
  btn.addEventListener('pointerdown', e => {
    e.preventDefault(); startDriving(dir); btn.classList.add('active');
  });
  ['pointerup','pointerleave'].forEach(ev =>
    btn.addEventListener(ev, () => { stopDriving(); btn.classList.remove('active'); })
  );
});

// ── AMR 리셋 ──────────────────────────────────────────────────
function sendReset() {
  const msg = document.getElementById('reset-msg');
  msg.textContent = '리셋 전송중...';
  fetch('/api/reset', { method: 'POST' })
    .then(r => r.json())
    .then(d => {
      msg.style.color = d.ok ? '#2ecc71' : '#e74c3c';
      msg.textContent = d.ok
        ? '✅ AMR 리셋 전송 완료 — 약 10초 후 주행 테스트 재시도'
        : '오류: ' + (d.msg || '알 수 없음');
    }).catch(() => { msg.style.color='#e74c3c'; msg.textContent = '연결 오류'; });
}

// ── Phase 1: 맵 생성 시작 버튼 ───────────────────────────────
let mappingStarted = false;
function startMapping() {
  if (robotStatus === 7) {
    setMsg('⚠ 충전 스테이션에서 먼저 이동하세요 (WASD로 이동 후 클릭)', true);
    return;
  }
  const btn = document.getElementById('start-btn');
  btn.disabled = true;
  btn.textContent = '시작 중...';
  fetch('/api/start_mapping', { method: 'POST' })
    .then(r => r.json())
    .then(d => {
      if (d.ok) {
        mappingStarted = true;
        btn.textContent = '🗺 맵 생성 중...';
        document.getElementById('done-btn').disabled = false;
        setMsg('맵 생성 시작! 방 전체를 이동한 후 충전기로 복귀하세요.', false);
      } else {
        btn.disabled = false;
        btn.textContent = '▶ 맵 생성 시작';
        setMsg('⚠ ' + (d.msg || '오류'), true);
      }
    }).catch(() => {
      btn.disabled = false;
      btn.textContent = '▶ 맵 생성 시작';
      setMsg('연결 오류', true);
    });
}

// ── Phase 1: 완료 버튼 ────────────────────────────────────────
function setMsg(text, isErr) {
  const el = document.getElementById('status-msg');
  if (el) { el.textContent = text; el.className = isErr ? 'err' : ''; }
}

function sendDone() {
  const btn = document.getElementById('done-btn');
  btn.disabled = true;
  btn.textContent = '처리중... (맵 저장 대기)';
  fetch('/api/done', { method: 'POST' })
    .then(r => r.json())
    .then(d => {
      if (d.ok) {
        btn.textContent = '✅ 완료';
        setMsg('맵 저장 완료! Phase 2로 이동합니다...', false);
        setTimeout(() => goPhase(2), 1500);
      } else {
        btn.textContent = '✅ 맵 생성 완료 (충전 스테이션 복귀 후 클릭)';
        btn.disabled = false;
        setMsg('⚠ ' + (d.msg || '오류'), true);
      }
    }).catch(() => {
      btn.textContent = '✅ 맵 생성 완료 (충전 스테이션 복귀 후 클릭)';
      btn.disabled = false;
      setMsg('연결 오류', true);
    });
}

function sendDone2() {
  const btn = document.getElementById('done-btn2');
  btn.disabled = false;
  fetch('/api/done', { method: 'POST' })
  setTimeout(() => goPhase(2), 1500);
}

document.getElementById('map4').addEventListener('click', (e) => onZoneCanvasClick('forbidden', e));
document.getElementById('map5').addEventListener('click', (e) => onZoneCanvasClick('roi', e));

if (initialState.map_ready) showMenu();
else goPhase(1);
</script>
</body>
</html>
"""


class _Handler(BaseHTTPRequestHandler):
    """개별 HTTP 요청 처리 핸들러."""

    def log_message(self, fmt: str, *args: object) -> None:
        log.debug("[MapHTTP] " + fmt, *args)

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path in ("/", "/index.html"):
            self._send_html(self.server.map_server.render_html())  # type: ignore[attr-defined]
        elif parsed.path == "/api/stream":
            self._serve_sse()
        elif parsed.path == "/api/get_waypoints":
            self._send_json(200, self._handle_get_waypoints())
        elif parsed.path == "/api/get_zones":
            zone_type = parse_qs(parsed.query).get("zone_type", [""])[0]
            self._send_json(200, self._handle_get_zones(zone_type))
        elif parsed.path == "/api/initial_state":
            self._send_json(200, self.server.map_server.check_initial_state())  # type: ignore[attr-defined]
        else:
            self._send_json(404, {"error": "not found"})

    def do_POST(self) -> None:
        parsed = urlparse(self.path)
        length = int(self.headers.get("Content-Length", 0))
        body: dict = {}
        if length:
            try:
                body = json.loads(self.rfile.read(length))
            except Exception:
                pass

        if parsed.path == "/api/control":
            self._handle_control(body)
        elif parsed.path == "/api/start_mapping":
            self._handle_start_mapping()
        elif parsed.path == "/api/done":
            self._handle_done()
        elif parsed.path == "/api/nav_test":
            self._handle_nav_test(body)
        elif parsed.path == "/api/save_waypoints":
            self._handle_save_waypoints(body)
        elif parsed.path == "/api/save_forbidden_zones":
            self._send_json(200, self._handle_save_forbidden_zones(body))
        elif parsed.path == "/api/save_roi_zones":
            self._send_json(200, self._handle_save_roi_zones(body))
        elif parsed.path == "/api/get_zones":
            self._send_json(200, self._handle_get_zones(str(body.get("zone_type", ""))))
        elif parsed.path == "/api/reset":
            self._handle_reset()
        else:
            self._send_json(404, {"error": "not found"})

    # ── Phase 1 핸들러 ─────────────────────────────────────────

    def _handle_control(self, body: dict) -> None:
        ms   = float(body.get("mS",   0.0))
        rads = float(body.get("radS", 0.0))
        self.server.bundle.amr.send_manual_vw(ms, rads)
        self._send_json(200, {"ok": True})

    def _handle_start_mapping(self) -> None:
        """Phase 1: 맵 생성 시작. 충전기 위치에서는 거부."""
        amr = self.server.bundle.amr
        rs  = amr.cached_robot_status
        if rs == 7:
            msg = "충전 스테이션에서 이동 후 시작하세요 (AMR이 충전 스테이션 위치입니다)"
            log.warning("[MapHTTP] /api/start_mapping rejected: on charger (status=%d)", rs)
            self._send_json(400, {"ok": False, "msg": msg})
            return
        amr.send_mapping_start(manual=True)
        log.info("[MapHTTP] /api/start_mapping → mapping started")
        self._send_json(200, {"ok": True})

    def _handle_done(self) -> None:
        """Phase 1 완료: RobotStatus=7 검증 → SaveMap → MapData 수신 → 맵핑 정지.
        맵 파일은 여기서 즉시 저장하고, 전체 웹 플로우 완료 신호는 Phase 5에서 설정한다."""
        amr = self.server.bundle.amr
        rs  = amr.cached_robot_status
        if rs != 7:
            msg = f"충전 스테이션으로 이동 후 완료해 주세요 (현재 status={rs})"
            log.warning("[MapHTTP] /api/done rejected: robot_status=%d", rs)
            self._send_json(400, {"ok": False, "msg": msg})
            return

        # SaveMap 전송
        amr.send_save_map()
        log.info("[MapHTTP] SaveMap sent — waiting for MapData (max 10s)")

        # MapData 폴링 (최대 10초) — SSE 및 즉시 파일 저장을 위해 amr.latest_map_data 갱신
        deadline = time.monotonic() + 10.0
        amr.request_map_data()
        while time.monotonic() < deadline:
            if amr.latest_map_data:
                log.info("[MapHTTP] MapData received — ready for Phase 2")
                break
            time.sleep(0.5)
            amr.request_map_data()

        if not amr.latest_map_data:
            msg = "MapData 수신 실패: 맵 파일을 저장할 수 없습니다"
            log.error("[MapHTTP] /api/done failed: no MapData")
            self._send_json(504, {"ok": False, "msg": msg})
            return

        try:
            self.server.map_server.write_map_files(amr.latest_map_data)  # type: ignore[attr-defined]
            self.server.bb.latest_map_data = amr.latest_map_data
            self.server.bb.map_ready = True
        except Exception as exc:
            log.exception("[MapHTTP] map file save failed: %s", exc)
            self._send_json(500, {"ok": False, "msg": f"map save failed: {exc}"})
            return

        # 맵 생성 모드 종료 (cmd=62 set=4)
        amr.send_mapping_stop()
        log.info("[MapHTTP] /api/done → mapping stopped. Phase 2 시작. "
                 "(map file saved, full flow completion waits for Phase 5)")
        self._send_json(200, {"ok": True})

    # ── Phase 2 핸들러 ─────────────────────────────────────────

    def _handle_nav_test(self, body: dict) -> None:
        x     = float(body.get("x",     0.0))
        y     = float(body.get("y",     0.0))
        theta = float(body.get("theta", 0.0))
        # cmd=60 만 전송 (type 필드 없음, cmd=61 없음).
        # send_target_position() uses cmd=60-only AMR-compatible TargetPosition format.
        # amr_api_test.py 직접 호출(cmd=60 only) 과 동일 포맷.
        self.server.bundle.amr.send_target_position({"x": x, "y": y, "theta": theta})
        log.info("[MapHTTP] nav_test → (%.3f, %.3f, %.3f)", x, y, theta)
        self._send_json(200, {"ok": True})

    # ── Phase 3 핸들러 ─────────────────────────────────────────

    def _handle_get_waypoints(self) -> dict:
        """기존 waypoints.json 파일에서 waypoint 목록 로드 → 클라이언트 반환."""
        wm = self.server.waypoint_mgr  # type: ignore[attr-defined]
        if wm is None:
            return {"ok": False, "waypoints": [], "msg": "waypoint_mgr not available"}
        wm.reload()
        wps = [
            {
                "key": w.key, "label": w.label,
                "x": w.x, "y": w.y, "theta": w.theta,
                "type": w.type, "comment": w.comment, "bell_id": w.bell_id,
            }
            for w in wm.list()
        ]
        return {"ok": True, "waypoints": wps}

    def _handle_save_waypoints(self, body: dict) -> None:
        wps = body.get("waypoints", [])
        if not wps:
            self._send_json(400, {"ok": False, "msg": "waypoints empty"})
            return
        wm = self.server.waypoint_mgr
        if wm is None:
            self._send_json(500, {"ok": False, "msg": "waypoint_mgr not available"})
            return
        wm.save(wps)
        log.info("[MapHTTP] save_waypoints: %d waypoints saved", len(wps))
        self._send_json(200, {"ok": True})

    # ── Phase 4/5 핸들러 ───────────────────────────────────────

    def _handle_save_forbidden_zones(self, body: dict) -> dict:
        zones = body.get("zones", [])
        if not isinstance(zones, list):
            return {"ok": False, "msg": "zones must be list"}

        mgr = ZoneManager()
        for zone in zones:
            key = str(zone.get("key", "")).strip()
            if not key:
                continue
            mgr.add_zone(key, str(zone.get("label", "")), list(zone.get("polygon", [])))

        mgr.save(self.server.forbidden_zones_path)  # type: ignore[attr-defined]
        self.server.forbidden_zone_mgr = mgr        # type: ignore[attr-defined]

        bypass_areas = mgr.get_as_bypass_areas()
        try:
            self.server.bundle.amr.send_bypass(block_areas=bypass_areas)
        except Exception as exc:
            log.warning("[MapHTTP] send_bypass failed: %s", exc)
            return {"ok": False, "msg": f"send_bypass failed: {exc}"}

        log.info("[MapHTTP] save_forbidden_zones: %d zones -> cmd72", len(mgr.list()))
        return {"ok": True, "count": len(mgr.list())}

    def _handle_save_roi_zones(self, body: dict) -> dict:
        zones = body.get("zones", [])
        if not isinstance(zones, list):
            return {"ok": False, "msg": "zones must be list"}

        mgr = ZoneManager()
        for zone in zones:
            key = str(zone.get("key", "")).strip()
            if not key:
                continue
            mgr.add_zone(key, str(zone.get("label", "")), list(zone.get("polygon", [])))

        mgr.save(self.server.roi_zones_path)  # type: ignore[attr-defined]
        self.server.roi_zone_mgr = mgr        # type: ignore[attr-defined]
        self.server.bb.map_creation_done = True
        log.info("[MapHTTP] save_roi_zones: %d zones -> map_creation_done=True", len(mgr.list()))
        return {"ok": True, "count": len(mgr.list())}

    def _handle_get_zones(self, zone_type: str) -> dict:
        if zone_type == "forbidden":
            mgr = self.server.forbidden_zone_mgr  # type: ignore[attr-defined]
            mgr.load(self.server.forbidden_zones_path)  # type: ignore[attr-defined]
            return {"ok": True, "zones": mgr.list()}
        if zone_type == "roi":
            mgr = self.server.roi_zone_mgr  # type: ignore[attr-defined]
            mgr.load(self.server.roi_zones_path)  # type: ignore[attr-defined]
            return {"ok": True, "zones": mgr.list()}
        return {"ok": False, "msg": "invalid zone_type", "zones": []}

    def _handle_reset(self) -> None:
        """AMR 소프트웨어 리셋 (맵 저장 후 재초기화 시 사용)."""
        self.server.bundle.amr.send_software_reset()
        log.info("[MapHTTP] /api/reset → send_software_reset")
        self._send_json(200, {"ok": True})

    # ── SSE ────────────────────────────────────────────────────

    def _serve_sse(self) -> None:
        self.send_response(200)
        self.send_header("Content-Type",  "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection",    "keep-alive")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()

        bb     = self.server.bb
        bundle = self.server.bundle
        last_map_ts:  float = 0.0
        last_map_req: float = 0.0

        try:
            while not self.server.shutdown_event.is_set():
                now = time.monotonic()

                # ── 위치 + movingState 이벤트 ──────────────────
                pos = bundle.amr.cached_position
                ev  = json.dumps({
                    "type":               "position",
                    "x":                  pos.get("x", 0.0),
                    "y":                  pos.get("y", 0.0),
                    "theta":              pos.get("theta", 0.0),
                    "movingState":        bundle.amr.cached_moving_state,
                    "validTargetPos":     bundle.amr.cached_valid_target_position,
                })
                self._sse_send(ev)

                # ── RobotStatus 이벤트 (2초마다) ───────────────
                if now - last_map_req >= 2.0:
                    self._sse_send(json.dumps({
                        "type":   "robot_status",
                        "status": bundle.amr.cached_robot_status,
                    }))
                    # MapData 요청 (AMR은 push가 아닌 요청 응답 방식)
                    try:
                        bundle.amr.request_map_data()
                    except Exception:
                        pass
                    last_map_req = now

                # ── 맵 이벤트 (2초마다, amr.latest_map_data 직접 읽기) ──
                if now - last_map_ts >= 2.0:
                    md = bundle.amr.latest_map_data  # bb 경유 없이 직접 읽기 (bridge 타이밍 무관)
                    if md:
                        ev = json.dumps({
                            "type":       "map",
                            "width":      md.get("width",  0),
                            "height":     md.get("height", 0),
                            "resolution": md.get("resolution", 0.05),
                            "posX":       md.get("posX", 0.0),
                            "posY":       md.get("posY", 0.0),
                            "data":       md.get("data", ""),  # base64 PGM pixel
                        })
                        self._sse_send(ev)
                    last_map_ts = now

                time.sleep(0.2)
        except (BrokenPipeError, ConnectionResetError):
            pass
        except Exception as e:
            log.debug("[MapHTTP] SSE error: %s", e)

    def _sse_send(self, data: str) -> None:
        msg = f"data: {data}\n\n".encode("utf-8")
        self.wfile.write(msg)
        self.wfile.flush()

    def _send_html(self, html: str) -> None:
        body = html.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type",   "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_json(self, code: int, obj: dict) -> None:
        body = json.dumps(obj).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type",   "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


class MapCreationServer:
    """
    맵 생성/편집용 5-Phase HTTP 웹서버.

    ActionStartMapCreationServer → start()
    ActionStopMapCreationServer  → stop()
    """

    def __init__(
        self,
        bb: "RobotBlackboard",
        bundle: "ServiceBundle",
        *,
        port: int = 8080,
        host: str = "0.0.0.0",
        waypoint_mgr: "WaypointManager | None" = None,
        map_pgm_path: str = "configs/map/map.pgm",
        waypoints_path: str = "configs/waypoints.json",
        forbidden_zones_path: str = "configs/map/forbidden_zones.json",
        roi_zones_path: str = "configs/map/roi_zones.json",
    ) -> None:
        self.bb           = bb
        self.bundle       = bundle
        self.port         = port
        self.host         = host
        self.waypoint_mgr = waypoint_mgr
        self._map_pgm_path = map_pgm_path
        self._waypoints_path = waypoints_path
        self._forbidden_zones_path = forbidden_zones_path
        self._roi_zones_path = roi_zones_path
        self._forbidden_zone_mgr = ZoneManager()
        self._roi_zone_mgr = ZoneManager()
        self._ensure_storage_dirs()

        self.shutdown_event = threading.Event()
        self._httpd:  ThreadingHTTPServer | None = None
        self._thread: threading.Thread    | None = None

    def _ensure_storage_dirs(self) -> None:
        for path in (
            self._map_pgm_path,
            self._waypoints_path,
            self._forbidden_zones_path,
            self._roi_zones_path,
        ):
            parent = Path(path).parent
            if str(parent):
                parent.mkdir(parents=True, exist_ok=True)

    def check_initial_state(self) -> dict[str, bool]:
        return {
            "map_ready": os.path.exists(self._map_pgm_path),
            "waypoints_ready": os.path.exists(self._waypoints_path),
            "zones_ready": (
                os.path.exists(self._forbidden_zones_path)
                or os.path.exists(self._roi_zones_path)
            ),
        }

    def render_html(self) -> str:
        initial_state = json.dumps(self.check_initial_state(), ensure_ascii=False)
        return _HTML.replace("__INITIAL_STATE__", initial_state)

    def write_map_files(self, map_data: dict) -> None:
        import base64
        try:
            import yaml as _yaml
        except ImportError as exc:
            raise RuntimeError("PyYAML not installed") from exc

        map_path = Path(self._map_pgm_path)
        map_path.parent.mkdir(parents=True, exist_ok=True)
        yaml_path = map_path.with_suffix(".yaml")

        width = int(map_data["width"])
        height = int(map_data["height"])
        resolution = map_data["resolution"]
        origin = [map_data["posX"], map_data["posY"], 0.0]
        raw_data = base64.b64decode(map_data["data"])

        with map_path.open("wb") as fh:
            fh.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
            fh.write(raw_data)

        yaml_data = {
            "image": map_path.name,
            "resolution": resolution,
            "origin": origin,
            "negate": 0,
            "occupied_thresh": 0.65,
            "free_thresh": 0.25,
        }
        with yaml_path.open("w", encoding="utf-8") as fh:
            _yaml.dump(yaml_data, fh, default_flow_style=False)

        log.info("[MapCreationServer] saved map files: %s, %s", map_path, yaml_path)

    def start(self) -> None:
        """별도 스레드에서 HTTP 서버 기동."""
        if self._thread is not None:
            return

        self._ensure_storage_dirs()
        self.shutdown_event.clear()

        class _BoundHandler(_Handler):
            pass

        self._httpd = ThreadingHTTPServer((self.host, self.port), _BoundHandler)
        self._httpd.daemon_threads  = True                          # type: ignore[attr-defined]
        self._httpd.bb              = self.bb                       # type: ignore[attr-defined]
        self._httpd.bundle          = self.bundle                   # type: ignore[attr-defined]
        self._httpd.waypoint_mgr    = self.waypoint_mgr             # type: ignore[attr-defined]
        self._httpd.shutdown_event  = self.shutdown_event           # type: ignore[attr-defined]
        self._httpd.map_server      = self                         # type: ignore[attr-defined]
        self._httpd.forbidden_zones_path = self._forbidden_zones_path  # type: ignore[attr-defined]
        self._httpd.roi_zones_path       = self._roi_zones_path        # type: ignore[attr-defined]
        self._httpd.forbidden_zone_mgr   = self._forbidden_zone_mgr    # type: ignore[attr-defined]
        self._httpd.roi_zone_mgr         = self._roi_zone_mgr          # type: ignore[attr-defined]

        self._thread = threading.Thread(
            target=self._serve, name="map-creation-server", daemon=True,
        )
        self._thread.start()
        log.info("[MapCreationServer] started on %s:%d", self.host, self.port)

    def stop(self) -> None:
        """서버 정지."""
        self.shutdown_event.set()
        if self._httpd is not None:
            self._httpd.shutdown()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
        self._httpd  = None
        self._thread = None
        log.info("[MapCreationServer] stopped")

    def _serve(self) -> None:
        try:
            self._httpd.serve_forever(poll_interval=0.5)
        except Exception as e:
            if not self.shutdown_event.is_set():
                log.error("[MapCreationServer] serve error: %s", e)
