#!/usr/bin/env python3
"""
speaker_manager.py — 블루투스 스피커 관리 SW

이중 스레드 동시 재생:
  BGM  (_bgm_thread)  : bgm 전용 큐, 루프 재생 지원
  HIGH (_high_thread) : tts / alert 전용 큐
  → PulseAudio가 두 스트림을 자동 믹싱해 BT 스피커로 출력

Audio Ducking:
  TTS/Alert 시작 → _duck.set() → BGM 볼륨 BGM_DUCK_VOLUME(15%)로 낮춤
  TTS/Alert 종료 → _duck.clear() → BGM 볼륨 _volumes["bgm"]으로 복원

HTTP API :8083:
  POST /play   {"file": str, "type": "tts|alert|bgm", "loop": bool}
  POST /stop   {"type": "all|bgm|tts"}
  POST /volume {"type": "bgm|tts|alert", "volume": 0-100}
  GET  /status
  GET  /health

전원 관리:
  startup 시 GPIO PIN29(PQ.05, offset 105)에 2초 HIGH 펄스 자동 발화.
  외부 API 미노출 — speaker_manager 내부 전용.

BT 연결 관리:
  _bt_monitor_thread: 3초 주기 bluetoothctl 폴링 → _bt_connected 자동 갱신.
  pactl은 user-session PulseAudio 전용으로 systemd/root 환경에서 Connection refused.
  bluetoothctl은 BlueZ D-Bus API 경유로 root에서도 동작.
  active = bt_connected=True (GET /status "active" 필드 확인).

내부상수 (소스 수정):
    GPIO_DISABLE : True 설정 시 GPIO 트리거 skip (CI/테스트용)
    BT_MAC       : BT 스피커 MAC 주소 ("" 시 bluetoothctl skip)
    BT_SINK      : paplay --device sink 이름 ("" 시 SPEAKER_BT_SINK env 참조)

환경변수:
  SPEAKER_BT_SINK      : BT_SINK 미설정 시 sink 이름 지정 fallback
"""
import json
import logging
import os
import queue
import re
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

log = logging.getLogger(__name__)

try:
    import yaml as _yaml
except Exception:
    _yaml = None


def _load_hw_cfg() -> dict:
    """Load hw_components.yaml; return {} when unavailable to preserve defaults."""
    if _yaml is None:
        return {}
    cfg_path = os.environ.get(
        "HW_CONFIG_PATH",
        os.path.join(os.path.dirname(__file__), "../../../configs/hw_components.yaml"),
    )
    try:
        with open(cfg_path, encoding="utf-8") as cfg_file:
            return _yaml.safe_load(cfg_file) or {}
    except (FileNotFoundError, OSError, _yaml.YAMLError):
        return {}


_CFG = _load_hw_cfg()
_COMMON_CFG = _CFG.get("common", {})
_SPEAKER_CFG = _CFG.get("speaker", {})

# ── 설정 상수 ──────────────────────────────────────────────────────────
HW_DATA_DIR          = _COMMON_CFG.get("hw_data_dir", "/dev/shm/hw_data")
SPEAKER_DATA_DIR     = os.path.join(HW_DATA_DIR, "speaker")
SPEAKER_INFO_PATH    = os.path.join(SPEAKER_DATA_DIR, "speaker_info.json")
API_PORT             = _SPEAKER_CFG.get("api_port", 8083)

VALID_TYPES          = {"tts", "alert", "bgm"}

GPIO_DISABLE = _SPEAKER_CFG.get("gpio_disable", True)

# ── GPIO 설정 ──────────────────────────────────────────────────────────
GPIO_CHIP            = _SPEAKER_CFG.get("gpio_chip", "/dev/gpiochip0")
GPIO_LINE            = _SPEAKER_CFG.get("gpio_line", 105)               # PIN29 = PQ.05
GPIO_POWER_PULSE_SEC = _SPEAKER_CFG.get("gpio_power_pulse_sec", 2.0)

# ── BT 모니터 설정 ─────────────────────────────────────────────────────
BT_POLL_INTERVAL_SEC = _SPEAKER_CFG.get("bt_poll_interval_sec", 3.0)
BT_RECOVERY_MAX_RETRIES = _SPEAKER_CFG.get("bt_recovery_max_retries", 5)
BT_RECOVERY_BACKOFF_SEC = _SPEAKER_CFG.get("bt_recovery_backoff_sec", 10.0)

# ── BT 스피커 고정 설정 ────────────────────────────────────────────────
# 근본 원인: pactl/paplay는 user-session PulseAudio daemon 전용.
#   systemd service가 root로 실행되면 /run/user/0/pulse/native 소켓 없음
#   → Connection refused → paplay silent exit → 무음.
# 해결:
#   - BT 연결 확인 → bluetoothctl (BlueZ D-Bus, root에서도 동작)
#   - paplay 호출 → AUDIO_USER_UID(1000)로 PULSE_SERVER 명시해 env 전달
#
# AUDIO_USER_UID: PulseAudio를 실행하는 user UID (everybot=1000).
#                 hw_bringup이 root 실행이므로 os.getuid()=0 → 직접 지정.
# BT_MAC:  BT 스피커 MAC 주소.
# BT_SINK: paplay --device 에 사용할 sink 이름.
#          PulseAudio : bluez_sink.{MAC_콜론→언더}.a2dp_sink
#          PipeWire   : bluez_output.{MAC_콜론→언더}.1
AUDIO_USER_UID  = _SPEAKER_CFG.get("audio_user_uid", 1000)               # everybot UID
AUDIO_USER_HOME = _SPEAKER_CFG.get("audio_user_home", "/home/everybot")   # PulseAudio cookie 경로 계산용
BT_MAC  = _SPEAKER_CFG.get("bt_mac", "DA:55:CB:9F:03:B6")
BT_SINK = _SPEAKER_CFG.get("bt_sink", "bluez_sink.DA_55_CB_9F_03_B6.a2dp_sink")

# ── 타입별 초기 볼륨 설정 (0-100%) ────────────────────────────────────
VOLUME_BGM   = _SPEAKER_CFG.get("volume_bgm", 40)    # 이동 중 배경음악 — 대화 방해 최소화
VOLUME_TTS   = _SPEAKER_CFG.get("volume_tts", 50)    # 안내/인사 음성
VOLUME_ALERT = _SPEAKER_CFG.get("volume_alert", 80)   # 알림음 — 반드시 들려야 함

# ── 덕킹 볼륨 ──────────────────────────────────────────────────────────
BGM_DUCK_VOLUME = _SPEAKER_CFG.get("bgm_duck_volume", 15)   # TTS/Alert 재생 중 BGM 임시 볼륨 (%)

USE_PYGAME = _SPEAKER_CFG.get("use_pygame", True)
PYGAME_FREQUENCY = _SPEAKER_CFG.get("pygame_frequency", 44100)
PYGAME_BUFFER = _SPEAKER_CFG.get("pygame_buffer", 1024)
PYGAME_RETRY_INTERVAL_SEC = 2.0
_CORS_ALLOW_ORIGINS_CFG = _SPEAKER_CFG.get("cors_allow_origins", ["*"])
if isinstance(_CORS_ALLOW_ORIGINS_CFG, str):
    SPEAKER_CORS_ALLOW_ORIGINS = [
        origin.strip() for origin in _CORS_ALLOW_ORIGINS_CFG.split(",") if origin.strip()
    ] or ["*"]
else:
    SPEAKER_CORS_ALLOW_ORIGINS = list(_CORS_ALLOW_ORIGINS_CFG or ["*"])


# ── GPIOD 버전 감지 (모듈 레벨) ───────────────────────────────────────
try:
    import importlib.metadata as _imeta
    _GPIOD_V2: bool = int(_imeta.version("gpiod").split(".")[0]) >= 2
except Exception:
    _GPIOD_V2 = False


# ── 자료구조 ───────────────────────────────────────────────────────────

@dataclass
class PlayItem:
    """재생 큐 아이템."""
    file: str
    type: str    # "tts" | "alert" | "bgm"
    loop: bool = False
    fade_ms: int = 0


# ── Pydantic 모델 — 모듈 레벨 정의 필수 (함수 내부 정의 시 ForwardRef 오류 발생)
try:
    from pydantic import BaseModel, Field, field_validator

    class PlayRequest(BaseModel):
        file: str = Field(..., description="Audio file path. WAV/OGG is recommended for pygame backend.")
        type: str = Field("tts", description="Audio type: bgm | tts | alert")
        loop: bool = Field(False, description="Loop playback. Mostly used for BGM.")
        fade_ms: int = Field(0, ge=0, description="Fade-in time in milliseconds. 0 means immediate playback.")

        @field_validator("type")
        @classmethod
        def _check_type(cls, v: str) -> str:
            if v not in VALID_TYPES:
                raise ValueError(f"type must be one of {VALID_TYPES}")
            return v

    class StopRequest(BaseModel):
        type: str = "all"

    class VolumeRequest(BaseModel):
        type:   str = Field(..., description="Volume type: bgm | tts | alert")
        volume: int = Field(..., ge=0, le=100, description="Volume percentage, 0-100")

        @field_validator("type")
        @classmethod
        def _check_type(cls, v: str) -> str:
            if v not in VALID_TYPES:
                raise ValueError(f"type must be one of {VALID_TYPES}")
            return v

        @field_validator("volume")
        @classmethod
        def _check_volume(cls, v: int) -> int:
            if not 0 <= v <= 100:
                raise ValueError("volume must be between 0 and 100")
            return v

except ImportError:
    PlayRequest   = None  # type: ignore
    StopRequest   = None  # type: ignore
    VolumeRequest = None  # type: ignore


# ── 공유 상태 ──────────────────────────────────────────────────────────
_stop       = threading.Event()
_start_time = time.time()
_state_lock = threading.Lock()

# 런타임 볼륨 — /volume API로 변경 가능 (CPython GIL → dict 읽기/쓰기 원자적)
_volumes: dict[str, int] = {
    "bgm":   VOLUME_BGM,
    "tts":   VOLUME_TTS,
    "alert": VOLUME_ALERT,
}

# 이중 재생 큐
_bgm_queue:  queue.Queue[PlayItem] = queue.Queue()
_high_queue: queue.Queue[PlayItem] = queue.Queue()

# 현재 재생 아이템 (_state_lock 보호)
_bgm_item:  Optional[PlayItem] = None
_high_item: Optional[PlayItem] = None

# 현재 재생 프로세스 (_state_lock 보호, PID 조회용)
_bgm_proc:  Optional[subprocess.Popen] = None
_high_proc: Optional[subprocess.Popen] = None
_last_play_error: str = ""

# 제어 이벤트
_duck      = threading.Event()  # _high_thread → _bgm_thread 덕킹 신호
_stop_bgm  = threading.Event()  # BGM 스트림 강제 중단
_stop_high = threading.Event()  # TTS/Alert 스트림 강제 중단

_bt_connected = False
_pygame_ok = False
_pg = None
_BGM_CH = None
_TTS_CH = None
_ALERT_CH = None
_pygame_init_lock = threading.Lock()
_speaker_info_lock = threading.Lock()


def _set_last_play_error(message: str) -> None:
    global _last_play_error
    with _state_lock:
        _last_play_error = message


def _clear_last_play_error() -> None:
    _set_last_play_error("")


def _drain_queue(q: queue.Queue) -> int:
    drained = 0
    while not q.empty():
        try:
            q.get_nowait()
            drained += 1
        except queue.Empty:
            break
    return drained


def _validate_play_file(file_path: str) -> Optional[str]:
    if not file_path or not file_path.strip():
        return "file path is empty"
    if not os.path.isfile(file_path):
        return f"audio file not found: {file_path}"
    return None


# ── GPIO 유틸 ──────────────────────────────────────────────────────────

def _gpio_power_trigger(sec: float = GPIO_POWER_PULSE_SEC) -> bool:
    """
    GPIOD v1/v2 호환 — GPIO LINE에 sec초간 HIGH 펄스 출력.
    SPEAKER_DISABLE_GPIO 환경변수 존재 시 skip(True 반환).
    실패 시 False 반환 (서비스 계속 유지).
    """
    if GPIO_DISABLE:
        log.info("[speaker_manager] GPIO 트리거 skip (SPEAKER_DISABLE_GPIO 설정)")
        return True
    try:
        if _GPIOD_V2:
            import gpiod
            from gpiod.line import Direction, Value
            cfg = gpiod.LineSettings(
                direction=Direction.OUTPUT,
                output_value=Value.INACTIVE,
            )
            with gpiod.request_lines(
                GPIO_CHIP,
                consumer="spk-power",
                config={GPIO_LINE: cfg},
            ) as req:
                req.set_value(GPIO_LINE, Value.ACTIVE)
                time.sleep(sec)
                req.set_value(GPIO_LINE, Value.INACTIVE)
                time.sleep(0.05)
        else:
            import gpiod
            chip = gpiod.Chip(GPIO_CHIP)
            line = chip.get_line(GPIO_LINE)
            line.request(
                consumer="spk-power",
                type=gpiod.LINE_REQ_DIR_OUT,
                default_val=0,
            )
            line.set_value(1)
            time.sleep(sec)
            line.set_value(0)
            time.sleep(0.05)
            line.release()
        log.info("[speaker_manager] GPIO 전원 트리거 완료 (%.1fs)", sec)
        return True
    except Exception as exc:
        log.warning("[speaker_manager] GPIO 트리거 실패 — %s", exc)
        return False


def _gpio_startup_thread() -> None:
    """main()에서 daemon Thread로 기동 — startup 1회 GPIO 트리거."""
    log.info("[speaker_manager] startup GPIO 트리거 시작")
    _gpio_power_trigger()
    log.info("[speaker_manager] startup GPIO 트리거 완료")


# ── PulseAudio 유틸 ────────────────────────────────────────────────────

def _pulse_env() -> dict:
    """
    paplay/pactl subprocess에 전달할 PulseAudio 소켓+인증 환경 반환.

    root → everybot PulseAudio 소켓 접근 문제:
      1) Connection refused  → PULSE_SERVER=unix:/run/user/0/pulse/native (wrong UID)
         Fix: AUDIO_USER_UID(1000)로 경로 고정
      2) Access denied       → PulseAudio cookie 인증 실패
         Fix: PULSE_COOKIE=everybot의 cookie 파일 경로 명시
    """
    env = os.environ.copy()
    xdg = f"/run/user/{AUDIO_USER_UID}"
    env["XDG_RUNTIME_DIR"]    = xdg
    env["PULSE_RUNTIME_PATH"] = f"{xdg}/pulse"
    env["PULSE_SERVER"]       = f"unix:{xdg}/pulse/native"

    for cookie in (
        f"{AUDIO_USER_HOME}/.config/pulse/cookie",
        f"{AUDIO_USER_HOME}/.pulse-cookie",
    ):
        if os.path.exists(cookie):
            env["PULSE_COOKIE"] = cookie
            break
    else:
        log.warning("[speaker_manager] PulseAudio cookie 미발견 (Access denied 가능)")

    return env


def _apply_pulse_env() -> None:
    """pygame가 root 실행 환경에서도 everybot PulseAudio 소켓을 보도록 환경을 적용."""
    env = _pulse_env()
    for key in ("XDG_RUNTIME_DIR", "PULSE_RUNTIME_PATH", "PULSE_SERVER", "PULSE_COOKIE"):
        if key in env:
            os.environ[key] = env[key]
    if BT_SINK:
        os.environ["PULSE_SINK"] = BT_SINK


def _pulse_socket_path() -> str:
    return f"/run/user/{AUDIO_USER_UID}/pulse/native"


def _init_pygame() -> bool:
    """pygame.mixer 초기화. 실패 시 paplay fallback을 사용한다."""
    global _pygame_ok, _pg
    if not USE_PYGAME:
        log.info("[speaker_manager] pygame 비활성화 - paplay fallback 사용")
        return False
    with _pygame_init_lock:
        if _pygame_ok:
            return True
        try:
            os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
            os.environ.setdefault("SDL_AUDIODRIVER", "pulseaudio")
            _apply_pulse_env()

            import pygame as pygame_mod

            pygame_mod.mixer.pre_init(
                frequency=PYGAME_FREQUENCY,
                size=-16,
                channels=2,
                buffer=PYGAME_BUFFER,
            )
            pygame_mod.mixer.init()
            pygame_mod.mixer.set_num_channels(8)
            _pg = pygame_mod
            _pygame_ok = True
            _setup_pygame_channels()
            log.info(
                "[speaker_manager] pygame.mixer 초기화 완료 freq=%d buffer=%d sink=%s",
                PYGAME_FREQUENCY,
                PYGAME_BUFFER,
                BT_SINK or os.getenv("SPEAKER_BT_SINK", ""),
            )
            return True
        except Exception as exc:
            _pygame_ok = False
            _pg = None
            log.warning("[speaker_manager] pygame 초기화 실패 - paplay fallback 사용: %s", exc)
            return False


def _pygame_retry_thread() -> None:
    """부팅 직후 PulseAudio user socket이 늦게 준비되는 경우 pygame 초기화를 재시도."""
    if not USE_PYGAME:
        return

    pulse_socket = _pulse_socket_path()
    waiting_logged = False

    while not _stop.is_set() and not _pygame_ok:
        if not os.path.exists(pulse_socket):
            if not waiting_logged:
                log.info("[speaker_manager] pygame 재시도 대기 - PulseAudio 소켓 미준비: %s", pulse_socket)
                waiting_logged = True
            _stop.wait(PYGAME_RETRY_INTERVAL_SEC)
            continue

        waiting_logged = False
        if _init_pygame():
            _write_speaker_info()
            log.info("[speaker_manager] pygame backend 활성화 완료 (재시도 성공)")
            return
        _stop.wait(PYGAME_RETRY_INTERVAL_SEC)


def _setup_pygame_channels() -> None:
    """pygame Channel 0/1/2를 BGM/TTS/Alert 전용으로 할당."""
    global _BGM_CH, _TTS_CH, _ALERT_CH
    if _pg is None:
        return
    _BGM_CH = _pg.mixer.Channel(0)
    _TTS_CH = _pg.mixer.Channel(1)
    _ALERT_CH = _pg.mixer.Channel(2)


def _set_pygame_channel_volume(audio_type: str, pct: int) -> bool:
    if not _pygame_ok:
        return False
    vol = max(0.0, min(1.0, pct / 100.0))
    if audio_type == "bgm" and _BGM_CH is not None and _BGM_CH.get_busy():
        effective = BGM_DUCK_VOLUME / 100.0 if _duck.is_set() else vol
        _BGM_CH.set_volume(effective)
        return True
    if audio_type == "tts" and _TTS_CH is not None and _TTS_CH.get_busy():
        _TTS_CH.set_volume(vol)
        return True
    if audio_type == "alert" and _ALERT_CH is not None and _ALERT_CH.get_busy():
        _ALERT_CH.set_volume(vol)
        return True
    return False


def _pct_to_pa(pct: int) -> int:
    """볼륨 백분율(0-100) → PulseAudio 정수(0-65536)."""
    return max(0, min(65536, int(pct / 100 * 65536)))


# ── BT 연결 유틸 ──────────────────────────────────────────────────────

def _get_bt_sink() -> str:
    """
    paplay --device 에 사용할 BT A2DP sink 이름 반환.
    BT_SINK 상수 → SPEAKER_BT_SINK 환경변수 순으로 참조.
    """
    if BT_SINK:
        return BT_SINK
    fixed = os.getenv("SPEAKER_BT_SINK", "")
    if fixed:
        return fixed
    log.warning("[speaker_manager] BT_SINK 미설정 — SPEAKER_BT_SINK 환경변수 또는 상수를 지정하세요")
    return ""


def _check_bt_sink_connected() -> bool:
    """
    BT 스피커 연결 상태 확인.
    BT_MAC 설정 시 bluetoothctl info {MAC} 사용 (BlueZ D-Bus, root에서도 동작).
    """
    if not BT_MAC:
        return bool(BT_SINK or os.getenv("SPEAKER_BT_SINK", ""))
    try:
        out = subprocess.check_output(
            ["bluetoothctl", "info", BT_MAC],
            stderr=subprocess.DEVNULL,
            timeout=3.0,
        ).decode()
        connected = "Connected: yes" in out
        log.debug("[speaker_manager] bluetoothctl %s → connected=%s", BT_MAC, connected)
        return connected
    except Exception as exc:
        log.warning("[speaker_manager] bluetoothctl 실패: %s", exc)
        return False


def _try_recover_bt_connection() -> bool:
    """BT_MAC이 설정된 경우 bluetoothctl connect로 연결 복구를 시도."""
    if not BT_MAC:
        return False

    for attempt in range(1, BT_RECOVERY_MAX_RETRIES + 1):
        if _stop.is_set():
            return False
        log.warning(
            "[speaker_manager] BT 연결 끊김 - 복구 시도 %d/%d",
            attempt,
            BT_RECOVERY_MAX_RETRIES,
        )
        try:
            proc = subprocess.run(
                ["bluetoothctl", "connect", BT_MAC],
                capture_output=True,
                text=True,
                timeout=10.0,
                check=False,
            )
            if proc.returncode == 0 and _check_bt_sink_connected():
                log.info("[speaker_manager] BT 복구 성공")
                return True
            detail = (proc.stderr or proc.stdout or "").strip()
            if detail:
                log.warning("[speaker_manager] BT 복구 실패: %s", detail)
        except Exception as exc:
            log.warning("[speaker_manager] BT 복구 명령 실패: %s", exc)
        _stop.wait(BT_RECOVERY_BACKOFF_SEC)
    return False


def _bt_monitor_thread() -> None:
    """BT 연결 상태를 주기적으로 감시, _bt_connected 갱신."""
    global _bt_connected
    log.info("[speaker_manager] BT 모니터 스레드 시작")
    while not _stop.is_set():
        connected = _check_bt_sink_connected()
        if not connected and BT_RECOVERY_MAX_RETRIES > 0:
            connected = _try_recover_bt_connection()
        if connected != _bt_connected:
            _bt_connected = connected
            _write_speaker_info()
            log.info("[speaker_manager] BT 연결 상태 변경 → %s", connected)
        _stop.wait(BT_POLL_INTERVAL_SEC)
    log.info("[speaker_manager] BT 모니터 스레드 종료")


# ── 볼륨 실시간 제어 ───────────────────────────────────────────────────

def _parse_sink_input_by_pid(pid: int) -> Optional[int]:
    """
    pactl list sink-inputs 파싱 → PID에 해당하는 sink-input index 반환.
    실패 시 None 반환 (호출자가 무시).
    """
    try:
        out = subprocess.check_output(
            ["pactl", "list", "sink-inputs"],
            env=_pulse_env(),
            stderr=subprocess.DEVNULL,
            timeout=3.0,
        ).decode()
    except Exception:
        return None

    current_index: Optional[int] = None
    for line in out.splitlines():
        m = re.match(r"\s*Sink Input #(\d+)", line)
        if m:
            current_index = int(m.group(1))
        if f'application.process.id = "{pid}"' in line and current_index is not None:
            return current_index
    return None


def _set_stream_volume(proc: subprocess.Popen, pct: int) -> bool:
    """
    실행 중인 paplay proc의 볼륨을 즉시 변경.
    pactl 실패 시 False 반환 (재생은 계속).
    """
    if proc is None or proc.poll() is not None:
        return False
    idx = _parse_sink_input_by_pid(proc.pid)
    if idx is None:
        return False
    try:
        subprocess.run(
            ["pactl", "set-sink-input-volume", str(idx), f"{pct}%"],
            env=_pulse_env(),
            stderr=subprocess.DEVNULL,
            timeout=2.0,
            check=False,
        )
        return True
    except Exception:
        return False


# ── 파일 갱신 ─────────────────────────────────────────────────────────

def _write_speaker_info() -> None:
    """speaker_info.json 원자적 갱신."""
    with _speaker_info_lock:
        os.makedirs(SPEAKER_DATA_DIR, exist_ok=True)
        with _state_lock:
            bgm  = _bgm_item
            high = _high_item
            last_play_error = _last_play_error
        info = {
            "bt_connected": _bt_connected,
            "active":       _bt_connected,
            "playing_bgm": {
                "type": bgm.type,
                "file": bgm.file,
                "loop": bgm.loop,
                "fade_ms": bgm.fade_ms,
            } if bgm else None,
            "playing_high": {
                "type": high.type,
                "file": high.file,
                "loop": high.loop,
                "fade_ms": high.fade_ms,
            } if high else None,
            "backend": "pygame" if _pygame_ok else "paplay",
            "use_pygame": USE_PYGAME,
            "ducking":    _duck.is_set(),
            "volumes":    dict(_volumes),
            "queue_bgm":  _bgm_queue.qsize(),
            "queue_high": _high_queue.qsize(),
            "last_play_error": last_play_error,
        }
        tmp = f"{SPEAKER_INFO_PATH}.{os.getpid()}.{threading.get_ident()}.tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(info, f, indent=4)
        os.replace(tmp, SPEAKER_INFO_PATH)


# ── BGM 스레드 ─────────────────────────────────────────────────────────

def _bgm_thread() -> None:
    """
    BGM 큐(_bgm_queue) 처리 스레드.
    루프 재생 지원. _duck 이벤트 감지 → 볼륨 자동 조정 (덕킹/복원).
    """
    global _bgm_item, _bgm_proc
    log.info("[speaker_manager] BGM 스레드 시작")

    while not _stop.is_set():
        try:
            item = _bgm_queue.get(timeout=0.5)
        except queue.Empty:
            continue

        if not _bt_connected:
            _set_last_play_error(f"bt not connected: {item.file}")
            _write_speaker_info()
            log.warning("[speaker_manager] BGM BT 미연결 — skip: %s", item.file)
            continue

        sink = _get_bt_sink()
        if not sink:
            _set_last_play_error(f"bt sink unavailable: {item.file}")
            _write_speaker_info()
            log.warning("[speaker_manager] BGM BT sink 미발견 — skip: %s", item.file)
            continue

        if _pygame_ok and _pg is not None and _BGM_CH is not None:
            try:
                sound = _pg.mixer.Sound(item.file)
            except Exception as exc:
                _set_last_play_error(f"bgm load failed: {item.file} ({exc})")
                _write_speaker_info()
                log.error("[speaker_manager] BGM Sound 로드 실패: %s — %s", item.file, exc)
                continue

            with _state_lock:
                _bgm_item = item
            _clear_last_play_error()
            _write_speaker_info()

            start_vol = BGM_DUCK_VOLUME if _duck.is_set() else _volumes["bgm"]
            sound.set_volume(start_vol / 100.0)
            _BGM_CH.set_volume(start_vol / 100.0)
            _stop_bgm.clear()
            _BGM_CH.play(sound, loops=-1 if item.loop else 0, fade_ms=item.fade_ms)
            log.info(
                "[speaker_manager] BGM pygame 재생 시작 → file=%s loop=%s vol=%d%% fade_ms=%d",
                item.file, item.loop, start_vol, item.fade_ms,
            )

            while not _stop.is_set():
                if _stop_bgm.is_set():
                    _stop_bgm.clear()
                    _BGM_CH.stop()
                    log.info("[speaker_manager] BGM 중단 요청")
                    break
                if not _BGM_CH.get_busy():
                    break
                target = BGM_DUCK_VOLUME if _duck.is_set() else _volumes["bgm"]
                _BGM_CH.set_volume(target / 100.0)
                _stop.wait(0.05)

            with _state_lock:
                _bgm_item = None
            _write_speaker_info()
            continue

        # 덕킹 중이면 덕킹 볼륨으로 시작
        start_vol = BGM_DUCK_VOLUME if _duck.is_set() else _volumes["bgm"]
        vol_pa    = _pct_to_pa(start_vol)
        pulse_env = _pulse_env()
        _stop_bgm.clear()

        def _play_bgm(fp: str) -> subprocess.Popen:
            return subprocess.Popen(
                ["paplay", "--device", sink, "--volume", str(vol_pa), fp],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                env=pulse_env,
            )

        proc = _play_bgm(item.file)
        with _state_lock:
            _bgm_proc = proc
            _bgm_item = item
        _clear_last_play_error()
        _write_speaker_info()

        log.info(
            "[speaker_manager] BGM 재생 시작 → file=%s loop=%s vol=%d%%",
            item.file, item.loop, start_vol,
        )

        was_ducked = _duck.is_set()

        while not _stop.is_set():
            if _stop_bgm.is_set():
                _stop_bgm.clear()
                log.info("[speaker_manager] BGM 중단 요청")
                break

            # 덕킹 상태 변화 감지 → 즉시 볼륨 변경
            is_ducked = _duck.is_set()
            if is_ducked != was_ducked:
                was_ducked = is_ducked
                target = BGM_DUCK_VOLUME if is_ducked else _volumes["bgm"]
                _set_stream_volume(proc, target)
                log.debug(
                    "[speaker_manager] BGM 덕킹 %s → vol=%d%%",
                    "ON" if is_ducked else "OFF", target,
                )

            rc = proc.poll()
            if rc is not None:
                if rc != 0:
                    err = proc.stderr.read().decode(errors="replace").strip()
                    _set_last_play_error(f"bgm paplay failed rc={rc}: {err or item.file}")
                    _write_speaker_info()
                    log.warning("[speaker_manager] BGM paplay 종료 rc=%d stderr: %s", rc, err)
                if item.loop and not _stop.is_set() and not _stop_bgm.is_set():
                    cur_vol  = BGM_DUCK_VOLUME if _duck.is_set() else _volumes["bgm"]
                    vol_pa   = _pct_to_pa(cur_vol)
                    proc     = _play_bgm(item.file)
                    with _state_lock:
                        _bgm_proc = proc
                    was_ducked = _duck.is_set()
                else:
                    break

            time.sleep(0.1)

        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                proc.kill()

        with _state_lock:
            _bgm_proc = None
            _bgm_item = None
        _write_speaker_info()

    log.info("[speaker_manager] BGM 스레드 종료")


# ── HIGH 스레드 (TTS / Alert) ──────────────────────────────────────────

def _high_thread() -> None:
    """
    TTS/Alert 큐(_high_queue) 처리 스레드.
    재생 시작 시 _duck.set() → BGM 덕킹. 재생 완료 시 _duck.clear() → 복원.
    """
    global _high_item, _high_proc
    log.info("[speaker_manager] HIGH 스레드 시작")

    while not _stop.is_set():
        try:
            item = _high_queue.get(timeout=0.5)
        except queue.Empty:
            continue

        if not _bt_connected:
            _set_last_play_error(f"bt not connected: {item.file}")
            _write_speaker_info()
            log.warning("[speaker_manager] HIGH BT 미연결 — skip: %s", item.file)
            continue

        sink = _get_bt_sink()
        if not sink:
            _set_last_play_error(f"bt sink unavailable: {item.file}")
            _write_speaker_info()
            log.warning("[speaker_manager] HIGH BT sink 미발견 — skip: %s", item.file)
            continue

        if _pygame_ok and _pg is not None:
            ch = _ALERT_CH if item.type == "alert" else _TTS_CH
            if ch is None:
                _set_last_play_error(f"pygame channel unavailable: {item.file}")
                _write_speaker_info()
                log.warning("[speaker_manager] HIGH pygame channel 미초기화 — skip: %s", item.file)
                continue
            try:
                sound = _pg.mixer.Sound(item.file)
            except Exception as exc:
                _set_last_play_error(f"high load failed: {item.file} ({exc})")
                _write_speaker_info()
                log.error("[speaker_manager] HIGH Sound 로드 실패: %s — %s", item.file, exc)
                continue

            _duck.set()
            if _BGM_CH is not None and _BGM_CH.get_busy():
                _BGM_CH.set_volume(BGM_DUCK_VOLUME / 100.0)

            with _state_lock:
                _high_item = item
            _clear_last_play_error()
            _write_speaker_info()

            volume_pct = _volumes.get(item.type, VOLUME_TTS)
            sound.set_volume(volume_pct / 100.0)
            ch.set_volume(volume_pct / 100.0)
            _stop_high.clear()
            ch.play(sound, loops=0, fade_ms=item.fade_ms)
            log.info(
                "[speaker_manager] HIGH pygame 재생 시작 → type=%s file=%s vol=%d%% fade_ms=%d",
                item.type, item.file, volume_pct, item.fade_ms,
            )

            while not _stop.is_set():
                if _stop_high.is_set():
                    _stop_high.clear()
                    ch.stop()
                    log.info("[speaker_manager] HIGH 중단 요청")
                    break
                if not ch.get_busy():
                    break
                _stop.wait(0.05)

            _duck.clear()
            if _BGM_CH is not None and _BGM_CH.get_busy():
                _BGM_CH.set_volume(_volumes.get("bgm", VOLUME_BGM) / 100.0)

            with _state_lock:
                _high_item = None
            _write_speaker_info()
            continue

        _duck.set()   # BGM 덕킹 시작

        vol_pa    = _pct_to_pa(_volumes.get(item.type, VOLUME_TTS))
        pulse_env = _pulse_env()

        proc = subprocess.Popen(
            ["paplay", "--device", sink, "--volume", str(vol_pa), item.file],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            env=pulse_env,
        )
        with _state_lock:
            _high_proc = proc
            _high_item = item
        _clear_last_play_error()
        _write_speaker_info()

        log.info(
            "[speaker_manager] HIGH 재생 시작 → type=%s file=%s vol=%d%%",
            item.type, item.file, _volumes.get(item.type, VOLUME_TTS),
        )

        while not _stop.is_set():
            if _stop_high.is_set():
                _stop_high.clear()
                log.info("[speaker_manager] HIGH 중단 요청")
                break

            rc = proc.poll()
            if rc is not None:
                if rc != 0:
                    err = proc.stderr.read().decode(errors="replace").strip()
                    _set_last_play_error(f"high paplay failed rc={rc}: {err or item.file}")
                    _write_speaker_info()
                    log.warning("[speaker_manager] HIGH paplay 종료 rc=%d stderr: %s", rc, err)
                break

            time.sleep(0.1)

        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                proc.kill()

        _duck.clear()   # BGM 볼륨 복원 트리거 (_bgm_thread가 감지)

        with _state_lock:
            _high_proc = None
            _high_item = None
        _write_speaker_info()

    log.info("[speaker_manager] HIGH 스레드 종료")


# ── HTTP Status API ────────────────────────────────────────────────────

def _start_api() -> None:
    """HTTP API — 포트 8083."""
    try:
        import uvicorn
        from fastapi import FastAPI
        from fastapi.middleware.cors import CORSMiddleware
    except ImportError as e:
        log.error("[speaker_manager] API 비활성: %s", e)
        return

    app = FastAPI(
        title="Speaker Manager API",
        version="2.0.0",
        description=(
            "Bluetooth speaker management API with pygame Channel backend and paplay fallback.\n\n"
            "- BGM/TTS/Alert playback queues\n"
            "- Audio ducking while TTS/Alert is playing\n"
            "- Fade-in support through POST /play fade_ms"
        ),
    )
    app.add_middleware(
        CORSMiddleware,
        allow_origins=SPEAKER_CORS_ALLOW_ORIGINS,
        allow_credentials=False,
        allow_methods=["*"],
        allow_headers=["*"],
    )
    log.info("[speaker_manager] CORS 허용 origin=%s", SPEAKER_CORS_ALLOW_ORIGINS)

    @app.get("/health")
    def health():
        return {"ok": True}

    @app.get("/status")
    def status():
        with _state_lock:
            bgm  = _bgm_item
            high = _high_item
            last_play_error = _last_play_error
        return {
            "process": {
                "pid":        os.getpid(),
                "uptime_sec": round(time.time() - _start_time, 1),
            },
            "bt_connected": _bt_connected,
            "active":       _bt_connected,
            "playing_bgm": {
                "type": bgm.type,
                "file": bgm.file,
                "loop": bgm.loop,
                "fade_ms": bgm.fade_ms,
            } if bgm else None,
            "playing_high": {
                "type": high.type,
                "file": high.file,
                "loop": high.loop,
                "fade_ms": high.fade_ms,
            } if high else None,
            "backend": "pygame" if _pygame_ok else "paplay",
            "use_pygame": USE_PYGAME,
            "ducking":    _duck.is_set(),
            "volumes":    dict(_volumes),
            "queue_bgm":  _bgm_queue.qsize(),
            "queue_high": _high_queue.qsize(),
            "last_play_error": last_play_error,
        }

    @app.post("/play")
    def play(req: PlayRequest):
        file_error = _validate_play_file(req.file)
        if file_error:
            _set_last_play_error(file_error)
            _write_speaker_info()
            log.warning("[speaker_manager] play reject: type=%s file=%s reason=%s", req.type, req.file, file_error)
            return {
                "ok": False,
                "queued": None,
                "queue": None,
                "fade_ms": req.fade_ms,
                "replaced": 0,
                "error": file_error,
            }

        item = PlayItem(file=req.file, type=req.type, loop=req.loop, fade_ms=req.fade_ms)
        if req.type == "bgm":
            replaced = _drain_queue(_bgm_queue)
            _stop_bgm.set()
            _bgm_queue.put(item)
            target_queue = "bgm"
        else:
            replaced = 0
            _high_queue.put(item)
            target_queue = "high"
        _clear_last_play_error()
        _write_speaker_info()
        log.info(
            "[speaker_manager] queued: type=%s file=%s loop=%s fade_ms=%d queue=%s replaced=%d",
            req.type, req.file, req.loop, req.fade_ms, target_queue, replaced,
        )
        return {
            "ok": True,
            "queued": req.file,
            "queue": target_queue,
            "fade_ms": req.fade_ms,
            "replaced": replaced,
        }

    @app.post("/stop")
    def stop(req: StopRequest):
        stopped_bgm  = 0
        stopped_high = 0

        if req.type in ("all", "bgm"):
            _stop_bgm.set()
            stopped_bgm += _drain_queue(_bgm_queue)

        if req.type in ("all", "tts", "alert"):
            _stop_high.set()
            stopped_high += _drain_queue(_high_queue)

        _write_speaker_info()
        return {
            "ok":           True,
            "stopped_bgm":  stopped_bgm,
            "stopped_high": stopped_high,
        }

    @app.post("/volume")
    def volume(req: VolumeRequest):
        _volumes[req.type] = req.volume
        applied = False
        if _pygame_ok:
            applied = _set_pygame_channel_volume(req.type, req.volume)
        else:
            with _state_lock:
                bgm_proc  = _bgm_proc
                high_proc = _high_proc
            if req.type == "bgm" and bgm_proc and not _duck.is_set():
                applied = _set_stream_volume(bgm_proc, req.volume)
            elif req.type in ("tts", "alert") and high_proc:
                applied = _set_stream_volume(high_proc, req.volume)
        log.info(
            "[speaker_manager] 볼륨 변경: type=%s volume=%d%% applied_now=%s",
            req.type, req.volume, applied,
        )
        _write_speaker_info()
        return {"ok": True, "type": req.type, "volume": req.volume, "applied_now": applied}

    log.info("[speaker_manager] API 시작 — http://0.0.0.0:%d/status", API_PORT)
    uvicorn.run(app, host="0.0.0.0", port=API_PORT, log_level="error")


# ── Main ────────────────────────────────────────────────────────────────

def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(name)s %(levelname)s %(message)s",
    )

    os.makedirs(SPEAKER_DATA_DIR, exist_ok=True)
    _init_pygame()
    _write_speaker_info()

    def _sig(_s, _f) -> None:
        _stop.set()

    signal.signal(signal.SIGTERM, _sig)
    signal.signal(signal.SIGINT,  _sig)

    log.info("[speaker_manager] 시작 — data_dir=%s  gpiod_v2=%s",
             SPEAKER_DATA_DIR, _GPIOD_V2)

    threads = [
        threading.Thread(target=_gpio_startup_thread, name="spk-gpio",   daemon=True),
        threading.Thread(target=_pygame_retry_thread, name="spk-pg-init", daemon=True),
        threading.Thread(target=_bt_monitor_thread,   name="spk-bt-mon", daemon=True),
        threading.Thread(target=_bgm_thread,          name="spk-bgm",    daemon=True),
        threading.Thread(target=_high_thread,         name="spk-high",   daemon=True),
        threading.Thread(target=_start_api,           name="spk-api",    daemon=True),
    ]
    for t in threads:
        t.start()

    _stop.wait()

    for t in threads:
        t.join(timeout=3.0)
    log.info("[speaker_manager] 종료")


if __name__ == "__main__":
    main()
