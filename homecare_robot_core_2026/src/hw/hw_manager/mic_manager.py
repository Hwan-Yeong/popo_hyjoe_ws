#!/usr/bin/env python3
"""
mic_manager.py - USB 마이크 관리 SW

AI 연동:
  - filtered FIFO -> 실시간 PCM 스트림 (WakeWord/STT용, 기존 계약 유지)
  - raw FIFO      -> 리샘플-only PCM 스트림 (테스트/디버그용)
  - mic_info.json -> FIFO/포맷/상태 계약 파일
  - HTTP API      -> 상태 조회 + 제어 + 녹음

Fallback:
  - sounddevice/scipy/noisereduce 미설치 또는 입력 스트림 오픈 실패 시 stub 모드
  - filtered FIFO에는 침묵(0x00) 데이터를 계속 공급
"""
from __future__ import annotations

import json
import logging
import os
import queue
import signal
import stat
import subprocess
import threading
import time
import wave
from pathlib import Path
from typing import Optional, Union

log = logging.getLogger(__name__)

try:
    import fcntl as _fcntl
except Exception:
    _fcntl = None

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
_MIC_CFG = _CFG.get("mic", {})

# ── 설정 상수 ──────────────────────────────────────────────────────────
HW_DATA_DIR        = _COMMON_CFG.get("hw_data_dir", "/dev/shm/hw_data")
MIC_DATA_DIR       = os.path.join(HW_DATA_DIR, "mic")
MIC_INFO_PATH      = os.path.join(MIC_DATA_DIR, "mic_info.json")
MIC_FIFO_PATH      = os.path.join(MIC_DATA_DIR, "mic.fifo")
MIC_RAW_FIFO_PATH  = os.path.join(MIC_DATA_DIR, "mic_raw.fifo")
MIC_LOCK_PATH      = os.path.join(MIC_DATA_DIR, "mic_manager.lock")
RECORDINGS_DIR     = os.path.join(MIC_DATA_DIR, "recordings")
API_PORT           = _MIC_CFG.get("api_port", 8082)

_DEFAULT_MIC_DEVICE_KEYWORDS = ("ab13x", "lav", "xs", "sennheiser", "usbc")
_keywords_cfg = _MIC_CFG.get("device_keywords", _DEFAULT_MIC_DEVICE_KEYWORDS)
if isinstance(_keywords_cfg, str):
    _keywords_cfg = [_keywords_cfg]
MIC_DEVICE_KEYWORDS = tuple(str(k).lower() for k in _keywords_cfg if str(k).strip())
ALLOW_DEFAULT_INPUT_FALLBACK = _MIC_CFG.get("allow_default_input_fallback", False)
PREFERRED_ALSA_DEVICE = str(_MIC_CFG.get("preferred_alsa_device", "")).strip() or None
STARTUP_DEVICE_WAIT_SEC = float(_MIC_CFG.get("startup_device_wait_sec", 30.0))
DEVICE_DIAGNOSTICS_INTERVAL_RETRIES = int(_MIC_CFG.get("device_diagnostics_interval_retries", 10))

SAMPLE_RATE_IN    = _MIC_CFG.get("sample_rate_in", 48_000)
RESAMPLE_DOWN     = _MIC_CFG.get("resample_down", 3)
BLOCK_SIZE_IN     = _MIC_CFG.get("block_size_in", 4_800)
SAMPLE_RATE       = 16_000
CHANNELS          = 1
DTYPE             = "int16"
CHUNK_MS          = 20
CHUNK_FRAMES      = 320
CHUNK_BYTES       = CHUNK_FRAMES * 2

NOISE_PROFILE_SEC  = _MIC_CFG.get("noise_profile_sec", 1.5)
PROP_DECREASE      = _MIC_CFG.get("prop_decrease", 0.92)
TARGET_RMS         = _MIC_CFG.get("target_rms", 0.05)
MAX_GAIN           = _MIC_CFG.get("max_gain", 10.0)
SPEECH_SNR_DB      = _MIC_CFG.get("speech_snr_db", 8.0)
SILENCE_GAIN       = _MIC_CFG.get("silence_gain", 0.05)
HPF_CUTOFF         = _MIC_CFG.get("hpf_cutoff", 80.0)
LPF_CUTOFF         = _MIC_CFG.get("lpf_cutoff", 7000.0)
SPEECH_HOLD_FRAMES = _MIC_CFG.get("speech_hold_frames", 8)
CAPTURE_RETRY_INTERVAL_SEC = _MIC_CFG.get("capture_retry_interval_sec", 3.0)

RAW_QUEUE_MAXSIZE  = 100
CAP_QUEUE_MAXSIZE  = 50   # 5초 분량 (100ms × 50)
_SILENCE_CHUNK     = bytes(CHUNK_BYTES)

# 스레드 간 큐 (module-level, main() 시작 전 사용 가능)
_cap_queue: "queue.Queue" = queue.Queue(maxsize=CAP_QUEUE_MAXSIZE)
_raw_q:     "queue.Queue[bytes]" = queue.Queue(maxsize=RAW_QUEUE_MAXSIZE)

# ── 의존성 로드 (fallback 허용) ───────────────────────────────────────
try:
    import noisereduce as nr
    import numpy as np
    from scipy.signal import butter, sosfilt, sosfilt_zi, resample_poly

    _DSP_IMPORT_ERROR: Optional[Exception] = None
except Exception as exc:  # pragma: no cover - 환경 의존
    nr = None  # type: ignore[assignment]
    np = None  # type: ignore[assignment]
    butter = None  # type: ignore[assignment]
    sosfilt = None  # type: ignore[assignment]
    sosfilt_zi = None  # type: ignore[assignment]
    resample_poly = None  # type: ignore[assignment]
    _DSP_IMPORT_ERROR = exc

try:
    import sounddevice as sd

    _SOUNDDEVICE_IMPORT_ERROR: Optional[Exception] = None
except Exception as exc:  # pragma: no cover - 환경 의존
    sd = None  # type: ignore[assignment]
    _SOUNDDEVICE_IMPORT_ERROR = exc

# ── Pydantic 모델 ─────────────────────────────────────────────────────
try:
    from fastapi import Body, FastAPI
    import uvicorn
    from pydantic import BaseModel, Field, field_validator

    class FilterRequest(BaseModel):
        prop_decrease: Optional[float] = Field(None, description="Noise reduction strength, 0.0-1.0")
        target_rms: Optional[float] = Field(None, description="AGC target RMS, must be > 0")
        speech_snr_db: Optional[float] = Field(None, description="Speech SNR threshold in dB")
        silence_gain: Optional[float] = Field(None, description="Gain applied to non-speech frames, 0.0-1.0")
        lpf_cutoff: Optional[float] = Field(None, description="Low-pass filter cutoff in Hz")

        @field_validator("prop_decrease", "silence_gain")
        @classmethod
        def _check_unit_range(cls, v: Optional[float]) -> Optional[float]:
            if v is not None and not 0.0 <= v <= 1.0:
                raise ValueError("must be between 0.0 and 1.0")
            return v

        @field_validator("target_rms")
        @classmethod
        def _check_positive_rms(cls, v: Optional[float]) -> Optional[float]:
            if v is not None and v <= 0.0:
                raise ValueError("must be > 0")
            return v

        @field_validator("speech_snr_db", "lpf_cutoff")
        @classmethod
        def _check_non_negative(cls, v: Optional[float]) -> Optional[float]:
            if v is not None and v < 0.0:
                raise ValueError("must be >= 0")
            return v


    class RecordRequest(BaseModel):
        out_dir: str = Field(RECORDINGS_DIR, description="Directory where raw/filtered WAV files are written")

except Exception:  # pragma: no cover - API 비활성 fallback
    Body = None  # type: ignore[assignment]
    FastAPI = None  # type: ignore[assignment]
    uvicorn = None  # type: ignore[assignment]
    FilterRequest = None  # type: ignore[assignment]
    RecordRequest = None  # type: ignore[assignment]


# ── 공유 상태 ──────────────────────────────────────────────────────────
_stop         = threading.Event()
_start_time   = time.time()
_hw_ready     = False
_filter_ready = False
_muted        = False
_device_name  = ""
_capture_status = "starting"
_last_capture_error = ""
_capture_retry_count = 0
_last_device_scan: dict[str, object] = {}

_state_lock = threading.Lock()
_rec_lock   = threading.Lock()
_mic_info_lock = threading.Lock()

_pipeline: Optional["FilterPipeline"] = None
_recorder: Optional["WavWriter"] = None
_instance_lock_fp = None

_stats: dict[str, int] = {
    "speech_frames": 0,
    "silence_frames": 0,
    "total_frames": 0,
}
_nr_latency_ms = 0.0


def _set_capture_state(
    status: str,
    error: str = "",
    *,
    increment_retry: bool = False,
    reset_retry: bool = False,
) -> None:
    global _capture_status, _last_capture_error, _capture_retry_count
    with _state_lock:
        _capture_status = status
        _last_capture_error = error
        if reset_retry:
            _capture_retry_count = 0
        if increment_retry:
            _capture_retry_count += 1


class FilterPipeline:
    def __init__(
        self,
        noise_profile_sec: float = NOISE_PROFILE_SEC,
        prop_decrease: float = PROP_DECREASE,
        target_rms: float = TARGET_RMS,
        max_gain: float = MAX_GAIN,
        speech_snr_db: float = SPEECH_SNR_DB,
        silence_gain: float = SILENCE_GAIN,
        lpf_cutoff: float = LPF_CUTOFF,
    ) -> None:
        if np is None or butter is None or sosfilt is None or sosfilt_zi is None:
            raise RuntimeError(f"DSP dependencies unavailable: {_DSP_IMPORT_ERROR}")

        self._prop_decrease = prop_decrease
        self._target_rms = target_rms
        self._max_gain = max_gain
        self._speech_snr_db = speech_snr_db
        self._silence_gain = silence_gain
        self._lpf_cutoff = lpf_cutoff

        self._sos_hpf = butter(N=4, Wn=HPF_CUTOFF, btype="high", fs=SAMPLE_RATE, output="sos")
        self._zi_hpf = None

        if lpf_cutoff > 0:
            self._sos_lpf = butter(N=4, Wn=lpf_cutoff, btype="low", fs=SAMPLE_RATE, output="sos")
            self._zi_lpf = None
        else:
            self._sos_lpf = None
            self._zi_lpf = None

        self._noise_chunks: list["np.ndarray"] = []
        self._noise_profile: Optional["np.ndarray"] = None
        self._noise_rms = 0.0
        self._speech_threshold = 0.0
        self._profile_target = int(noise_profile_sec * SAMPLE_RATE)
        self._collected = 0
        self._speech_hold = 0
        self._current_gain = 1.0

    @property
    def profile_ready(self) -> bool:
        return self._noise_profile is not None

    @property
    def current_params(self) -> dict:
        return {
            "prop_decrease": self._prop_decrease,
            "target_rms": self._target_rms,
            "max_gain": self._max_gain,
            "speech_snr_db": self._speech_snr_db,
            "silence_gain": self._silence_gain,
            "lpf_cutoff": self._lpf_cutoff if self._sos_lpf is not None else 0.0,
        }

    def update_params(
        self,
        prop_decrease: Optional[float] = None,
        target_rms: Optional[float] = None,
        speech_snr_db: Optional[float] = None,
        silence_gain: Optional[float] = None,
        lpf_cutoff: Optional[float] = None,
    ) -> None:
        if prop_decrease is not None:
            self._prop_decrease = prop_decrease
        if target_rms is not None:
            self._target_rms = target_rms
        if speech_snr_db is not None:
            self._speech_snr_db = speech_snr_db
            if self._noise_rms > 0:
                self._speech_threshold = self._noise_rms * (10 ** (speech_snr_db / 20))
        if silence_gain is not None:
            self._silence_gain = silence_gain
        if lpf_cutoff is not None:
            self._lpf_cutoff = lpf_cutoff
            if lpf_cutoff > 0:
                self._sos_lpf = butter(
                    N=4,
                    Wn=lpf_cutoff,
                    btype="low",
                    fs=SAMPLE_RATE,
                    output="sos",
                )
                self._zi_lpf = None
            else:
                self._sos_lpf = None
                self._zi_lpf = None

    def process(self, chunk_48k: "np.ndarray") -> tuple["np.ndarray", Optional["np.ndarray"]]:
        raw_16k = self._resample(chunk_48k)
        chunk = self._highpass(raw_16k)
        chunk = self._lowpass(chunk)

        if not self.profile_ready:
            self._noise_chunks.append(chunk.copy())
            self._collected += len(chunk)
            if self._collected >= self._profile_target:
                self._build_noise_profile()
            return raw_16k, None

        is_speech = self._detect_speech(chunk)
        chunk = self._noise_reduce(chunk)
        chunk = self._agc(chunk, is_speech)
        return raw_16k, chunk

    def _resample(self, chunk: "np.ndarray") -> "np.ndarray":
        return resample_poly(chunk, up=1, down=RESAMPLE_DOWN).astype(np.float32)

    def _highpass(self, chunk: "np.ndarray") -> "np.ndarray":
        if self._zi_hpf is None:
            self._zi_hpf = sosfilt_zi(self._sos_hpf) * chunk[0]
        filtered, self._zi_hpf = sosfilt(self._sos_hpf, chunk, zi=self._zi_hpf)
        return filtered.astype(np.float32)

    def _lowpass(self, chunk: "np.ndarray") -> "np.ndarray":
        if self._sos_lpf is None:
            return chunk
        if self._zi_lpf is None:
            self._zi_lpf = sosfilt_zi(self._sos_lpf) * chunk[0]
        filtered, self._zi_lpf = sosfilt(self._sos_lpf, chunk, zi=self._zi_lpf)
        return filtered.astype(np.float32)

    def _build_noise_profile(self) -> None:
        self._noise_profile = np.concatenate(self._noise_chunks).astype(np.float32)
        self._noise_rms = float(np.sqrt(np.mean(self._noise_profile ** 2)))
        self._speech_threshold = self._noise_rms * (10 ** (self._speech_snr_db / 20))
        log.info(
            "[mic_manager] 노이즈 프로파일 완료 %.1fs noise_rms=%.6f speech_threshold=%.6f",
            self._collected / SAMPLE_RATE,
            self._noise_rms,
            self._speech_threshold,
        )

    def _detect_speech(self, chunk: "np.ndarray") -> bool:
        rms = float(np.sqrt(np.mean(chunk ** 2)))
        if rms > self._speech_threshold:
            self._speech_hold = SPEECH_HOLD_FRAMES
            return True
        if self._speech_hold > 0:
            self._speech_hold -= 1
            return True
        return False

    def _noise_reduce(self, chunk: "np.ndarray") -> "np.ndarray":
        reduced = nr.reduce_noise(
            y=chunk,
            sr=SAMPLE_RATE,
            y_noise=self._noise_profile,
            stationary=True,
            prop_decrease=self._prop_decrease,
            n_fft=512,
        )
        return reduced.astype(np.float32)

    def _agc(self, chunk: "np.ndarray", is_speech: bool) -> "np.ndarray":
        rms = float(np.sqrt(np.mean(chunk ** 2)))
        if is_speech and rms > 1e-6:
            target_gain = min(self._target_rms / rms, self._max_gain)
            alpha = 0.15
        else:
            target_gain = self._silence_gain
            alpha = 0.50

        self._current_gain = alpha * target_gain + (1 - alpha) * self._current_gain
        chunk = chunk * self._current_gain
        return np.clip(chunk, -1.0, 1.0).astype(np.float32)


class WavWriter:
    """raw + filtered PCM을 동시 저장하는 테스트용 writer."""

    def __init__(self) -> None:
        self._raw_file: Optional[wave.Wave_write] = None
        self._filtered_file: Optional[wave.Wave_write] = None
        self._raw_path: Optional[str] = None
        self._filtered_path: Optional[str] = None

    def start(self, out_dir: str = RECORDINGS_DIR) -> dict:
        os.makedirs(out_dir, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        self._raw_path = os.path.join(out_dir, f"raw_{ts}.wav")
        self._filtered_path = os.path.join(out_dir, f"filtered_{ts}.wav")

        for path, attr in (
            (self._raw_path, "_raw_file"),
            (self._filtered_path, "_filtered_file"),
        ):
            wf = wave.open(path, "wb")
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(SAMPLE_RATE)
            setattr(self, attr, wf)

        return {"raw": self._raw_path, "filtered": self._filtered_path}

    def write(self, raw_pcm: bytes, filtered_pcm: bytes) -> None:
        if self._raw_file:
            self._raw_file.writeframes(raw_pcm)
        if self._filtered_file:
            self._filtered_file.writeframes(filtered_pcm)

    def stop(self) -> dict:
        paths: dict[str, str] = {}
        for file_attr, path_attr, key in (
            ("_raw_file", "_raw_path", "raw"),
            ("_filtered_file", "_filtered_path", "filtered"),
        ):
            f = getattr(self, file_attr)
            path = getattr(self, path_attr)
            if f:
                f.close()
                setattr(self, file_attr, None)
            if path:
                paths[key] = path
        return paths

    @property
    def is_recording(self) -> bool:
        return self._raw_file is not None


def _ensure_fifo(path: str) -> None:
    """FIFO 존재 보장 — race-safe (여러 스레드 동시 호출 허용)."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    try:
        os.mkfifo(path)
    except FileExistsError:
        # 이미 존재: FIFO인지 확인, 일반 파일이면 제거 후 재생성
        try:
            if not stat.S_ISFIFO(os.stat(path).st_mode):
                os.remove(path)
                os.mkfifo(path)
        except OSError:
            pass  # 다른 스레드가 동시에 처리 중 — 무시


def _pcm_from_float32(chunk: "np.ndarray") -> bytes:
    return (np.clip(chunk, -1.0, 1.0) * 32767).astype(np.int16).tobytes()


DeviceSelector = Union[int, str]


def _run_capture_command(args: list[str], timeout: float = 2.0) -> str:
    try:
        result = subprocess.run(
            args,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=timeout,
            check=False,
        )
        return result.stdout.strip()
    except Exception as exc:
        return f"<{args[0]} failed: {exc}>"


def _read_text_file(path: str) -> str:
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            return f.read().strip()
    except OSError as exc:
        return f"<read failed: {exc}>"


def _sounddevice_input_devices() -> list[dict[str, object]]:
    if sd is None:
        return []
    devices: list[dict[str, object]] = []
    for index, dev in enumerate(sd.query_devices()):
        channels = int(dev.get("max_input_channels", 0))
        if channels <= 0:
            continue
        devices.append({
            "index": index,
            "name": str(dev.get("name", "")),
            "max_input_channels": channels,
        })
    return devices


def _host_audio_snapshot(include_arecord: bool = False) -> dict[str, object]:
    snapshot: dict[str, object] = {
        "sounddevice_inputs": [],
        "proc_asound_cards": _read_text_file("/proc/asound/cards"),
    }
    try:
        snapshot["sounddevice_inputs"] = _sounddevice_input_devices()
    except Exception as exc:
        snapshot["sounddevice_error"] = str(exc)
    if include_arecord:
        snapshot["arecord_l"] = _run_capture_command(["arecord", "-l"])
    return snapshot


def _snapshot_has_target_mic(snapshot: dict[str, object]) -> bool:
    text_parts = [str(snapshot.get("proc_asound_cards", ""))]
    text_parts.extend(str(dev.get("name", "")) for dev in snapshot.get("sounddevice_inputs", []))
    if "arecord_l" in snapshot:
        text_parts.append(str(snapshot.get("arecord_l", "")))
    haystack = "\n".join(text_parts).lower()
    return any(keyword in haystack for keyword in MIC_DEVICE_KEYWORDS)


def _wait_for_startup_mic_ready() -> None:
    if STARTUP_DEVICE_WAIT_SEC <= 0:
        return

    deadline = time.time() + STARTUP_DEVICE_WAIT_SEC
    log.info("[mic_manager] USB mic 준비 대기 시작 max=%.1fs", STARTUP_DEVICE_WAIT_SEC)
    while not _stop.is_set() and time.time() < deadline:
        snapshot = _host_audio_snapshot(include_arecord=False)
        if _snapshot_has_target_mic(snapshot):
            log.info("[mic_manager] USB mic 준비 확인")
            return
        time.sleep(1.0)
    log.warning("[mic_manager] USB mic 준비 대기 timeout - sounddevice 재시도 루프로 전환")


def _find_usb_mic() -> tuple[Optional[DeviceSelector], str]:
    global _last_device_scan

    if sd is None:
        return None, "default"
    try:
        devices = _sounddevice_input_devices()
        _last_device_scan = {
            "timestamp_ms": int(time.time() * 1000),
            "sounddevice_inputs": devices,
            "proc_asound_cards": _read_text_file("/proc/asound/cards"),
        }
        for dev in devices:
            index = int(dev["index"])
            name = str(dev["name"])
            lower = name.lower()
            if any(k in lower for k in MIC_DEVICE_KEYWORDS):
                log.info("[mic_manager] USB mic 발견: [%d] %s", index, name)
                return index, name
    except Exception as exc:
        log.warning("[mic_manager] 장치 목록 조회 실패: %s", exc)
        _last_device_scan = {
            "timestamp_ms": int(time.time() * 1000),
            "sounddevice_error": str(exc),
            "proc_asound_cards": _read_text_file("/proc/asound/cards"),
        }

    if PREFERRED_ALSA_DEVICE and _snapshot_has_target_mic(_last_device_scan):
        log.warning(
            "[mic_manager] sounddevice 목록에서 USB mic 미발견 - ALSA 직접 장치 시도: %s",
            PREFERRED_ALSA_DEVICE,
        )
        return PREFERRED_ALSA_DEVICE, PREFERRED_ALSA_DEVICE

    if ALLOW_DEFAULT_INPUT_FALLBACK:
        log.warning("[mic_manager] USB mic 미발견 - 기본 입력 장치 fallback 사용")
    else:
        log.warning("[mic_manager] USB mic 미발견 - 기본 입력 장치 fallback 비활성")
    return None, "default"


def _write_mic_info(status: str) -> None:
    with _mic_info_lock:
        os.makedirs(MIC_DATA_DIR, exist_ok=True)
        with _state_lock:
            params = _pipeline.current_params if _pipeline else {}
            capture_status = _capture_status
            last_capture_error = _last_capture_error
            capture_retry_count = _capture_retry_count
        with _rec_lock:
            recording = bool(_recorder and _recorder.is_recording)

        info = {
            "status": status,
            "capture_status": capture_status,
            "last_capture_error": last_capture_error,
            "capture_retry_count": capture_retry_count,
            "hw_ready": _hw_ready,
            "filter_ready": _filter_ready,
            "muted": _muted,
            "recording": recording,
            "device_name": _device_name,
            "device_keywords": list(MIC_DEVICE_KEYWORDS),
            "preferred_alsa_device": PREFERRED_ALSA_DEVICE,
            "filtered_fifo_path": MIC_FIFO_PATH,
            "raw_fifo_path": MIC_RAW_FIFO_PATH,
            "fifo_path": MIC_FIFO_PATH,
            "sample_rate": SAMPLE_RATE,
            "channels": CHANNELS,
            "dtype": DTYPE,
            "chunk_ms": CHUNK_MS,
            "chunk_bytes": CHUNK_BYTES,
            "noise_profile_sec": NOISE_PROFILE_SEC,
            "filter_params": params,
            "pipeline": {
                "queue_depth": _cap_queue.qsize(),
                "queue_maxsize": CAP_QUEUE_MAXSIZE,
                "nr_latency_ms": round(_nr_latency_ms, 1),
            },
            "diagnostics": {
                "last_device_scan": _last_device_scan,
            },
            "note": (
                "filtered_fifo_path is the AI contract stream. "
                "raw_fifo_path is resample-only output for testing."
            ),
        }
        tmp = f"{MIC_INFO_PATH}.{os.getpid()}.{threading.get_ident()}.tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(info, f, indent=4)
        os.replace(tmp, MIC_INFO_PATH)


def _acquire_instance_lock() -> bool:
    global _instance_lock_fp
    os.makedirs(MIC_DATA_DIR, exist_ok=True)
    _instance_lock_fp = open(MIC_LOCK_PATH, "w", encoding="utf-8")
    _instance_lock_fp.write(str(os.getpid()))
    _instance_lock_fp.flush()
    if _fcntl is None:
        return True
    try:
        _fcntl.flock(_instance_lock_fp.fileno(), _fcntl.LOCK_EX | _fcntl.LOCK_NB)
        return True
    except OSError:
        return False


def _release_instance_lock() -> None:
    global _instance_lock_fp
    if _instance_lock_fp is None:
        return
    try:
        if _fcntl is not None:
            _fcntl.flock(_instance_lock_fp.fileno(), _fcntl.LOCK_UN)
    except OSError:
        pass
    try:
        _instance_lock_fp.close()
    except OSError:
        pass
    _instance_lock_fp = None


def _run_stub() -> None:
    _set_capture_state("running_stub")
    _write_mic_info("running_stub")
    log.info("[mic_manager] stub 모드 시작 - FIFO: %s", MIC_FIFO_PATH)
    interval = CHUNK_MS / 1000.0

    while not _stop.is_set():
        try:
            with open(MIC_FIFO_PATH, "wb") as fifo:
                while not _stop.is_set():
                    fifo.write(_SILENCE_CHUNK)
                    time.sleep(interval)
        except BrokenPipeError:
            time.sleep(0.5)
        except FileNotFoundError:
            _ensure_fifo(MIC_FIFO_PATH)
            time.sleep(0.2)


def _raw_fifo_writer() -> None:
    """Raw FIFO 전용 쓰기 스레드 — 소비자 없어도 capture/NR 스레드 블록 없음."""
    log.info("[mic_manager] raw FIFO writer 시작")
    while not _stop.is_set():
        try:
            with open(MIC_RAW_FIFO_PATH, "wb") as raw_fifo:
                log.info("[mic_manager] Raw FIFO 소비자 연결됨")
                while not _stop.is_set():
                    try:
                        pcm = _raw_q.get(timeout=0.1)
                    except queue.Empty:
                        continue
                    raw_fifo.write(pcm)
        except BrokenPipeError:
            log.info("[mic_manager] Raw FIFO 소비자 연결 끊김")
            time.sleep(0.2)
        except FileNotFoundError:
            _ensure_fifo(MIC_RAW_FIFO_PATH)
            time.sleep(0.2)
    log.info("[mic_manager] raw FIFO writer 종료")


def _nr_fifo_thread() -> None:
    """NR+AGC 처리 및 Filtered FIFO 쓰기 전담 스레드.

    noisereduce(FFT 기반, CPU-heavy)를 capture thread에서 분리해
    sounddevice 타이밍을 보호하고 타 프로세스(카메라 등) CPU 스타베이션을 방지한다.
    """
    global _filter_ready, _nr_latency_ms

    # pipeline이 초기화될 때까지 대기. 부팅 직후 ALSA 오픈 실패 시 capture thread가 재시도할 수 있다.
    pipeline: Optional[FilterPipeline] = None
    waiting_logged = False
    while not _stop.is_set():
        with _state_lock:
            pipeline = _pipeline
        if pipeline is not None:
            if waiting_logged:
                log.info("[mic_manager] NR thread: pipeline 초기화 확인, 처리 시작")
            break
        if not waiting_logged:
            log.info("[mic_manager] NR thread: pipeline 대기 중")
            waiting_logged = True
        time.sleep(0.2)

    if pipeline is None:
        return

    while not _stop.is_set():
        try:
            with open(MIC_FIFO_PATH, "wb") as fifo:
                log.info("[mic_manager] Filtered FIFO 소비자 연결됨")

                # FIFO 연결 시 _cap_queue에 쌓인 구형 오디오 버림 — burst 처리 방지
                drained = 0
                while not _cap_queue.empty():
                    try:
                        _cap_queue.get_nowait()
                        drained += 1
                    except queue.Empty:
                        break
                if drained:
                    log.info("[mic_manager] FIFO 연결 burst 방지: %d 청크 드레인", drained)

                while not _stop.is_set():
                    try:
                        mono = _cap_queue.get(timeout=0.1)
                    except queue.Empty:
                        continue

                    if mono is None:
                        return  # capture thread 종료 신호

                    # pipeline.process(): resample → HPF/LPF → [NR] → AGC
                    with _state_lock:
                        pipeline = _pipeline  # /filter API로 교체될 수 있음
                    process_t0 = time.perf_counter()
                    raw_16k, filtered_16k = pipeline.process(mono)
                    _nr_latency_ms = (time.perf_counter() - process_t0) * 1000.0
                    #if _nr_latency_ms > CHUNK_MS:
                    #    log.warning(
                    #        "[mic_manager] NR 처리 지연 %.1fms > chunk %dms",
                    #        _nr_latency_ms,
                    #        CHUNK_MS,
                    #    )

                    # ── Raw FIFO 분기 ─────────────────────────────────────
                    for start in range(0, len(raw_16k), CHUNK_FRAMES):
                        seg_r = raw_16k[start:start + CHUNK_FRAMES]
                        if len(seg_r) < CHUNK_FRAMES:
                            break
                        try:
                            _raw_q.put_nowait(_pcm_from_float32(seg_r))
                        except queue.Full:
                            pass

                    if filtered_16k is None:
                        continue  # noise profile 수집 중

                    # ── noise profile 완료 감지 ────────────────────────────
                    if not _filter_ready and pipeline.profile_ready:
                        _filter_ready = True
                        _write_mic_info("running")
                        log.info("[mic_manager] 노이즈 프로파일 완료, 필터 활성화")

                    # ── VAD 통계 ──────────────────────────────────────────
                    rms = float(np.sqrt(np.mean(filtered_16k ** 2)))
                    _stats["total_frames"] += 1
                    if rms > SILENCE_GAIN * 2:
                        _stats["speech_frames"] += 1
                    else:
                        _stats["silence_frames"] += 1

                    # ── Filtered FIFO + WAV 기록 ──────────────────────────
                    raw_ptr = 0
                    for start in range(0, len(filtered_16k), CHUNK_FRAMES):
                        seg_f = filtered_16k[start:start + CHUNK_FRAMES]
                        if len(seg_f) < CHUNK_FRAMES:
                            break

                        pcm_f = _pcm_from_float32(seg_f)
                        out_pcm = _SILENCE_CHUNK if _muted else pcm_f
                        fifo.write(out_pcm)

                        seg_r2 = raw_16k[raw_ptr:raw_ptr + CHUNK_FRAMES]
                        raw_ptr += CHUNK_FRAMES
                        pcm_r2 = (
                            _pcm_from_float32(seg_r2)
                            if len(seg_r2) >= CHUNK_FRAMES
                            else _SILENCE_CHUNK
                        )
                        with _rec_lock:
                            if _recorder and _recorder.is_recording:
                                _recorder.write(pcm_r2, out_pcm)

        except BrokenPipeError:
            log.info("[mic_manager] Filtered FIFO 소비자 연결 끊김 - 재연결 대기")
            time.sleep(0.5)


def _capture_thread() -> None:
    """오디오 캡처 전담 경량 스레드 — sounddevice.read() 후 _cap_queue에만 enqueue."""
    global _hw_ready, _device_name, _pipeline, _filter_ready, _last_device_scan

    _ensure_fifo(MIC_FIFO_PATH)
    _ensure_fifo(MIC_RAW_FIFO_PATH)

    if np is None or _DSP_IMPORT_ERROR is not None:
        _set_capture_state("running_stub", str(_DSP_IMPORT_ERROR))
        log.warning("[mic_manager] DSP 의존성 로드 실패 - stub fallback: %s", _DSP_IMPORT_ERROR)
        _run_stub()
    elif sd is None or _SOUNDDEVICE_IMPORT_ERROR is not None:
        _set_capture_state("running_stub", str(_SOUNDDEVICE_IMPORT_ERROR))
        log.warning(
            "[mic_manager] sounddevice 로드 실패 - stub fallback: %s",
            _SOUNDDEVICE_IMPORT_ERROR,
        )
        _run_stub()
    else:
        _wait_for_startup_mic_ready()
        while not _stop.is_set():
            device_idx, device_name = _find_usb_mic()
            if device_idx is None and not ALLOW_DEFAULT_INPUT_FALLBACK:
                _hw_ready = False
                _filter_ready = False
                _set_capture_state("retrying", "USB mic not found", increment_retry=True)
                with _state_lock:
                    retry_count = _capture_retry_count
                log.warning(
                    "[mic_manager] USB mic 미검출 - 기본 입력 fallback 비활성화, %.1fs 후 재시도",
                    CAPTURE_RETRY_INTERVAL_SEC,
                )
                if DEVICE_DIAGNOSTICS_INTERVAL_RETRIES > 0 and retry_count % DEVICE_DIAGNOSTICS_INTERVAL_RETRIES == 0:
                    _last_device_scan = _host_audio_snapshot(include_arecord=True)
                    log.warning("[mic_manager] 장치 진단 snapshot: %s", _last_device_scan)
                _write_mic_info("retrying")
                if _stop.wait(CAPTURE_RETRY_INTERVAL_SEC):
                    break
                continue
            try:
                _set_capture_state("opening")
                _write_mic_info("starting")
                with sd.InputStream(
                    device=device_idx,
                    samplerate=SAMPLE_RATE_IN,
                    channels=1,
                    dtype="float32",
                    blocksize=BLOCK_SIZE_IN,
                ) as stream:
                    _hw_ready = True
                    _device_name = device_name
                    _filter_ready = False
                    with _state_lock:
                        _pipeline = FilterPipeline()
                    drained = 0
                    while not _cap_queue.empty():
                        try:
                            _cap_queue.get_nowait()
                            drained += 1
                        except queue.Empty:
                            break
                    if drained:
                        log.info("[mic_manager] 캡처 재시작 전 queue 드레인: %d", drained)
                    _set_capture_state("running", reset_retry=True)
                    _write_mic_info("running")
                    log.info("[mic_manager] 캡처 시작 - device=%s", device_name)

                    while not _stop.is_set():
                        chunk_48k, overflowed = stream.read(BLOCK_SIZE_IN)
                        if overflowed:
                            log.warning("[mic_manager] 오디오 버퍼 오버플로")
                        mono = chunk_48k[:, 0]
                        if _cap_queue.full():
                            try:
                                _cap_queue.get_nowait()  # 가장 오래된 청크 drop
                            except queue.Empty:
                                pass
                        try:
                            _cap_queue.put_nowait(mono)
                        except queue.Full:
                            pass  # 동시 경쟁 시 skip
            except Exception as exc:
                _hw_ready = False
                _filter_ready = False
                _set_capture_state("retrying", str(exc), increment_retry=True)
                log.error(
                    "[mic_manager] sounddevice 오류 - 재시도 대기 %.1fs: %s",
                    CAPTURE_RETRY_INTERVAL_SEC,
                    exc,
                )
                _write_mic_info("retrying")
                if _stop.wait(CAPTURE_RETRY_INTERVAL_SEC):
                    break

    _set_capture_state("stopped")
    _cap_queue.put(None)  # _nr_fifo_thread 종료 신호
    _write_mic_info("stopped")
    log.info("[mic_manager] capture thread 종료")


def _start_api() -> None:
    if FastAPI is None or uvicorn is None:
        log.error("[mic_manager] API 비활성 - fastapi/uvicorn 미설치")
        return

    app = FastAPI(
        title="Mic Manager API",
        version="2.0.0",
        description=(
            "XS Lav USB-C microphone capture and filtering API.\n\n"
            "- Filtered FIFO: /dev/shm/hw_data/mic/mic.fifo\n"
            "- Raw FIFO: /dev/shm/hw_data/mic/mic_raw.fifo\n"
            "- Pipeline: HPF/LPF, noisereduce, AGC"
        ),
    )

    @app.get("/health")
    def health():
        return {"ok": True}

    @app.get("/status")
    def status():
        with _state_lock:
            params = _pipeline.current_params if _pipeline else {}
            capture_status = _capture_status
            last_capture_error = _last_capture_error
            capture_retry_count = _capture_retry_count
        with _rec_lock:
            recording = bool(_recorder and _recorder.is_recording)
        total = _stats["total_frames"]
        return {
            "process": {
                "pid": os.getpid(),
                "uptime_sec": round(time.time() - _start_time, 1),
            },
            "mic": {
                "capture_status": capture_status,
                "last_capture_error": last_capture_error,
                "capture_retry_count": capture_retry_count,
                "hw_ready": _hw_ready,
                "device_name": _device_name,
                "device_keywords": list(MIC_DEVICE_KEYWORDS),
                "preferred_alsa_device": PREFERRED_ALSA_DEVICE,
                "filter_ready": _filter_ready,
                "muted": _muted,
                "recording": recording,
                "filtered_fifo_path": MIC_FIFO_PATH,
                "raw_fifo_path": MIC_RAW_FIFO_PATH,
                "fifo_exists": os.path.exists(MIC_FIFO_PATH),
                "sample_rate": SAMPLE_RATE,
                "channels": CHANNELS,
                "dtype": DTYPE,
                "chunk_ms": CHUNK_MS,
                "chunk_bytes": CHUNK_BYTES,
                "noise_profile_sec": NOISE_PROFILE_SEC,
            },
            "pipeline": {
                "queue_depth": _cap_queue.qsize(),
                "queue_maxsize": CAP_QUEUE_MAXSIZE,
                "nr_latency_ms": round(_nr_latency_ms, 1),
            },
            "diagnostics": {
                "last_device_scan": _last_device_scan,
            },
            "control": {
                "capture_always_on": True,
                "preferred_apis": ["/mute", "/unmute", "/record/start", "/record/stop"],
                "deprecated_aliases": ["/start", "/stop"],
            },
            "filter": params,
            "vad_stats": {
                "speech_frames": _stats["speech_frames"],
                "silence_frames": _stats["silence_frames"],
                "total_frames": total,
                "speech_ratio": round(_stats["speech_frames"] / total, 3) if total else 0.0,
            },
        }

    @app.post("/mute")
    def mute():
        global _muted
        _muted = True
        _write_mic_info("running")
        log.info("[mic_manager] 음소거 활성화")
        return {"ok": True, "muted": True}

    @app.post("/unmute")
    def unmute():
        global _muted
        _muted = False
        _write_mic_info("running")
        log.info("[mic_manager] 음소거 해제")
        return {"ok": True, "muted": False}

    @app.post("/start", deprecated=True)
    def start():
        global _muted
        _muted = False
        _write_mic_info("running")
        return {
            "ok": True,
            "muted": False,
            "deprecated": True,
            "note": "capture is always on; use /unmute instead",
        }

    @app.post("/stop", deprecated=True)
    def stop():
        global _muted
        _muted = True
        _write_mic_info("running")
        return {
            "ok": True,
            "muted": True,
            "deprecated": True,
            "note": "capture continues; use /mute instead",
        }

    @app.post("/filter")
    def filter_update(req: FilterRequest):
        with _state_lock:
            pipeline = _pipeline
        if pipeline is None:
            return {"ok": False, "note": "pipeline not ready (hw_ready=false)"}

        pipeline.update_params(
            prop_decrease=req.prop_decrease,
            target_rms=req.target_rms,
            speech_snr_db=req.speech_snr_db,
            silence_gain=req.silence_gain,
            lpf_cutoff=req.lpf_cutoff,
        )
        _write_mic_info("running")
        log.info("[mic_manager] 필터 파라미터 업데이트: %s", req.model_dump(exclude_none=True))
        return {"ok": True, "filter": pipeline.current_params}

    @app.post("/record/start")
    def record_start(req: RecordRequest = Body(default_factory=RecordRequest)):
        with _rec_lock:
            if _recorder is None:
                return {"ok": False, "note": "recorder not initialized"}
            if _recorder.is_recording:
                return {"ok": False, "note": "already recording"}
            paths = _recorder.start(req.out_dir)
        _write_mic_info("running")
        log.info("[mic_manager] 녹음 시작: %s", paths)
        return {"ok": True, "files": paths}

    @app.post("/record/stop")
    def record_stop():
        with _rec_lock:
            if _recorder is None or not _recorder.is_recording:
                return {"ok": False, "note": "not recording"}
            paths = _recorder.stop()
        _write_mic_info("running")
        log.info("[mic_manager] 녹음 중단: %s", paths)
        return {"ok": True, "files": paths}

    log.info("[mic_manager] API 시작 - http://0.0.0.0:%d/status", API_PORT)
    uvicorn.run(app, host="0.0.0.0", port=API_PORT, log_level="error")


def main() -> None:
    global _recorder

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(name)s %(levelname)s %(message)s",
    )

    os.makedirs(MIC_DATA_DIR, exist_ok=True)
    os.makedirs(RECORDINGS_DIR, exist_ok=True)
    if not _acquire_instance_lock():
        log.error("[mic_manager] 다른 인스턴스가 이미 실행 중 - 중복 기동 종료")
        return
    _recorder = WavWriter()
    _write_mic_info("starting")

    def _sig(_s, _f) -> None:
        _stop.set()

    signal.signal(signal.SIGTERM, _sig)
    signal.signal(signal.SIGINT, _sig)

    log.info("[mic_manager] 시작 - data_dir=%s", MIC_DATA_DIR)

    threads = [
        threading.Thread(target=_capture_thread, name="mic-capture", daemon=True),
        threading.Thread(target=_nr_fifo_thread,  name="mic-nr-fifo", daemon=True),
        threading.Thread(target=_raw_fifo_writer,  name="mic-raw-fifo", daemon=True),
        threading.Thread(target=_start_api,        name="mic-api",     daemon=True),
    ]
    for thread in threads:
        thread.start()

    _stop.wait()

    with _rec_lock:
        if _recorder and _recorder.is_recording:
            _recorder.stop()

    for thread in threads:
        thread.join(timeout=3.0)

    for path in (MIC_FIFO_PATH, MIC_RAW_FIFO_PATH):
        try:
            if os.path.exists(path):
                os.remove(path)
        except OSError:
            pass

    _write_mic_info("stopped")
    _release_instance_lock()
    log.info("[mic_manager] 종료")


if __name__ == "__main__":
    main()
