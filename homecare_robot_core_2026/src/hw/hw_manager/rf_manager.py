#!/usr/bin/env python3
"""
rf_manager.py — RF 트랜시버 관리 SW

하드웨어: 씨스콜 SUD-100 계열 USB 시리얼 RF 모듈
장치:     /dev/ttyUSB0 (환경변수 RF_SERIAL_PORT로 오버라이드 가능)

Bell ID 규칙:
  BASE_BELL_ID = "3FA17B18"  ← 이 값 이하는 동작하지 않음 (할당 금지)
  유효 범위:    BASE+1 (3FA17B19) 이상
  pos=N 알림벨: hex(0x3FA17B18 + N) — 예) pos=1 → 3FA17B19

저장 경로: /dev/shm/hw_data/rf/
  └── rf_info.json    ← 마지막 notify 결과 + 시리얼 상태

HTTP API: GET  http://0.0.0.0:8084/health
          GET  http://0.0.0.0:8084/status
          POST http://0.0.0.0:8084/notify  {"bell_id": "3FA17B19"}
"""

import json
import os
import signal
import sys
import threading
import time
from collections import deque

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
_RF_CFG = _CFG.get("rf", {})

# ── 설정 상수 ──────────────────────────────────────────────────────────
RF_SERIAL_PORT = os.environ.get("RF_SERIAL_PORT", _RF_CFG.get("serial_port", "/dev/ttyUSB0"))
RF_BAUD        = _RF_CFG.get("baud_rate", 115200)
SERIAL_RECONNECT_INTERVAL_SEC = _RF_CFG.get("serial_reconnect_interval_sec", 3.0)
BELL_EVENT_HISTORY_SIZE = _RF_CFG.get("bell_event_history_size", 50)

HW_DATA_DIR  = _COMMON_CFG.get("hw_data_dir", "/dev/shm/hw_data")
RF_DATA_DIR  = os.path.join(HW_DATA_DIR, "rf")
RF_INFO_PATH = os.path.join(RF_DATA_DIR, "rf_info.json")
API_PORT     = _RF_CFG.get("api_port", 8084)

# ── Bell ID 관리 상수 ─────────────────────────────────────────────────
BASE_BELL_ID:  str = _RF_CFG.get("base_bell_id", "3FA17B18")   # 문자열 표현
BASE_BELL_INT: int = int(BASE_BELL_ID, 16)   # 기준 ID (이 값 이하는 동작하지 않음)

# ── Pydantic 모델 — 모듈 레벨 정의 필수 (ForwardRef 오류 방지) ────────
# from __future__ import annotations 사용 금지
try:
    from pydantic import BaseModel, Field, field_validator

    class NotifyRequest(BaseModel):
        bell_id: str = Field(..., description="8-char uppercase hex Bell ID, e.g. 3FA17B19")

        @field_validator("bell_id")
        @classmethod
        def _check_bell_id(cls, v: str) -> str:
            v = v.strip().upper()
            # 1. 8자리 hex 형식 검사
            if len(v) != 8:
                raise ValueError(
                    f"bell_id must be 8-char hex string (e.g. 3FA17B19), got '{v}'"
                )
            try:
                val = int(v, 16)
            except ValueError:
                raise ValueError(f"bell_id must be valid hex string, got '{v}'")
            # 2. BASE 이하 값 거부 (0x3FA17B18 이하는 동작하지 않음)
            if val <= BASE_BELL_INT:
                raise ValueError(
                    f"bell_id 0x{v} is at or below BASE (0x{BASE_BELL_ID}). "
                    f"Valid IDs start from 3FA17B19"
                )
            return v

    class PosNotifyRequest(BaseModel):
        pos: int = Field(..., ge=1, description="1-based position number. pos=1 -> bell_id=3FA17B19")

        @field_validator("pos")
        @classmethod
        def _check_pos(cls, v: int) -> int:
            if v < 1:
                raise ValueError(f"pos must be >= 1, got {v}")
            return v

except ImportError:
    NotifyRequest = None  # type: ignore
    PosNotifyRequest = None  # type: ignore


# ── Bell ID 유틸 함수 ─────────────────────────────────────────────────

def bell_id_for_pos(pos: int) -> str:
    """
    위치 번호(1-based)로 Bell ID 문자열 생성.

    Args:
        pos: 1-based 위치 번호 (1 이상)
    Returns:
        "3FA17B19" 형식 대문자 hex 문자열

    Examples:
        bell_id_for_pos(1) → "3FA17B19"
        bell_id_for_pos(2) → "3FA17B1A"
        bell_id_for_pos(3) → "3FA17B1B"
    """
    if pos < 1:
        raise ValueError(f"pos must be >= 1, got {pos}")
    return format(BASE_BELL_INT + pos, "08X")


def is_valid_bell_id(bell_id: str) -> bool:
    """
    Bell ID 유효성 검사.
      - 8자리 hex 문자열
      - int 변환 값이 BASE_BELL_INT 초과 (BASE 이하 동작 안 함)
    """
    try:
        val = int(bell_id, 16)
        return val > BASE_BELL_INT
    except (ValueError, TypeError):
        return False


# ── RF 프로토콜 함수 (씨스콜 SUD-100 프레임 포맷) ─────────────────────

def _checksum(frame: bytes) -> int:
    """프레임 체크섬: (−sum) & 0xFF"""
    return (-sum(frame)) & 0xFF


def _checksum_ok(frame: bytes) -> bool:
    """수신 프레임 체크섬 검증: 전체 바이트 합 & 0xFF == 0"""
    return (sum(frame) & 0xFF) == 0


def _make_ack(subtype: int) -> bytes:
    """ACK 응답 프레임 생성 (TX, msg_type=0xC5)."""
    body = bytes([0x03, 0x01, 0x00, 0x08, 0xC5, subtype, 0x00])
    return body + bytes([_checksum(body)])


def _make_notify(bell_id_hex: str) -> bytes:
    """
    알림벨 울리기 프레임 생성 (TX, msg_type=0xC5, subtype=0x14).

    Args:
        bell_id_hex: 8자리 대문자 hex 문자열 (예: "3FA17B19")
    Returns:
        19-byte 프레임
    """
    bell_bytes = bytes.fromhex(bell_id_hex)
    body = (
        bytes([0x03, 0x01, 0x00, 0x14, 0xC5, 0x14, 0x00, 0xCC, 0xA1, 0x02])
        + bell_bytes
        + bytes([0x0F, 0x01, 0x00, 0x00, 0x00])
    )
    return body + bytes([_checksum(body)])


def _parse_frames(buf: bytearray) -> list:
    """수신 버퍼에서 유효한 프레임 목록 추출 (SOF=0x03 기준)."""
    frames = []
    while True:
        if len(buf) < 4:
            break
        if buf[0] != 0x03:
            buf.pop(0)
            continue
        frame_len = buf[3]
        if frame_len < 5 or frame_len > 64:
            buf.pop(0)
            continue
        if len(buf) < frame_len:
            break
        frame = bytes(buf[:frame_len])
        del buf[:frame_len]
        if _checksum_ok(frame):
            frames.append(frame)
        else:
            print(f"[rf_manager] BADCHK: {frame.hex()}", file=sys.stderr)
    return frames


# ── 공유 상태 ──────────────────────────────────────────────────────────
_stop        = threading.Event()
_rf_lock     = threading.Lock()   # _serial 쓰기 + _last_notify 갱신 보호
_rf_info_lock = threading.Lock()
_start_time  = time.time()

try:
    import serial as _serial_mod
    _serial: "_serial_mod.Serial | None" = None
except ImportError:
    _serial_mod = None  # type: ignore
    _serial = None

_serial_open: bool = False
_last_notify: dict = {
    "bell_id":    None,
    "sent_at_ms": 0,
    "ok":         None,   # None=미송신, True=성공, False=실패
    "note":       "",
}
_bell_events = deque(maxlen=BELL_EVENT_HISTORY_SIZE)


# ── 상태 파일 갱신 (원자적) ────────────────────────────────────────────

def _write_rf_info() -> None:
    """rf_info.json 원자적 갱신 (tmp → os.replace)."""
    with _rf_info_lock:
        os.makedirs(RF_DATA_DIR, exist_ok=True)
        with _rf_lock:
            info = {
                "serial_open": _serial_open,
                "last_notify": dict(_last_notify),
                "bell_events": list(_bell_events),
            }
        tmp = RF_INFO_PATH + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(info, f, indent=4)
        os.replace(tmp, RF_INFO_PATH)


# ── 시리얼 수신 스레드 ──────────────────────────────────────────────────

def _try_open_serial() -> None:
    """시리얼 포트 오픈 시도. 실패 시 _serial_open = False 유지."""
    global _serial, _serial_open
    if _serial_mod is None:
        print("[rf_manager] pyserial 미설치 — stub 모드", file=sys.stderr)
        return
    try:
        s = _serial_mod.Serial(
            port=RF_SERIAL_PORT,
            baudrate=RF_BAUD,
            bytesize=_serial_mod.EIGHTBITS,
            parity=_serial_mod.PARITY_NONE,
            stopbits=_serial_mod.STOPBITS_ONE,
            timeout=0.05,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )
        _serial = s
        _serial_open = True
        print(f"[rf_manager] 시리얼 오픈: {RF_SERIAL_PORT} @ {RF_BAUD}bps")
    except Exception as e:
        _serial_open = False
        print(f"[rf_manager] 시리얼 오픈 실패: {e}", file=sys.stderr)


def _record_bell_event(bell_id: str | None, frame: bytes) -> None:
    received_at = int(time.time() * 1000)
    raw = frame.hex().upper()
    event = {
        "bell_id": bell_id,
        "received_at": received_at,
        "received_at_ms": received_at,
        "raw": raw,
        "frame_hex": raw,
    }
    with _rf_lock:
        _bell_events.append(event)


def _do_notify(bell_id: str) -> dict:
    """NOTIFY 송신 공통 처리. 상태 파일 갱신은 락 밖에서 수행해 데드락을 피한다."""
    global _last_notify
    with _rf_lock:
        if not _serial_open or _serial is None:
            _last_notify = {
                "bell_id":    bell_id,
                "sent_at_ms": int(time.time() * 1000),
                "ok":         False,
                "note":       "serial not open",
            }
            result = dict(_last_notify)
        else:
            try:
                frame = _make_notify(bell_id)
                _serial.write(frame)
                _serial.flush()
                _last_notify = {
                    "bell_id":    bell_id,
                    "sent_at_ms": int(time.time() * 1000),
                    "ok":         True,
                    "note":       "sent",
                }
                print(f"[rf_manager] TX NOTIFY → bell_id={bell_id}")
            except Exception as e:
                _last_notify = {
                    "bell_id":    bell_id,
                    "sent_at_ms": int(time.time() * 1000),
                    "ok":         False,
                    "note":       f"write error: {e}",
                }
                print(f"[rf_manager] NOTIFY 전송 오류: {e}", file=sys.stderr)
            result = dict(_last_notify)

    _write_rf_info()
    return result


def _handle_rx_frame(frame: bytes) -> None:
    """수신 프레임 처리 — heartbeat ACK 자동 응답."""
    global _serial, _serial_open
    if len(frame) < 6:
        return

    msg_type = frame[4]
    subtype  = frame[5]

    if msg_type == 0xA5 and subtype == 0x01:
        # heartbeat → ACK 응답 (시리얼 연결 유지 필수)
        ack = _make_ack(0x01)
        with _rf_lock:
            if _serial_open and _serial:
                try:
                    _serial.write(ack)
                    _serial.flush()
                except Exception as e:
                    print(f"[rf_manager] ACK 전송 오류: {e}", file=sys.stderr)
                    _serial_open = False

    elif msg_type == 0xA5 and subtype == 0x05:
        # 호출벨 버튼 이벤트 → ACK 응답
        bell_id = None
        if len(frame) >= 14:
            bell_id = frame[10:14].hex().upper()
        _record_bell_event(bell_id, frame)
        ack = _make_ack(0x05)
        with _rf_lock:
            if _serial_open and _serial:
                try:
                    _serial.write(ack)
                    _serial.flush()
                except Exception as e:
                    print(f"[rf_manager] ACK 전송 오류: {e}", file=sys.stderr)
                    _serial_open = False
        print(f"[rf_manager] 호출벨 이벤트 bell_id={bell_id}")

    elif msg_type == 0xA5 and subtype == 0x14:
        # NOTIFY 재시도 알림 (알림벨 미응답 — 정상 범위, 로그만)
        result = frame[6] if len(frame) > 6 else None
        print(f"[rf_manager] NOTIFY 재시도 (알림벨 미응답?) result=0x{result:02X}" if result is not None else
              "[rf_manager] NOTIFY 재시도 (알림벨 미응답?)")

    else:
        print(f"[rf_manager] UNKNOWN frame type=0x{msg_type:02X} sub=0x{subtype:02X}")


def _serial_reader_thread() -> None:
    """시리얼 수신 루프 — heartbeat ACK 처리 + NOTIFY 결과 수신."""
    global _serial_open
    buf = bytearray()

    print(f"[rf_manager] 시리얼 리더 스레드 시작 (port={RF_SERIAL_PORT})")
    _try_open_serial()
    _write_rf_info()

    while not _stop.is_set():
        if not _serial_open:
            time.sleep(SERIAL_RECONNECT_INTERVAL_SEC)
            if not _stop.is_set():
                print(f"[rf_manager] 시리얼 재연결 시도...")
                _try_open_serial()
                _write_rf_info()
            continue

        try:
            with _rf_lock:
                ser = _serial
            if ser is None:
                _serial_open = False
                continue

            chunk = ser.read(ser.in_waiting or 1)
            if chunk:
                buf.extend(chunk)
                frames = _parse_frames(buf)
                for frame in frames:
                    _handle_rx_frame(frame)

            time.sleep(0.002)

        except Exception as e:
            print(f"[rf_manager] 시리얼 오류: {e}", file=sys.stderr)
            with _rf_lock:
                _serial_open = False
            buf.clear()

    print("[rf_manager] 시리얼 리더 스레드 종료")


# ── HTTP API ───────────────────────────────────────────────────────────

def _start_api() -> None:
    """HTTP API — 포트 8084 (FastAPI + uvicorn daemon)."""
    try:
        import uvicorn
        from fastapi import FastAPI
    except ImportError as e:
        print(f"[rf_manager] API 비활성: {e}", file=sys.stderr)
        return

    app = FastAPI(
        title="RF Manager API",
        version="2.0.0",
        description=(
            "SUD-100 RF transceiver management API.\n\n"
            "- POST /notify: ring by Bell ID\n"
            "- POST /notify/pos: ring by 1-based position\n"
            "- GET /events: received bell event history"
        ),
    )

    @app.get("/health")
    def health():
        return {"ok": True}

    @app.get("/status")
    def status():
        with _rf_lock:
            last = dict(_last_notify)
            s_open = _serial_open
            event_count = len(_bell_events)
            last_event = _bell_events[-1] if _bell_events else None
        return {
            "process": {
                "pid":        os.getpid(),
                "uptime_sec": round(time.time() - _start_time, 1),
            },
            "serial": {
                "port": RF_SERIAL_PORT,
                "open": s_open,
                "baud": RF_BAUD,
            },
            "last_notify": last,
            "bell_events_count": event_count,
            "bell_event_count": event_count,
            "last_bell_event": last_event,
            "data_dir": RF_DATA_DIR,
        }

    @app.post("/notify")
    def notify(req: NotifyRequest):
        bell_id = req.bell_id   # validator에서 이미 upper + 검증 완료
        result = _do_notify(bell_id)
        return {"ok": result["ok"], "bell_id": bell_id, "note": result["note"]}

    @app.post("/notify/pos")
    def notify_pos(req: PosNotifyRequest):
        bell_id = bell_id_for_pos(req.pos)
        result = _do_notify(bell_id)
        return {"ok": result["ok"], "pos": req.pos, "bell_id": bell_id, "note": result["note"]}

    @app.get("/events")
    def events(limit: int = 20):
        limit = max(1, min(limit, BELL_EVENT_HISTORY_SIZE))
        with _rf_lock:
            all_items = list(_bell_events)
            items = all_items[-limit:]
        return {"ok": True, "total": len(all_items), "count": len(items), "events": items}

    @app.get("/events/last")
    def last_event():
        with _rf_lock:
            event = _bell_events[-1] if _bell_events else None
        return {"ok": event is not None, "event": event}

    print(f"[rf_manager] HTTP API 시작 — http://0.0.0.0:{API_PORT}/status")
    uvicorn.run(app, host="0.0.0.0", port=API_PORT, log_level="error")


# ── Main ────────────────────────────────────────────────────────────────

def main() -> None:
    os.makedirs(RF_DATA_DIR, exist_ok=True)
    _write_rf_info()

    def _sig(_s, _f) -> None:
        _stop.set()

    signal.signal(signal.SIGTERM, _sig)
    signal.signal(signal.SIGINT,  _sig)

    print(f"[rf_manager] 시작 — port={RF_SERIAL_PORT}, data_dir={RF_DATA_DIR}")
    print(f"[rf_manager] BASE_BELL_ID={BASE_BELL_ID} (유효 범위: {bell_id_for_pos(1)} 이상)")

    threads = [
        threading.Thread(target=_serial_reader_thread, name="rf-serial", daemon=False),
        threading.Thread(target=_start_api,            name="rf-api",    daemon=True),
    ]
    for t in threads:
        t.start()

    _stop.wait()

    # 시리얼 포트 정리
    with _rf_lock:
        if _serial is not None:
            try:
                _serial.close()
            except Exception:
                pass

    for t in threads:
        t.join(timeout=3.0)
    print("[rf_manager] 종료")


if __name__ == "__main__":
    main()
