# - Gpiod is required for GPIO trigger.
#   If gpiod lib is not install... please install 'libgpiod' and etc.
#
# - This util module use mpv binary for video playing, please install mpv
#
# - This util is not support multi class define. (gpio safety issue)
#   you must define only one this class in main thread for using this util class

from __future__ import annotations
import os
import time
import subprocess
import queue
import threading
from dataclasses import dataclass, field
from typing import Optional, Any, Dict, Callable
from contextlib import contextmanager

import gpiod


# =========================
# Result type (True/False + info)
# =========================
@dataclass
class OpResult:
    ok: bool
    error: str = ""
    data: Dict[str, Any] = field(default_factory=dict)


# =========================
# Configs
# =========================
@dataclass
class BeamGpioConfig:
    chip: str = "gpiochip0"
    line_offset: int = 105            # PIN29(PQ.05) in Jetson nano
    consumer: str = "beam"
    pulse_gap_sec: float = 0.05
    gui_usr_name = "everybot"

@dataclass
class ServoPwmConfig:
    pwmchip: str = "/sys/class/pwm/pwmchip2"
    channel: int = 0                  # PWM0 -> PIN33 in Jetson nano
    period_ns: int = 20_000_000       # 50Hz
    min_us: int = 544
    max_us: int = 2400
    clamp_min_us: int = 544
    clamp_max_us: int = 2400
    center_us: int = 1470
    BEAM_OPEN_ANGLE: int = 70
    BEAM_CLOSE_ANGLE: int = 110


# =========================
# Main util
# =========================
class BeamProjectorUtil:
    """
    [요약]
    - Projector GPIO 트리거 + Servo PWM + 비디오 재생 기능 구현 클래스.
    - 동일 리소스 동시 제어를 락으로 막고, 모든 API는 OpResult(ok/error/data) 반환.
    
    [주요 내장 함수]
    1. 프로젝터 관련
    - projector_on() : 프로젝터 on
    - projector_off() : 프로젝터 off
    - beam_open() : 사전 설정 된 각도로 빔을 Open하도록 Servo 동작
    - beam_close() : 사전 설정 된 각도로 빔을 Close하도록 Servo 동작
    
    2. Servo 관련
    - servo_set_angle(angle_deg: float) :   서보 각도를 설정(angle_deg == -1 이면 PWM OFF), Servo 캘리브레이션에 따라 각도가 달라짐
    - servo_off() : 서보 PWM 출럭을 Off
    
    3. Video 관련
    - video_init() : 화면을 검은배경 화면으로 초기화
    - video_play(file_path: str) : 경로상의 비디오를 재생
    - video_stop() : 재생중인 비디오를 정지(Stop)
    - video_status() : GST 백엔드 플레이어 상태를 반환한다(IDLE/PLAYING/EOS/ERROR 등)
    
    """

    # --- class-level locks shared across all instances in this process
    _locks_guard = threading.Lock()
    _locks: Dict[str, threading.RLock] = {}

    def __init__(
        self,
        gpio_cfg: BeamGpioConfig = BeamGpioConfig(),
        servo_cfg: ServoPwmConfig = ServoPwmConfig(),
        use_file_lock: bool = False,              # (옵션) 프로세스 간 락
        lock_dir: str = "/tmp",
    ):
        self.gpio_cfg = gpio_cfg
        self.servo_cfg = servo_cfg

        self.use_file_lock = use_file_lock
        self.lock_dir = lock_dir

        # gpiod v2(LineRequest) 핸들
        self._gpio_req = None  # type: Optional[Any]
        self._gst_player = _GstVideoPlayer(fullscreen=True, mute=False)
        self.gui_usr_name = self.gpio_cfg.gui_usr_name

        self.last_error: str = ""

    # -------------------------
    # Context manager cleanup
    # -------------------------
    def __enter__(self) -> "BeamProjectorUtil":
        """
        - with 구문 진입 시 유틸 인스턴스를 반환한다.
        """
        return self

    def __exit__(self):
        """
        - 리소스를 정리한다.
        """
        self.video_stop()
        self.servo_off()
        self._gpio_close()

    # =========================
    # Lock helpers
    # =========================
    @classmethod
    def _get_lock(cls, key: str) -> threading.RLock:
        """
        - 프로세스 내부에서 key별 RLock(재진입 가능 락)을 반환한다.
        - key : str, 리소스 식별 문자열(예: "gpio:gpiochip0:105")
        """
        with cls._locks_guard:
            if key not in cls._locks:
                cls._locks[key] = threading.RLock()
            return cls._locks[key]

    @contextmanager
    def _resource_lock(self, key: str):
        """
        - key 리소스에 대해 (옵션) 프로세스 간 flock + 프로세스 내 RLock을 획득한다.
        - key : str, 리소스 식별 문자열 e.g. "gpio:gpiochip0:105", "pwm:/sys/class/pwm/pwmchip2:0", "mpv"
        """
        lock = self._get_lock(key)
        lock.acquire()
        file_fd = None
        try:
            if self.use_file_lock:
                # inter-process lock
                import fcntl
                safe = key.replace("/", "_").replace(":", "_")
                path = os.path.join(self.lock_dir, f"beam_lock_{safe}.lock")
                file_fd = os.open(path, os.O_CREAT | os.O_RDWR, 0o666)
                fcntl.flock(file_fd, fcntl.LOCK_EX)
            yield
        finally:
            if file_fd is not None:
                try:
                    import fcntl
                    fcntl.flock(file_fd, fcntl.LOCK_UN)
                except Exception:
                    pass
                try:
                    os.close(file_fd)
                except Exception:
                    pass
            lock.release()

    # =========================
    # Common safe wrapper
    # =========================
    def _safe(self, fn: Callable[[], OpResult], on_fail: Optional[Callable[[], None]] = None) -> OpResult:
        """
        - 예외를 OpResult(False)로 변환하고, 실패 시 on_fail(정리/롤백)을 시도한다.
        - fn : callable, OpResult를 반환하는 실행 함수
        - on_fail : callable|None, 실패 시 호출할 정리 함수
        """
        try:
            r = fn()
            if not isinstance(r, OpResult):
                return OpResult(False, "Internal: function did not return OpResult")
            if not r.ok and not r.error:
                r.error = "Unknown error"
            self.last_error = "" if r.ok else r.error
            return r
        except Exception as e:
            self.last_error = str(e)
            try:
                if on_fail:
                    on_fail()
            except Exception:
                pass
            return OpResult(False, str(e))

    # ============================================================
    # 1) GPIO (projector pulse)  -- gpiod 2.x (>=2.0) compatible
    # ============================================================
    def _gpio_key(self) -> str:
        """GPIO 락 키 문자열(gpiochip/offset) 생성."""
        return f"gpio:{self.gpio_cfg.chip}:{self.gpio_cfg.line_offset}"

    def _gpio_chip_path(self) -> str:
        """
        - gpiod 2.x는 Chip 생성 시 보통 '/dev/gpiochipX' 경로를 사용한다.
        - chip : str, 설정값("gpiochip0" 또는 "/dev/gpiochip0")
        """
        c = str(self.gpio_cfg.chip)
        if c.startswith("/dev/"):
            return c
        if c.startswith("gpiochip"):
            return "/dev/" + c
        return c

    def _gpio_open(self) -> None:
        """
        - gpiod 2.x 방식으로 LineRequest를 1회 열고, output(INACTIVE=LOW)로 설정한다.
        """
        if self._gpio_req is not None:
            return

        # enums (gpiod 2.x)
        try:
            from gpiod.line import Direction, Value
        except Exception:
            # 환경에 따라 위치가 다를 수 있어 fallback
            Direction = getattr(getattr(gpiod, "line", None), "Direction", None)
            Value = getattr(getattr(gpiod, "line", None), "Value", None)
            if Direction is None or Value is None:
                raise RuntimeError("gpiod 2.x enums not found: from gpiod.line import Direction, Value")

        chip_path = self._gpio_chip_path()
        offset = int(self.gpio_cfg.line_offset)

        # request_lines 예시는 공식/레퍼런스에서 동일 패턴으로 사용됨
        # config: {offset: LineSettings(direction=OUTPUT, output_value=INACTIVE)}
        self._gpio_req = gpiod.request_lines(
            chip_path,
            consumer=self.gpio_cfg.consumer,
            config={
                offset: gpiod.LineSettings(direction=Direction.OUTPUT, output_value=Value.INACTIVE)
            },
        )

    def _gpio_close(self) -> OpResult:
        """
        - GPIO를 LOW(INACTIVE)로 내린 뒤 request release 한다(best-effort).
        """
        def _do():
            with self._resource_lock(self._gpio_key()):
                if self._gpio_req is not None:
                    try:
                        from gpiod.line import Value
                    except Exception:
                        Value = getattr(getattr(gpiod, "line", None), "Value", None)

                    # best-effort LOW
                    try:
                        if Value is not None:
                            self._gpio_req.set_value(int(self.gpio_cfg.line_offset), Value.INACTIVE)
                    except Exception:
                        pass

                    # release request
                    try:
                        self._gpio_req.release()
                    finally:
                        self._gpio_req = None
            return OpResult(True)

        return self._safe(_do)

    def projector_pulse(self, high_sec: float) -> OpResult:
        """
        - GPIO를 high_sec 동안 HIGH(ACTIVE)로 유지한 뒤 LOW(INACTIVE)로 내리는 펄스 신호를 보낸다.
        - high_sec : float, HIGH 유지 시간(초)
        """
        def _do():
            with self._resource_lock(self._gpio_key()):
                self._gpio_open()

                try:
                    from gpiod.line import Value
                except Exception:
                    Value = getattr(getattr(gpiod, "line", None), "Value", None)
                    if Value is None:
                        raise RuntimeError("gpiod Value enum not found")

                offset = int(self.gpio_cfg.line_offset)

                # ensure final LOW even if sleep interrupted
                try:
                    self._gpio_req.set_value(offset, Value.ACTIVE)
                    time.sleep(float(high_sec))
                finally:
                    try:
                        self._gpio_req.set_value(offset, Value.INACTIVE)
                    except Exception:
                        pass

                time.sleep(float(self.gpio_cfg.pulse_gap_sec))

            return OpResult(True, data={"high_sec": float(high_sec)})

        return self._safe(_do)

    def projector_on(self) -> OpResult:
        """프로젝터 ON 트리거(기본: 5초 펄스)."""
        return self.projector_pulse(5.0)

    def projector_off(self) -> OpResult:
        """프로젝터 OFF 트리거(기본: 3초 펄스)."""
        return self.projector_pulse(3.0)


    # ============================================================
    # 2) Servo PWM
    # ============================================================
    def _pwm_key(self) -> str:
        """PWM 락 키 문자열(pwmchip/channel) 생성."""
        return f"pwm:{self.servo_cfg.pwmchip}:{self.servo_cfg.channel}"

    @property
    def _pwm_path(self) -> str:
        """PWM sysfs 경로(/sys/class/pwm/.../pwmX) 반환."""
        return f"{self.servo_cfg.pwmchip}/pwm{self.servo_cfg.channel}"

    def _exists(self, p: str) -> bool:
        """p 경로 파일 존재여부 반환."""
        return os.path.exists(p)

    def _sysfs_write(self, path: str, value) -> None:
        """
        - sysfs 파일에 값을 기록한다(os.open/os.write 사용).
        - path : str, sysfs 파일 경로
        - value : int|str, 기록할 값
        """
        fd = os.open(path, os.O_WRONLY)
        try:
            os.write(fd, (str(value) + "\n").encode())
        finally:
            os.close(fd)

    def _pwm_reset_export(self) -> None:
        """pwm unexport/export를 수행해 pwm 노드를 준비한다."""
        ch = self.servo_cfg.channel
        pwmchip = self.servo_cfg.pwmchip

        try:
            self._sysfs_write(f"{pwmchip}/unexport", ch)
            time.sleep(0.05)
        except Exception:
            pass

        if not self._exists(self._pwm_path):
            self._sysfs_write(f"{pwmchip}/export", ch)
            time.sleep(0.1)

        if not self._exists(self._pwm_path):
            raise RuntimeError(f"{self._pwm_path} does not exist after export")

    def _pwm_enable_50hz_if_needed(self) -> None:
        """period(50Hz) 설정 후 duty 설정, enable=1 순서로 PWM을 출력한다."""
        if not self._exists(self._pwm_path):
            self._pwm_reset_export()

        # order: period -> duty -> enable
        self._sysfs_write(f"{self._pwm_path}/period", self.servo_cfg.period_ns)
        self._sysfs_write(f"{self._pwm_path}/duty_cycle", self.servo_cfg.center_us * 1000)
        self._sysfs_write(f"{self._pwm_path}/enable", 1)

    def _set_pulse_us(self, us: int) -> int:
        """
        - duty_cycle을 펄스폭(us)으로 설정하고 적용된 값을 반환한다(클램프 포함).
        - us : int, 펄스폭 마이크로초(us)
        """
        us = int(us)
        us = max(self.servo_cfg.clamp_min_us, min(self.servo_cfg.clamp_max_us, us))
        self._sysfs_write(f"{self._pwm_path}/duty_cycle", us * 1000)
        return us

    def angle_to_pulse_us(self, angle_deg: float) -> int:
        """
        - 각도(0~180deg)를 펄스폭(us)으로 변환한다.
        - angle_deg : float, 서보 목표 각도(도)
        """
        a = max(0.0, min(180.0, float(angle_deg)))
        span = (self.servo_cfg.max_us - self.servo_cfg.min_us)
        return int(self.servo_cfg.min_us + (a / 180.0) * span)

    def servo_set_angle(self, angle_deg: float) -> OpResult:
        """
        - 서보 각도를 설정한다(angle_deg == -1 이면 PWM OFF).
        - angle_deg : float, 목표 각도(도) 또는 -1(OFF)
        """
        def _do():
            with self._resource_lock(self._pwm_key()):
                if angle_deg == -1:
                    return self.servo_off()

                self._pwm_enable_50hz_if_needed()
                pulse = self.angle_to_pulse_us(angle_deg)
                applied = self._set_pulse_us(pulse)
                return OpResult(True, data={"angle_deg": angle_deg, "pulse_us": applied})

        # 실패 시 PWM disable 시도(서보 떨림 방지)
        return self._safe(_do, on_fail=lambda: self.servo_off())

    def beam_open(self) -> OpResult:
        """빔 프로젝터의 각도를 Open으로 이동한다"""
        ret = self.servo_set_angle(self.servo_cfg.BEAM_OPEN_ANGLE)
        if ret.ok:
            return OpResult(True)
        
    def beam_close(self) -> OpResult:
        """빔 프로젝터의 각도를 Close로 이동한다"""
        ret = self.servo_set_angle(self.servo_cfg.BEAM_CLOSE_ANGLE)
        if ret.ok:
            return OpResult(True)

    def servo_center(self) -> OpResult:
        """서보를 중앙(center_us) 위치로 이동한다."""
        def _do():
            with self._resource_lock(self._pwm_key()):
                self._pwm_enable_50hz_if_needed()
                applied = self._set_pulse_us(self.servo_cfg.center_us)
                return OpResult(True, data={"pulse_us": applied})
        return self._safe(_do, on_fail=lambda: self.servo_off())
    
    def servo_off(self) -> OpResult:
        """PWM enable=0으로 서보 출력 OFF(best-effort)."""
        def _do():
            with self._resource_lock(self._pwm_key()):
                if self._exists(self._pwm_path):
                    try:
                        self._sysfs_write(f"{self._pwm_path}/enable", 0)
                    except Exception:
                        pass
            return OpResult(True)
        return self._safe(_do)

    # ============================================================
    # 3) Video playback
    # ============================================================
    def _dismiss_multiscreen_overlay(self, gui_user: str = "everybot",
                                    attempts: int = 3,
                                    delay_sec: float = 0.2) -> OpResult:
        """
        - OS 멀티스크린 선택/팝업 화면을 Esc 키로 닫기 시도
        - gui_user : str, XAUTHORITY 보강용 GUI 로그인 사용자
        - attempts : int, Esc 전송 시도 횟수
        - delay_sec : float, 시도 간 대기 시간(초)
        """
        def _do():
            # X11 환경 
            env = os.environ.copy()
            env.setdefault("DISPLAY", ":0")
            cand = f"/home/{gui_user}/.Xauthority"
            if os.path.exists(cand):
                env.setdefault("XAUTHORITY", cand)
                env.setdefault("HOME", f"/home/{gui_user}")

            # 세션 타입에 따라 선택
            session_type = env.get("XDG_SESSION_TYPE", "").lower()

            # Wayland : wtype 
            if session_type == "wayland" and shutil.which("wtype"):
                for _ in range(attempts):
                    subprocess.run(["wtype", "-k", "Escape"],
                                env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=1)
                    time.sleep(delay_sec)
                return OpResult(True, data={"method": "wtype", "attempts": attempts})

            # X11: xdotool
            if shutil.which("xdotool"):
                for _ in range(attempts):
                    subprocess.run(["xdotool", "key", "--clearmodifiers", "Escape"],
                                env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=1)
                    time.sleep(delay_sec)
                return OpResult(True, data={"method": "xdotool", "attempts": attempts})

            # 실패 예외
            return OpResult(False, "No key injection tool found (install xdotool or wtype)")

        import shutil
        return self._safe(_do)
    
    def _apply_gui_env(self, gui_user: str = "everybot") -> None:
        """
        - SSH 환경에서도 X11으로 렌더링되도록 DISPLAY/XAUTHORITY 선언
        - gui_user : str, GUI 로그인 사용자
        """
        os.environ.setdefault("GDK_BACKEND", "x11")
        os.environ.setdefault("DISPLAY", ":0")
        cand = f"/home/{gui_user}/.Xauthority"
        if os.path.exists(cand):
            os.environ.setdefault("XAUTHORITY", cand)
            
    def video_init(self, dismiss_overlay: bool = True) -> OpResult:
        """
        - 비디오 백엔드를 초기화하고, 최초 검은 전체화면(IDLE) 상태로 띄운다.
        - gui_user : str, XAUTHORITY 설정용 GUI 로그인 사용자
        """
        def _do():
            with self._resource_lock("video"):
                self._apply_gui_env(gui_user=self.gui_usr_name)

                dismiss_info = {"ok": True, "error": "", "data": {}}
                if dismiss_overlay:
                    # 멀티스크린 화면 등 쓰잘떼기 없는 오버레이 제거 
                    overlay_result = self._dismiss_multiscreen_overlay(gui_user=self.gui_usr_name, attempts=2, delay_sec=0.15)
                   
                    # 검은 전체화면 창 준비
                    self._gst_player.ensure_ui()

                    # 결과를 status에 첨부
                    dismiss_info = {
                        "ok": (overlay_result.ok),
                        "error": (overlay_result.error if not overlay_result.ok else ""),
                        "data": {"dismiss_overlay": overlay_result.data},
                    }
                else:
                    self._gst_player.ensure_ui()

                st = self._gst_player.get_status()
                st["dismiss_overlay"] = dismiss_info
                return OpResult(True, data=st)

        return self._safe(_do)
            
    def video_play(self, file_path: str) -> OpResult:
        """
        - 비디오 재생을 시작한다.
        - file_path : str, 재생할 파일 경로
        - gui_user : str, XAUTHORITY 지정 사용자
        """
        def _do():
            with self._resource_lock("video"):
                self._apply_gui_env(self.gui_usr_name)
                self._gst_player.play(file_path)
                return OpResult(True, data=self._gst_player.get_status())
        return self._safe(_do)

    def video_stop(self) -> OpResult:
        """
        - 재생을 정지하고 검은 화면(IDLE)로 전환한다.
        """
        def _do():
            with self._resource_lock("video"):
                self._gst_player.stop()
                return OpResult(True, data=self._gst_player.get_status())
        return self._safe(_do)

    def video_status(self) -> OpResult:
        """
        - Gst 플레이어 상태를 반환한다(IDLE/PLAYING/EOS/ERROR 등).
        """
        def _do():
            with self._resource_lock("video"):
                return OpResult(True, data=self._gst_player.get_status())
        return self._safe(_do)



class _GstVideoPlayer:
    def __init__(self, fullscreen: bool = True, mute: bool = False):
        """
        - GTK Fullscreen 창을 유지하면서 GStreamer로 영상 재생/정지/상태를 관리한다.
        - fullscreen : bool, True면 GTK 창을 fullscreen으로 띄운다.
        - mute : bool, True면 볼륨을 0으로 설정한다.
        """
        self.fullscreen = fullscreen
        self.mute = mute

        self._Gst = None
        self._GLib = None
        self._Gtk = None
        self._GstVideo = None
        self._GdkX11 = None

        self._thread: Optional[threading.Thread] = None
        self._ui_ready = threading.Event()
        self._cmd_q: "queue.Queue[tuple[str, Any]]" = queue.Queue()

        self._lock = threading.Lock()
        self._state: str = "STOPPED"      # STOPPED | IDLE | STARTING | PLAYING | EOS | ERROR
        self._error: str = ""
        self._file: str = ""

        # GTK objects (UI thread only)
        self._win = None
        self._da = None
        self._xid: Optional[int] = None

        # Gst objects (UI thread only)
        self._pipeline = None
        self._bus = None
        self._vsink = None

    # -------------------------
    # Public status helpers
    # -------------------------
    def get_status(self) -> Dict[str, Any]:
        """
        - 현재 플레이어 상태/에러/파일 정보를 반환한다.
        """
        with self._lock:
            return {"state": self._state, "error": self._error, "file": self._file}

    def is_playing(self) -> bool:
        """
        - 현재 PLAYING 상태인지 반환한다.
        """
        with self._lock:
            return self._state == "PLAYING"

    # -------------------------
    # Public control API
    # -------------------------
    def ensure_ui(self) -> None:
        """
        - UI/메인루프 스레드를 1회 시작하고 fullscreen 검은 화면을 띄운다.
        """
        if self._thread and self._thread.is_alive():
            return

        self._ui_ready.clear()
        self._thread = threading.Thread(target=self._ui_thread_main, daemon=True)
        self._thread.start()

        # UI가 준비될 때까지 짧게 대기(창 띄우기/명령 처리 준비)
        self._ui_ready.wait(timeout=2.0)

    def play(self, file_path: str) -> None:
        """
        - 영상 재생. 재생 전/후는 검은 화면 유지.
        - file_path : str, 재생할 파일 경로
        """
        self.ensure_ui()
        self._cmd_q.put(("play", file_path))

    def stop(self) -> None:
        """
        - 영상 정지. 정지 후 검은 화면으로 전환.
        """
        self.ensure_ui()
        self._cmd_q.put(("stop", None))

    def shutdown(self) -> None:
        """
        - 플레이어를 종료한다(창 닫음).
        """
        if not (self._thread and self._thread.is_alive()):
            return
        self._cmd_q.put(("shutdown", None))
        self._thread.join(timeout=2.0)

    # ============================================================
    # UI thread internals
    # ============================================================
    def _lazy_init(self) -> None:
        if self._Gst is not None:
            return

        import gi
        gi.require_version("Gst", "1.0")
        gi.require_version("Gtk", "3.0")
        gi.require_version("GstVideo", "1.0")

        from gi.repository import Gst, GLib, Gtk, GstVideo, GdkX11

        Gst.init(None)

        self._Gst = Gst
        self._GLib = GLib
        self._Gtk = Gtk
        self._GstVideo = GstVideo
        self._GdkX11 = GdkX11

    def _set_state(self, state: str, error: str = "", file: str = "") -> None:
        with self._lock:
            self._state = state
            if error:
                self._error = error
            if file != "":
                self._file = file

    def _ui_thread_main(self) -> None:
        """
        - GTK fullscreen 창을 띄우고, 주기적으로 명령 큐를 처리한다.
        """
        try:
            self._lazy_init()
            Gtk = self._Gtk
            GLib = self._GLib

            # GTK window
            win = Gtk.Window()
            win.set_decorated(False)
            if self.fullscreen:
                win.fullscreen()

            da = Gtk.DrawingArea()
            win.add(da)

            # 검은 화면을 항상 그릴 draw 핸들러
            def on_draw(widget, cr):
                # 항상 검은 배경
                cr.set_source_rgb(0.0, 0.0, 0.0)
                alloc = widget.get_allocation()
                cr.rectangle(0, 0, alloc.width, alloc.height)
                cr.fill()
                return False

            da.connect("draw", on_draw)

            # XID 확보
            def on_realize(widget):
                try:
                    gdk_window = widget.get_window()
                    self._xid = gdk_window.get_xid()
                except Exception:
                    self._xid = None

            da.connect("realize", on_realize)

            win.show_all()

            self._win = win
            self._da = da

            self._set_state("IDLE", error="", file="")
            self._ui_ready.set()

            # 명령 큐 폴링
            def poll_cmd():
                try:
                    while True:
                        cmd, arg = self._cmd_q.get_nowait()
                        if cmd == "play":
                            self._cmd_play(arg)
                        elif cmd == "stop":
                            self._cmd_stop()
                        elif cmd == "shutdown":
                            self._cmd_stop()
                            Gtk.main_quit()
                            return False
                except queue.Empty:
                    pass
                return True

            GLib.timeout_add(50, poll_cmd)

            Gtk.main()

        except Exception as e:
            self._set_state("ERROR", error=str(e))
            self._ui_ready.set()

    # -------------------------
    # Gst creation/cleanup
    # -------------------------
    def _make_overlay_sink(self):
        """
        - VideoOverlay 가능한 sink를 생성한다.
        """
        Gst = self._Gst
        for name in ["glimagesink", "xvimagesink", "ximagesink", "autovideosink"]:
            if Gst.ElementFactory.find(name) is None:
                continue
            sink = Gst.ElementFactory.make(name, "vsink")
            if sink:
                return sink
        raise RuntimeError("No available video sink found")

    def _attach_overlay(self) -> None:
        """
        - sink를 GTK DrawingArea(XID)에 붙인다.
        """
        if self._vsink is None or self._xid is None:
            return
        try:
            self._GstVideo.VideoOverlay.set_window_handle(self._vsink, self._xid)
        except Exception:
            pass

    def _bus_cb(self, bus, msg):
        Gst = self._Gst

        t = msg.type
        if t == Gst.MessageType.ERROR:
            err, dbg = msg.parse_error()
            self._set_state("ERROR", error=f"{err} ({dbg})" if dbg else str(err))
            self._cmd_stop(to_idle=True)  # 검은 화면
        elif t == Gst.MessageType.EOS:
            # 재생 종료 후 검은화면 변환
            self._set_state("EOS", error="")
            self._cmd_stop(to_idle=True)  # 검은 화면
        elif t == Gst.MessageType.STATE_CHANGED:
            if msg.src == self._pipeline:
                old, new, pending = msg.parse_state_changed()
                if new == Gst.State.PLAYING:
                    self._set_state("PLAYING", error="")

    def _cmd_play(self, file_path: str) -> None:
        """
        - UI 스레드에서 재생 시작: 기존 파이프라인 종료 후 새로 구성한다.
        """
        self._cmd_stop(to_idle=True)

        abs_path = os.path.abspath(file_path)
        if not os.path.exists(abs_path):
            self._set_state("ERROR", error=f"File not found: {abs_path}")
            if self._da:
                self._da.queue_draw()
            return

        uri = "file://" + abs_path
        self._set_state("STARTING", error="", file=abs_path)

        Gst = self._Gst

        pipeline = Gst.ElementFactory.make("playbin", "player")
        if not pipeline:
            self._set_state("ERROR", error="Failed to create playbin")
            return

        vsink = self._make_overlay_sink()

        # 재생 sync 조절 기능
        try:
            if vsink.find_property("sync") is not None:
                vsink.set_property("sync", True)
        except Exception:
            pass

        pipeline.set_property("uri", uri)
        pipeline.set_property("video-sink", vsink)

        # mute는 volume=0이 가장 간단/안정
        if self.mute:
            try:
                if pipeline.find_property("volume") is not None:
                    pipeline.set_property("volume", 0.0)
            except Exception:
                pass

        self._pipeline = pipeline
        self._vsink = vsink

        # overlay attach
        self._attach_overlay()

        # bus watch
        bus = pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self._bus_cb)
        self._bus = bus

        pipeline.set_state(Gst.State.PLAYING)

        # 재생 시작 직전/직후 검은 화면 draw
        if self._da:
            self._da.queue_draw()

    def _cmd_stop(self, to_idle: bool = True) -> None:
        """
        - UI 스레드에서 재생 정지: 파이프라인 NULL, 화면은 검은색으로 갱신한다.
        - to_idle : bool, True면 상태를 IDLE로 전환(검은 화면)
        """
        Gst = self._Gst

        if self._pipeline is not None:
            try:
                self._pipeline.set_state(Gst.State.NULL)
            except Exception:
                pass

        # bus 정리
        try:
            if self._bus is not None:
                self._bus.remove_signal_watch()
        except Exception:
            pass

        self._pipeline = None
        self._bus = None
        self._vsink = None

        # 검은 화면 redraw
        if self._da is not None:
            self._da.queue_draw()

        if to_idle:
            # EOS/STOPPED를 거쳐도 최종은 IDLE(검은 화면)로 통일
            if self.get_status()["state"] not in ("ERROR",):
                self._set_state("IDLE", error="")
