#!/usr/bin/env python3
"""
camera_recorder.py — 공용 카메라 데이터 저장 SW

카메라 구성:
  - top    (USB Webcam,  상단): ROS2 topic subscribe
  - middle (M4.51S RGBD, 중단): ROS2 topic subscribe (RGB + Depth)

저장 경로: /home/everybot/Everybot/hw_data/camera/
  ├── top.jpg             ← USB Webcam RGB
  ├── middle_rgb.jpg      ← M4.51S RGB
  ├── middle_depth.png    ← M4.51S Depth (16-bit PNG, mm 단위)
  └── cam_info.json       ← 메타데이터 + 타임스탬프

Status API: GET http://0.0.0.0:8081/status
           GET http://0.0.0.0:8081/health
"""

from __future__ import annotations

import json
import os
import signal
import sys
import threading
import time
import asyncio

import numpy as np
import cv2

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
_CAM_CFG = _CFG.get("camera", {})
_TOP_CFG = _CAM_CFG.get("top", {})
_MIDDLE_CFG = _CAM_CFG.get("middle", {})

# ── 설정 상수 ──────────────────────────────────────────────────────────
HW_DATA_DIR   = _COMMON_CFG.get("hw_data_dir", "/dev/shm/hw_data")
CAM_DATA_DIR  = os.path.join(HW_DATA_DIR, "camera")
CAM_INFO_PATH = os.path.join(CAM_DATA_DIR, "cam_info.json")

# Top Camera: USB Webcam (상단, usb_cam ROS2 driver)
TOP_RGB_TOPIC = _TOP_CFG.get("rgb_topic", "/web_cam/image_raw")
TOP_IMAGE_NAME = "top.jpg"
TOP_WIDTH = _TOP_CFG.get("width", 640)
TOP_HEIGHT = _TOP_CFG.get("height", 480)
TOP_FPS = _TOP_CFG.get("fps", 15)
JPEG_QUALITY = _TOP_CFG.get("jpeg_quality", 85)

# Middle Camera: Inuitive M4.51S RGBD (중단)
MIDDLE_RGB_TOPIC   = _MIDDLE_CFG.get("rgb_topic", "/camera/rgb/image_raw")    # inusensor_ros2_driver namespace=camera
MIDDLE_WIDTH = _MIDDLE_CFG.get("width", 640)
MIDDLE_HEIGHT = _MIDDLE_CFG.get("height", 480)
MIDDLE_FPS = _MIDDLE_CFG.get("fps", 15)
MIDDLE_RGB_NAME    = "middle_rgb.jpg"

MIDDLE_DEPTH_TOPIC = _MIDDLE_CFG.get("depth_topic", "/camera/depth/image_raw")
MIDDLE_DEPTH_NAME  = "middle_depth.png"         # 16-bit PNG (uint16, mm 단위)


STALE_THRESHOLD_SEC = _CAM_CFG.get("stale_threshold_sec", 5.0)   # 이 초 이상 미갱신 시 ok: false
API_PORT = _CAM_CFG.get("api_port", 8081)

# ── 공유 상태 ──────────────────────────────────────────────────────────
_stop       = threading.Event()
_cam_lock   = threading.Lock()
_start_time = time.time()
_timestamps: dict[str, int] = {
    "top":           0,
    "middle_rgb":    0,
    "middle_depth":  0,
}
_received_msgs: dict[str, int] = {
    "top": 0,
    "middle_rgb": 0,
    "middle_depth": 0,
}
_last_errors: dict[str, str] = {
    "top": "",
    "middle_rgb": "",
    "middle_depth": "",
}
_latest_jpeg_frames: dict[str, bytes] = {
    TOP_IMAGE_NAME: b"",
    MIDDLE_RGB_NAME: b"",
}
_frame_ids: dict[str, int] = {
    TOP_IMAGE_NAME: 0,
    MIDDLE_RGB_NAME: 0,
}


# ── 유틸 함수 ──────────────────────────────────────────────────────────

def _save_jpg(img_bgr: np.ndarray, filename: str) -> None:
    """BGR 이미지를 CAM_DATA_DIR 내 JPEG로 원자적 저장."""
    path = os.path.join(CAM_DATA_DIR, filename)
    tmp = path + ".tmp"
    ok, encoded = cv2.imencode(".jpg", img_bgr, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
    if not ok:
        raise RuntimeError(f"JPEG encode failed: {filename}")
    jpeg_bytes = encoded.tobytes()
    with open(tmp, "wb") as f:
        f.write(jpeg_bytes)
    os.replace(tmp, path)
    with _cam_lock:
        _latest_jpeg_frames[filename] = jpeg_bytes
        _frame_ids[filename] = _frame_ids.get(filename, 0) + 1


def _save_depth_png(depth_uint16: np.ndarray, filename: str) -> None:
    """16-bit Depth 이미지를 CAM_DATA_DIR 내 PNG로 원자적 저장."""
    path = os.path.join(CAM_DATA_DIR, filename)
    tmp = path + ".tmp"
    ok, encoded = cv2.imencode(".png", depth_uint16)
    if not ok:
        raise RuntimeError(f"PNG encode failed: {filename}")
    with open(tmp, "wb") as f:
        f.write(encoded.tobytes())
    os.replace(tmp, path)


def update_cam_info() -> None:
    """cam_info.json 원자적 갱신 (lock + os.replace).

    타 프로세서가 읽는 계약 파일. 읽기 중 손상 없도록 .tmp → os.replace() 사용.
    lock 범위를 파일 쓰기 전체로 확장 — top/middle 두 스레드가 동시에 호출 시
    동일 .tmp 파일을 두 번 os.replace하는 race condition 방지.
    """
    with _cam_lock:
        info = {
            "camera_top": {
                "type":         "usb_webcam",
                "position":     "top",
                "image_file":   TOP_IMAGE_NAME,
                "topic":        TOP_RGB_TOPIC,
                "width":        TOP_WIDTH,
                "height":       TOP_HEIGHT,
                "fps":          TOP_FPS,
                "encoding":     "jpg_bgr",
                "timestamp_ms": _timestamps["top"],
            },
            "camera_middle": {
                "type":              "rgbd_inuitive_m451s",
                "position":          "middle",
                "rgb_file":          MIDDLE_RGB_NAME,
                "depth_file":        MIDDLE_DEPTH_NAME,
                "rgb_topic":         MIDDLE_RGB_TOPIC,
                "depth_topic":       MIDDLE_DEPTH_TOPIC,
                "depth_encoding":    "16UC1_mm",
                "width":             MIDDLE_WIDTH,
                "height":            MIDDLE_HEIGHT,
                "fps":               MIDDLE_FPS,
                "rgb_timestamp_ms":   _timestamps["middle_rgb"],
                "depth_timestamp_ms": _timestamps["middle_depth"],
            },
        }
        tmp = CAM_INFO_PATH + ".tmp"
        with open(tmp, "w") as f:
            json.dump(info, f, indent=4)
        os.replace(tmp, CAM_INFO_PATH)


def _decode_color_image(bridge, msg) -> np.ndarray:
    """ROS Image를 BGR 이미지로 안전하게 변환한다."""
    try:
        return bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except Exception as first_exc:
        frame = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        encoding = str(getattr(msg, "encoding", "") or "").lower()

        if encoding == "bgr8":
            return frame
        if encoding == "rgb8":
            return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        if encoding in {"mono8", "8uc1"}:
            return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        if encoding in {"yuyv", "yuyv2", "yuv422", "yuv422_yuy2"}:
            return cv2.cvtColor(frame, cv2.COLOR_YUV2BGR_YUY2)
        if encoding in {"uyvy", "uyvy422", "y422"}:
            return cv2.cvtColor(frame, cv2.COLOR_YUV2BGR_UYVY)

        raise RuntimeError(
            f"unsupported top image encoding='{encoding}' after bgr8 conversion failure: {first_exc}"
        ) from first_exc


# ── ROS2 Cameras: Top USB Webcam + Middle RGBD ─────────────────────────

def ros2_camera_thread() -> None:
    """상단 USB Webcam과 중단 RGBD 카메라를 모두 ROS2 topic으로 구독해 저장."""
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
        from cv_bridge import CvBridge
    except ImportError as e:
        print(f"[camera_recorder] ROS2/cv_bridge import 실패: {e}", file=sys.stderr)
        print("[camera_recorder] camera ROS2 thread 종료 (ROS2 비활성 환경)", file=sys.stderr)
        return

    rclpy.init()
    bridge = CvBridge()

    class CameraNode(Node):
        def __init__(self) -> None:
            super().__init__("camera_recorder")
            self.create_subscription(Image, TOP_RGB_TOPIC,      self._top_cb,   qos_profile_sensor_data)
            self.create_subscription(Image, MIDDLE_RGB_TOPIC,   self._rgb_cb,   qos_profile_sensor_data)
            self.create_subscription(Image, MIDDLE_DEPTH_TOPIC, self._depth_cb, qos_profile_sensor_data)
            self.get_logger().info(
                f"subscribed: top={TOP_RGB_TOPIC}, middle_rgb={MIDDLE_RGB_TOPIC}, middle_depth={MIDDLE_DEPTH_TOPIC}"
            )

        def _top_cb(self, msg: Image) -> None:
            try:
                with _cam_lock:
                    _received_msgs["top"] += 1
                frame = _decode_color_image(bridge, msg)
                _save_jpg(frame, TOP_IMAGE_NAME)
                with _cam_lock:
                    _timestamps["top"] = int(time.time() * 1000)
                    _last_errors["top"] = ""
                update_cam_info()
            except Exception as e:
                with _cam_lock:
                    _last_errors["top"] = str(e)
                print(f"[camera_recorder] top RGB 오류: {e}", file=sys.stderr)

        def _rgb_cb(self, msg: Image) -> None:
            try:
                with _cam_lock:
                    _received_msgs["middle_rgb"] += 1
                frame = _decode_color_image(bridge, msg)
                _save_jpg(frame, MIDDLE_RGB_NAME)
                with _cam_lock:
                    _timestamps["middle_rgb"] = int(time.time() * 1000)
                    _last_errors["middle_rgb"] = ""
                update_cam_info()
            except Exception as e:
                with _cam_lock:
                    _last_errors["middle_rgb"] = str(e)
                print(f"[camera_recorder] middle RGB 오류: {e}", file=sys.stderr)

        def _depth_cb(self, msg: Image) -> None:
            try:
                with _cam_lock:
                    _received_msgs["middle_depth"] += 1
                # passthrough → uint16 numpy array (mm 단위)
                depth = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                _save_depth_png(depth, MIDDLE_DEPTH_NAME)
                with _cam_lock:
                    _timestamps["middle_depth"] = int(time.time() * 1000)
                    _last_errors["middle_depth"] = ""
                update_cam_info()
            except Exception as e:
                with _cam_lock:
                    _last_errors["middle_depth"] = str(e)
                print(f"[camera_recorder] middle Depth 오류: {e}", file=sys.stderr)

    node = CameraNode()
    print("[camera_recorder] ROS2 camera 시작 (top + middle subscriber)")
    try:
        while not _stop.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("[camera_recorder] ROS2 camera 종료")


# ── HTTP Status API (FastAPI + uvicorn, daemon thread) ─────────────────

def _start_api() -> None:
    """Status API — FastAPI daemon thread, 포트 8081.

    API 스레드 장애가 캡처 스레드에 영향을 주지 않도록 독립 운영.
    uvicorn/fastapi import 실패 시 경고만 출력하고 종료.
    """
    try:
        import uvicorn
        from fastapi import FastAPI, HTTPException
        from fastapi.responses import Response, StreamingResponse
    except ImportError as e:
        print(f"[camera_recorder] API 비활성: {e}", file=sys.stderr)
        return

    api = FastAPI(
        title="Camera Recorder API",
        version="2.0.0",
        description=(
            "Top USB Webcam + Middle RGBD camera capture and streaming API.\n\n"
            "- GET /stream/top: Top camera MJPEG stream\n"
            "- GET /stream/middle_rgb: Middle RGB MJPEG stream\n"
            "- GET /snapshot/top: Latest top JPEG snapshot\n"
            "- GET /snapshot/middle_rgb: Latest middle RGB JPEG snapshot"
        ),
    )

    def image_path(name: str) -> str:
        return os.path.join(CAM_DATA_DIR, name)

    def require_image(name: str) -> str:
        path = image_path(name)
        if not os.path.exists(path):
            raise HTTPException(status_code=503, detail=f"{name} not ready")
        return path

    def read_snapshot_bytes(name: str) -> bytes:
        with _cam_lock:
            frame = _latest_jpeg_frames.get(name, b"")
        if frame:
            return frame
        path = require_image(name)
        with open(path, "rb") as f:
            data = f.read()
        if not data:
            raise HTTPException(status_code=503, detail=f"{name} not ready")
        return data

    async def mjpeg_stream(name: str, fps: int):
        interval = 1.0 / max(1, fps)
        last_frame_id = -1
        while not _stop.is_set():
            try:
                with _cam_lock:
                    frame = _latest_jpeg_frames.get(name, b"")
                    frame_id = _frame_ids.get(name, 0)
                if not frame or frame_id == last_frame_id:
                    await asyncio.sleep(interval / 2.0)
                    continue
                last_frame_id = frame_id
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n"
                    b"Cache-Control: no-store, no-cache, must-revalidate, max-age=0\r\n"
                    b"Pragma: no-cache\r\n"
                    b"X-Accel-Buffering: no\r\n\r\n"
                    + frame
                    + b"\r\n"
                )
            except Exception as e:
                print(f"[camera_recorder] MJPEG stream 오류({name}): {e}", file=sys.stderr)
            await asyncio.sleep(interval)

    @api.get("/health")
    def health():
        return {"ok": True}

    @api.get("/status")
    def status():
        now_ms = int(time.time() * 1000)
        with _cam_lock:
            ts = dict(_timestamps)
            rx = dict(_received_msgs)
            errs = dict(_last_errors)

        def cam_stat(key: str) -> dict:
            t = ts[key]
            stale = (now_ms - t) / 1000.0 if t > 0 else 9999.0
            base = {
                "ok":             stale < STALE_THRESHOLD_SEC,
                "last_update_ms": t,
                "stale_sec":      round(stale, 2),
                "received_msgs":  rx[key],
                "last_error":     errs[key],
            }
            if key == "top":
                base.update({
                    "type": "usb_webcam",
                    "position": "top",
                    "image_file": TOP_IMAGE_NAME,
                    "image_path": os.path.join(CAM_DATA_DIR, TOP_IMAGE_NAME),
                    "topic": TOP_RGB_TOPIC,
                    "width": TOP_WIDTH,
                    "height": TOP_HEIGHT,
                    "fps": TOP_FPS,
                    "encoding": "jpg_bgr",
                })
            elif key == "middle_rgb":
                base.update({
                    "type": "rgbd_inuitive_m451s",
                    "position": "middle",
                    "image_file": MIDDLE_RGB_NAME,
                    "image_path": os.path.join(CAM_DATA_DIR, MIDDLE_RGB_NAME),
                    "topic": MIDDLE_RGB_TOPIC,
                    "width": MIDDLE_WIDTH,
                    "height": MIDDLE_HEIGHT,
                    "fps": MIDDLE_FPS,
                    "encoding": "jpg_bgr",
                })
            elif key == "middle_depth":
                base.update({
                    "type": "rgbd_inuitive_m451s",
                    "position": "middle",
                    "image_file": MIDDLE_DEPTH_NAME,
                    "image_path": os.path.join(CAM_DATA_DIR, MIDDLE_DEPTH_NAME),
                    "topic": MIDDLE_DEPTH_TOPIC,
                    "encoding": "16UC1_mm",
                })
            return base

        return {
            "process": {
                "pid":        os.getpid(),
                "uptime_sec": round(time.time() - _start_time, 1),
            },
            "cameras": {
                "top":          cam_stat("top"),
                "middle_rgb":   cam_stat("middle_rgb"),
                "middle_depth": cam_stat("middle_depth"),
            },
            "streams": {
                "top_mjpeg": f"http://0.0.0.0:{API_PORT}/stream/top",
                "middle_rgb_mjpeg": f"http://0.0.0.0:{API_PORT}/stream/middle_rgb",
                "snapshots": {
                    "top": f"http://0.0.0.0:{API_PORT}/snapshot/top",
                    "middle_rgb": f"http://0.0.0.0:{API_PORT}/snapshot/middle_rgb",
                },
            },
            "data_dir": CAM_DATA_DIR,
        }

    @api.get("/snapshot/top")
    def snapshot_top():
        return Response(
            content=read_snapshot_bytes(TOP_IMAGE_NAME),
            media_type="image/jpeg",
            headers=stream_headers,
        )

    @api.get("/snapshot/middle_rgb")
    def snapshot_middle_rgb():
        return Response(
            content=read_snapshot_bytes(MIDDLE_RGB_NAME),
            media_type="image/jpeg",
            headers=stream_headers,
        )

    stream_headers = {
        "Cache-Control": "no-store, no-cache, must-revalidate, max-age=0",
        "Pragma": "no-cache",
        "X-Accel-Buffering": "no",
    }

    @api.get("/stream/top")
    async def stream_top(fps: int | None = None):
        stream_fps = TOP_FPS if fps is None else max(1, min(fps, TOP_FPS))
        return StreamingResponse(
            mjpeg_stream(TOP_IMAGE_NAME, stream_fps),
            media_type="multipart/x-mixed-replace; boundary=frame",
            headers=stream_headers,
        )

    @api.get("/stream/middle_rgb")
    async def stream_middle_rgb(fps: int | None = None):
        stream_fps = MIDDLE_FPS if fps is None else max(1, min(fps, MIDDLE_FPS))
        return StreamingResponse(
            mjpeg_stream(MIDDLE_RGB_NAME, stream_fps),
            media_type="multipart/x-mixed-replace; boundary=frame",
            headers=stream_headers,
        )

    print(f"[camera_recorder] Status API 시작 — http://0.0.0.0:{API_PORT}/status")
    uvicorn.run(api, host="0.0.0.0", port=API_PORT, log_level="error")


# ── Main ────────────────────────────────────────────────────────────────

def main() -> None:
    os.makedirs(CAM_DATA_DIR, exist_ok=True)

    def _sig(_s, _f) -> None:
        _stop.set()

    signal.signal(signal.SIGTERM, _sig)
    signal.signal(signal.SIGINT,  _sig)

    print(f"[camera_recorder] 시작 — data_dir={CAM_DATA_DIR}")
    threads = [
        threading.Thread(target=ros2_camera_thread, name="cam-ros2", daemon=True),
        threading.Thread(target=_start_api,         name="cam-api",  daemon=True),
    ]
    for t in threads:
        t.start()

    _stop.wait()

    for t in threads:
        t.join(timeout=3.0)
    print("[camera_recorder] 종료")


if __name__ == "__main__":
    main()
