#!/usr/bin/env bash
set -eo pipefail

HOME_DIR="/home/everybot"
WORKING_DIR="${HOME_DIR}/bt_ws/homecare_robot_core_2026"

# ── 환경 설정 ──────────────────────────────────────────────────────────
source /opt/ros/humble/setup.bash
source "${WORKING_DIR}/src/hw/install/setup.bash"
source "${WORKING_DIR}/.venv/bin/activate"

export ROS_DOMAIN_ID=29
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
#FASTRTPS_DEFAULT_PROFILES_FILE="${WORKING_DIR}/src/hw/fastdds_config/fastdds.xml"

# ── PulseAudio 사용자 소켓 경로 (systemd 서비스 환경 보정) ───────────
# systemd 서비스는 user-session 환경변수(XDG_RUNTIME_DIR)를 상속받지 않으므로
# hw_manager 프로세스(speaker_manager 등)가 pactl 호출 시 경로를 알 수 있도록 명시.
export XDG_RUNTIME_DIR="/run/user/$(id -u)"
export PULSE_RUNTIME_PATH="${XDG_RUNTIME_DIR}/pulse"
export PULSE_SERVER="unix:${XDG_RUNTIME_DIR}/pulse/native"

# ── HW 데이터 디렉터리 초기화 ─────────────────────────────────────────
HW_DATA_DIR="/dev/shm/hw_data"
mkdir -p "${HW_DATA_DIR}/camera"
mkdir -p "${HW_DATA_DIR}/mic"
mkdir -p "${HW_DATA_DIR}/speaker"
mkdir -p "${HW_DATA_DIR}/rf"
mkdir -p "${WORKING_DIR}/configs/camera_info"

# ── ROS 로그 초기화 ───────────────────────────────────────────────────
echo "[hw_bringup] remove before log ..."
rm -rf "${HOME_DIR}/.ros/log/*"

# ── 프로세스 기동 ─────────────────────────────────────────────────────
echo "[hw_bringup] starting ..."
pids=()

wait_for_usb_mic() {
    local max_wait="${MIC_STARTUP_WAIT_SEC:-30}"
    local i

    echo "[hw_bringup] waiting for USB mic max=${max_wait}s ..." >&2
    udevadm settle --timeout=10 || true

    for i in $(seq 1 "${max_wait}"); do
        if grep -qiE "USBC|Sennheiser|XS LAV|LAV" /proc/asound/cards 2>/dev/null; then
            echo "[hw_bringup] USB mic ready via /proc/asound/cards" >&2
            return 0
        fi
        if arecord -l 2>/dev/null | grep -qiE "USBC|Sennheiser|XS LAV|LAV"; then
            echo "[hw_bringup] USB mic ready via arecord" >&2
            return 0
        fi
        sleep 1
    done

    echo "[hw_bringup] USB mic wait timeout - mic_manager will retry internally" >&2
    return 0
}

# 1. Inuitive M4.51S RGBD 드라이버 (ROS2)
ros2 launch inusensor_ros2_driver inusensor.launch.py &
pids+=($!)
sleep 2   # ROS2 노드 기동 대기

ros2 run usb_cam usb_cam_node_exe --ros-args \
  -r __ns:=/web_cam \
  -p camera_name:=web_cam \
  -p video_device:="/dev/video0" \
  -p image_width:=640 \
  -p image_height:=480 \
  -p framerate:=15.0 \
  -p io_method:=mmap \
  -p camera_info_url:="file://${WORKING_DIR}/configs/camera_info/web_cam.yaml" &
pids+=($!)
sleep 2   # ROS2 노드 기동 대기

# 2. Camera Recorder (top + middle RGB/Depth) + Status API :8081
python3 "${WORKING_DIR}/src/hw/hw_manager/camera_recorder.py" &
pids+=($!)

# 3. Mic Manager (skeleton) :8082
wait_for_usb_mic
python3 "${WORKING_DIR}/src/hw/hw_manager/mic_manager.py" &
pids+=($!)

# 4. Speaker Manager (skeleton) :8083
python3 "${WORKING_DIR}/src/hw/hw_manager/speaker_manager.py" &
pids+=($!)

# 5. RF Transceiver Manager :8084  (씨스콜 SUD-100, /dev/ttyUSB0)
python3 "${WORKING_DIR}/src/hw/hw_manager/rf_manager.py" &
pids+=($!)

echo "[hw_bringup] started. PIDs: ${pids[*]}"

# ── SIGTERM 트랩 → 자식 프로세스 정리 ────────────────────────────────
_term() {
    echo "[hw_bringup] stopping ..."
    kill -TERM "${pids[@]}" 2>/dev/null || true
    wait || true
    echo "[hw_bringup] stopped."
}
trap _term SIGINT SIGTERM

# 포그라운드 유지 (systemd가 메인 PID 감시)
wait
