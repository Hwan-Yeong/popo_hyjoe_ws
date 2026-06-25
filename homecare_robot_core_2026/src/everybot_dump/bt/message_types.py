"""
UI로 전달하는 메시지 타입 상수 중앙 관리.

nodes.py / tree.py 에서 문자열 하드코딩 없이 이 모듈을 참조한다.
ui_mqtt_service.py의 status_types / event_type_prefixes 라우팅 규칙과 일치해야 한다.
"""

# ── Status (topic_status) — status_types 목록에 등록된 타입 ──────
STATUS_ROBOT          = "robot_status"
STATUS_SHOW_MENU      = "request_show_menu"
STATUS_REG            = "REG_STATUS"
STATUS_BATTERY        = "notify_battery_status"

# ── Events (topic_event) — notify_ prefix ────────────────────────
NOTIFY_SCENARIO_DONE        = "notify_scenario_done"
NOTIFY_EMERGENCY_STOP       = "notify_emergency_stop"
NOTIFY_BATTERY_LOW          = "notify_battery_low"
NOTIFY_CHARGING             = "notify_charging"
NOTIFY_CHARGING_DONE        = "notify_charging_done"
NOTIFY_ANOMALY_DETECTED     = "notify_anomaly_detected"
NOTIFY_MAP_CREATION_STARTED = "notify_map_creation_started"
NOTIFY_MAP_CREATION_DONE    = "notify_map_creation_done"
NOTIFY_WIFI_SETUP_STARTED   = "notify_wifi_setup_started"
NOTIFY_WIFI_SETUP_DONE      = "notify_wifi_setup_done"
NOTIFY_PHOTO                = "notify_photo"
NOTIFY_ANOMALY              = "notify_anomaly"

# ── Events (topic_event) — response_ prefix ──────────────────────
RESPONSE_WAYPOINTS    = "response_waypoints"
RESPONSE_SETTINGS     = "response_settings"
