#!/usr/bin/env bash

ORANGEPI_CONFIG_FILE="${ORANGEPI_CONFIG_FILE:-/etc/default/orangepi-network}"
if [ -r "$ORANGEPI_CONFIG_FILE" ]; then
  # shellcheck disable=SC1090
  source "$ORANGEPI_CONFIG_FILE"
fi

ORANGEPI_USB_PATH="${ORANGEPI_USB_PATH:-platform-3610000.usb-usb-0:2:1.0}"
ORANGEPI_USB_DRIVER="${ORANGEPI_USB_DRIVER:-r8152}"
ORANGEPI_LAN_NAME="${ORANGEPI_LAN_NAME:-orangepi0}"
ORANGEPI_LAN_ADDR="${ORANGEPI_LAN_ADDR:-192.168.10.1/24}"
ORANGEPI_LAN_CIDR="${ORANGEPI_LAN_CIDR:-192.168.10.0/24}"
ORANGEPI_NM_CONNECTION="${ORANGEPI_NM_CONNECTION:-orangepi-lan}"
ORANGEPI_WAN_IF="${ORANGEPI_WAN_IF:-}"

orangepi_log() {
  logger -t orangepi-network -- "$*"
  printf '[orangepi-network] %s\n' "$*" >&2
}

orangepi_prop() {
  local iface="$1"
  local key="$2"
  udevadm info -q property -p "/sys/class/net/${iface}" 2>/dev/null \
    | awk -F= -v want="$key" '$1 == want { print $2; exit }'
}

orangepi_has_iface() {
  [ -n "${1:-}" ] && [ -d "/sys/class/net/$1" ]
}

orangepi_iface_has_addr() {
  local iface="$1"
  local addr_no_prefix="${ORANGEPI_LAN_ADDR%%/*}"
  ip -4 addr show dev "$iface" 2>/dev/null | grep -q "inet ${addr_no_prefix}/"
}

orangepi_find_wan_if() {
  if orangepi_has_iface "$ORANGEPI_WAN_IF"; then
    printf '%s\n' "$ORANGEPI_WAN_IF"
    return 0
  fi

  ip route show default 0.0.0.0/0 2>/dev/null \
    | awk '{ for (i = 1; i <= NF; i++) if ($i == "dev") { print $(i + 1); exit } }'
}

orangepi_usb_lan_candidates() {
  local iface props driver bus path carrier score

  for netdev in /sys/class/net/*; do
    iface="${netdev##*/}"
    case "$iface" in
      lo|docker*|l4tbr*|usb*|can*) continue ;;
    esac

    props="$(udevadm info -q property -p "/sys/class/net/${iface}" 2>/dev/null || true)"
    driver="$(printf '%s\n' "$props" | awk -F= '$1 == "ID_NET_DRIVER" { print $2; exit }')"
    bus="$(printf '%s\n' "$props" | awk -F= '$1 == "ID_BUS" { print $2; exit }')"
    path="$(printf '%s\n' "$props" | awk -F= '$1 == "ID_PATH" { print $2; exit }')"

    [ "$bus" = "usb" ] || continue
    [ "$driver" = "$ORANGEPI_USB_DRIVER" ] || continue

    score=10
    [ "$iface" = "$ORANGEPI_LAN_NAME" ] && score=$((score + 50))
    [ "$path" = "$ORANGEPI_USB_PATH" ] && score=$((score + 40))
    orangepi_iface_has_addr "$iface" && score=$((score + 30))
    carrier="$(cat "/sys/class/net/${iface}/carrier" 2>/dev/null || printf '0')"
    [ "$carrier" = "1" ] && score=$((score + 20))

    printf '%03d %s\n' "$score" "$iface"
  done
}

orangepi_find_lan_if() {
  if orangepi_has_iface "${ORANGEPI_LAN_IF:-}"; then
    printf '%s\n' "$ORANGEPI_LAN_IF"
    return 0
  fi

  if orangepi_has_iface "$ORANGEPI_LAN_NAME"; then
    printf '%s\n' "$ORANGEPI_LAN_NAME"
    return 0
  fi

  orangepi_usb_lan_candidates | sort -rn | awk 'NR == 1 { print $2 }'
}
