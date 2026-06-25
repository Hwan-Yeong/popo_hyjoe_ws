#!/usr/bin/env bash
set -euo pipefail

# Rebuild the NAT rules from the current LAN/WAN devices. The LAN device is
# discovered from USB path/driver instead of a MAC-derived interface name.
source /usr/local/lib/orangepi-network-common.sh

LAN_IF="$(orangepi_find_lan_if)"
WAN_IF="$(orangepi_find_wan_if)"

if [ -z "$LAN_IF" ] || [ -z "$WAN_IF" ]; then
  orangepi_log "skip NAT: LAN_IF='${LAN_IF}' WAN_IF='${WAN_IF}'"
  exit 0
fi

if [ "$LAN_IF" = "$WAN_IF" ]; then
  orangepi_log "skip NAT: LAN and WAN are both '$LAN_IF'"
  exit 0
fi

sysctl -w net.ipv4.ip_forward=1 >/dev/null

iptables -t nat -C POSTROUTING -s "$ORANGEPI_LAN_CIDR" -o "$WAN_IF" -j MASQUERADE 2>/dev/null || \
  iptables -t nat -A POSTROUTING -s "$ORANGEPI_LAN_CIDR" -o "$WAN_IF" -j MASQUERADE

iptables -C FORWARD -i "$LAN_IF" -o "$WAN_IF" -j ACCEPT 2>/dev/null || \
  iptables -A FORWARD -i "$LAN_IF" -o "$WAN_IF" -j ACCEPT

iptables -C FORWARD -i "$WAN_IF" -o "$LAN_IF" -m state --state RELATED,ESTABLISHED -j ACCEPT 2>/dev/null || \
  iptables -A FORWARD -i "$WAN_IF" -o "$LAN_IF" -m state --state RELATED,ESTABLISHED -j ACCEPT

orangepi_log "NAT ready: ${LAN_IF}(${ORANGEPI_LAN_CIDR}) -> ${WAN_IF}"
