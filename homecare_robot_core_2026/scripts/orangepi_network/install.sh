#!/usr/bin/env bash
set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
  echo "Run with sudo: sudo $0" >&2
  exit 1
fi

SRC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

install -D -m 0644 "$SRC_DIR/orangepi-network.conf" /etc/default/orangepi-network
install -D -m 0644 "$SRC_DIR/10-orangepi-usb-ethernet.link" /etc/systemd/network/10-orangepi-usb-ethernet.link
install -D -m 0644 "$SRC_DIR/orangepi-network-common.sh" /usr/local/lib/orangepi-network-common.sh
install -D -m 0755 "$SRC_DIR/orangepi-nat.sh" /usr/local/sbin/orangepi-nat.sh
install -D -m 0755 "$SRC_DIR/orangepi-network-setup.sh" /usr/local/sbin/orangepi-network-setup.sh
install -D -m 0755 "$SRC_DIR/90-orangepi-nat" /etc/NetworkManager/dispatcher.d/90-orangepi-nat
install -D -m 0644 "$SRC_DIR/orangepi-network.service" /etc/systemd/system/orangepi-network.service

systemctl daemon-reload
udevadm control --reload
nmcli networking reload || true
systemctl enable --now orangepi-network.service

echo "Orange Pi network setup installed."
echo "Current LAN interface: $(/usr/local/sbin/orangepi-network-setup.sh --print-lan-if || true)"
echo "A reboot or USB replug will apply the stable orangepi0 interface name."
