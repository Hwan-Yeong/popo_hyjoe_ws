# Orange Pi Shared Ethernet Setup

This setup removes the dependency on a MAC-derived interface name such as
`enx00e04c6846d9`. It detects the Realtek RTL8153 USB Ethernet adapter by USB
path and driver, configures Jetson as `192.168.10.1/24`, serves DHCP through
NetworkManager shared mode, and refreshes NAT rules when network events occur.

## Install

```bash
sudo /home/everybot/bt_ws/homecare_robot_core_2026/scripts/orangepi_network/install.sh
```

The current adapter can be used immediately. After a reboot or USB replug, the
adapter should be named `orangepi0`.

## Verify

```bash
systemctl status orangepi-network.service --no-pager
nmcli device status
ip -br addr
ip route
sudo iptables -t nat -S | grep 192.168.10
sudo iptables -S FORWARD | grep -E 'orangepi0|enx'
```

Expected Jetson side:

```text
orangepi0  UP  192.168.10.1/24
```

Expected Orange Pi side:

```text
192.168.10.x/24 via DHCP
default via 192.168.10.1
```
