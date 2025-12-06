#!/usr/bin/env bash

# Robot network connectivity diagnosis and configuration script
# Usage: sudo ./fix_robot_network.sh

set -e

echo "=========================================="
echo "Robot network connectivity diagnosis and configuration"
echo "=========================================="
echo ""

# 检查网线连接
echo "1. Checking Ethernet cable connection..."
if [ -f /sys/class/net/eno1/carrier ]; then
    CARRIER=$(cat /sys/class/net/eno1/carrier)
    if [ "$CARRIER" = "1" ]; then
        echo "    Ethernet cable is connected"
    else
        echo "    Ethernet cable not connected! Please check the cable"
        exit 1
    fi
else
    echo "   ? Unable to detect cable status"
fi
echo ""

# 设置临时IP
echo "2. Setting temporary IP address (192.168.0.77)..."
ip addr flush dev eno1 2>/dev/null || true
ip addr add 192.168.0.77/24 dev eno1
ip link set eno1 up
sleep 2
echo "    IP configured"
echo ""

# 显示当前IP
echo "3. Current network configuration:"
ip addr show eno1 | grep "inet " || echo "   Warning: no IPv4 address detected"
echo ""

# 测试连接
echo "4. Testing connection to robot (192.168.0.100)..."
if ping -c 2 -W 2 192.168.0.100 >/dev/null 2>&1; then
    echo "    Successfully reached the robot!"
    ping -c 3 192.168.0.100
else
    echo "    Unable to ping the robot"
    echo ""
    echo "5. 扫描网络查找设备..."
    if command -v nmap >/dev/null 2>&1; then
        echo "   Scanning 192.168.0.0/24 subnet..."
        nmap -sn 192.168.0.0/24 2>/dev/null | grep -E "Nmap scan|192.168.0" | head -20
    else
        echo "   nmap is not installed, skipping scan"
        echo "   You can install it manually: sudo apt install nmap"
    fi
    echo ""
    echo "6. Checking ARP table:"
    arp -a | grep 192.168.0 || echo "   No devices found in 192.168.0 subnet"
    echo ""
    echo "Possible reasons:"
    echo "  - Robot IP is not 192.168.0.100 (check robot teach pendant)"
    echo "  - Robot is powered off or network is not configured"
    echo "  - A switch/router is required but not connected"
    echo "  - Firewall is blocking the connection"
fi
echo ""

# 询问是否配置永久IP
echo "=========================================="
echo "If you need to configure a permanent static IP, run the following commands:"
echo "  sudo nmcli connection modify 'Wired connection 1' ipv4.method manual"
echo "  sudo nmcli connection modify 'Wired connection 1' ipv4.addresses '192.168.0.77/24'"
echo "  sudo nmcli connection modify 'Wired connection 1' ipv4.gateway '192.168.0.1'"
echo "  sudo nmcli connection down 'Wired connection 1'"
echo "  sudo nmcli connection up 'Wired connection 1'"
echo "=========================================="

