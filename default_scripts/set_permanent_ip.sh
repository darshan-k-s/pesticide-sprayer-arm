#!/usr/bin/env bash

# Permanently configure wired interface static IP (192.168.0.77)
# Usage: sudo ./set_permanent_ip.sh

echo "Configuring permanent static IP..."

# 先设置所有参数，最后再设置method（一次性设置避免错误）
nmcli connection modify 'Wired connection 1' \
    ipv4.addresses '192.168.0.77/24' \
    ipv4.gateway '192.168.0.1' \
    ipv4.dns '8.8.8.8' \
    ipv4.method manual

echo "Reactivating connection..."
# 检查连接是否活动
if nmcli connection show --active | grep -q "Wired connection 1"; then
    nmcli connection down 'Wired connection 1'
    sleep 1
fi
nmcli connection up 'Wired connection 1'

sleep 2
echo ""
echo "Configuration complete!"
echo "Current IP configuration:"
ip addr show eno1 | grep "inet "

echo ""
echo "Testing connection to robot..."
if ping -c 2 -W 2 192.168.0.100 >/dev/null 2>&1; then
    echo " Successfully connected to robot!"
else
    echo " Unable to reach robot, please check the network"
fi

