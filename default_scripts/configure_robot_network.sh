#!/usr/bin/env bash

# Configure wired network interface for robot network (192.168.0.xxx)
# Usage: sudo ./configure_robot_network.sh

echo "Configuring wired network interface for robot network..."

# 修改现有的有线连接，添加静态IP
nmcli connection modify "Wired connection 1" \
    ipv4.addresses 192.168.0.77/24 \
    ipv4.method manual \
    ipv4.gateway 192.168.0.1 \
    ipv4.dns "8.8.8.8,8.8.4.4"

# 激活连接
nmcli connection up "Wired connection 1"

echo "Configuration complete!"
echo "Wired interface IP has been set to: 192.168.0.77"
echo "Please make sure the robot IP is: 192.168.0.100"
echo ""
echo "Check IP configuration: ip addr show eno1"

