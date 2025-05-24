#!/bin/sh

IP_ETH="10.0.0.1"
IP_WIFI="172.0.0.1"

if ping -c 1 -W 0.2 $IP_WIFI >> /dev/null; then
    ROBOT_IP=$IP_WIFI
elif ping -c 1 -W 0.2 $IP_ETH >> /dev/null; then
    ROBOT_IP=$IP_ETH
else
    echo "No connection to the robot"
    exit 0
fi
echo "Robot IP: $ROBOT_IP"

echo 1 | sudo tee /proc/sys/net/ipv4/ip_forward

LOCAL_IFACE_ROBOT=$(ip route get $ROBOT_IP | grep -o "dev.*" | awk '{print $2}')
LOCAL_IFACE_INTERNET=$(ip route get 8.8.8.8 | grep -o "dev.*" | awk '{print $2}')
echo "Set up IP forwarding from $LOCAL_IFACE_INTERNET to $LOCAL_IFACE_ROBOT"

echo "\nSetting up iptables rules\n"

# Flush standard and NAT rules
sudo iptables -F

sudo iptables -t nat -A POSTROUTING -o $LOCAL_IFACE_INTERNET -j MASQUERADE
sudo iptables -A FORWARD -i $LOCAL_IFACE_INTERNET -o $LOCAL_IFACE_ROBOT -m state --state RELATED,ESTABLISHED -j ACCEPT
sudo iptables -A FORWARD -i $LOCAL_IFACE_ROBOT -o $LOCAL_IFACE_INTERNET -j ACCEPT

# Note: if you want to see the changes made:
#iptables -L -n -v
#iptables -L -n -v -t nat

IP_IFACE_ROBOT=$(ip route get $ROBOT_IP | grep -o "dev.*" | awk '{print $4}')

cmd="sudo ip route del default; sudo ip route add default via $IP_IFACE_ROBOT; ip route; sudo resolvectl dns wlp1s0 8.8.8.8; sudo resolvectl dns enp2s0 8.8.8.8"

echo "Executing through ssh: $cmd"
ssh -t champi@$ROBOT_IP "$cmd && echo Sharing set up. || echo Sharing set up failed."
