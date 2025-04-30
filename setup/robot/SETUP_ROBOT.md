# Setup Robot PC

Let's put raw commands / files we use here. We probably won't need to setup it again, so it's not worth to make a script for it.

## Access point, network
```bash
champi@champi:~$ sudo cat /etc/netplan/51-champi.yaml  
[sudo] password for champi:  
# This file is generated from information provided by the datasource.  Changes
# to it will not persist across an instance reboot.  To disable cloud-init's
# network configuration capabilities, write a file
# /etc/cloud/cloud.cfg.d/99-disable-network-config.cfg with the following:
# network: {config: disabled}
network:
   version: 2
   ethernets:
       enp2s0:
           dhcp4: no
           addresses: [10.0.0.1/24]
   wifis:
       wlp1s0:
           dhcp4: no
           addresses: [172.0.0.1/24]

champi@champi:~$ sudo cat /etc/hostapd/hostapd.conf  
interface=wlp1s0
channel=11
driver=nl80211
hw_mode=g
ssid=Champi-AP
wme_enabled=1
macaddr_acl=0
auth_algs=1
ignore_broadcast_ssid=0
wpa=3
wpa_passphrase=circuits
wpa_key_mgmt=WPA-PSK
wpa_pairwise=TKIP
rsn_pairwise=CCMP


champi@champi:~$ sudo cat /etc/dhcp/dhcpd.conf
default-lease-time 43200;
max-lease-time 86400;
authoritative;

subnet 10.0.0.0 netmask 255.255.255.0 {
  option subnet-mask 255.255.255.0;
  range 10.0.0.2 10.0.0.50;
  option routers 10.0.0.1;
  option domain-name-servers 10.0.0.1;
}

subnet 172.0.0.0 netmask 255.255.255.0 {
  option subnet-mask 255.255.255.0;
  option routers 172.0.0.1;
  range 172.0.0.2 172.0.0.50;
}


champi@champi:~$ sudo cat /etc/default/isc-dhcp-server
# Defaults for isc-dhcp-server (sourced by /etc/init.d/isc-dhcp-server)

# Path to dhcpd's config file (default: /etc/dhcp/dhcpd.conf).
#DHCPDv4_CONF=/etc/dhcp/dhcpd.conf
#DHCPDv6_CONF=/etc/dhcp/dhcpd6.conf

# Path to dhcpd's PID file (default: /var/run/dhcpd.pid).
#DHCPDv4_PID=/var/run/dhcpd.pid
#DHCPDv6_PID=/var/run/dhcpd6.pid

# Additional options to start dhcpd with.
#       Don't use options -cf or -pf here; use DHCPD_CONF/ DHCPD_PID instead
#OPTIONS=""

# On what interfaces should the DHCP server (dhcpd) serve DHCP requests?
#       Separate multiple interfaces with spaces, e.g. "eth0 eth1".
INTERFACESv4="enp2s0 wlp1s0"
INTERFACESv6=""
```


## For IP forwarding to work
```bash
champi@champi:~$ sudo resolvectl dns wlp1s0 8.8.8.8
[sudo] password for champi: 
champi@champi:~$ sudo resolvectl dns enp2s0 8.8.8.8
champi@champi:~$ resolvectl 
Global
       Protocols: -LLMNR -mDNS -DNSOverTLS DNSSEC=no/unsupported
resolv.conf mode: stub

Link 2 (enp2s0)
    Current Scopes: DNS
         Protocols: +DefaultRoute +LLMNR -mDNS -DNSOverTLS DNSSEC=no/unsupported
Current DNS Server: 8.8.8.8
       DNS Servers: 8.8.8.8

Link 3 (wlp1s0)
    Current Scopes: DNS
         Protocols: +DefaultRoute +LLMNR -mDNS -DNSOverTLS DNSSEC=no/unsupported
Current DNS Server: 8.8.8.8
       DNS Servers: 8.8.8.8

```


```bash
sudo usermod -a -G dialout $USER

```

```bash
echo "set -g mouse on" >> ~/.tmux.conf
```
... Then log-out log-in.



If this gives an output like this
champi@circuits:~$ sudo nft list ruleset
table ip nm-shared-wlp1s0 {
chain nat_postrouting {
type nat hook postrouting priority srcnat; policy accept;
ip saddr 172.0.0.0/24 ip daddr != 172.0.0.0/24 masquerade
}

        chain filter_forward {
                type filter hook forward priority filter; policy accept;
                ip daddr 172.0.0.0/24 oifname "wlp1s0" ct state { established, related } accept
                ip saddr 172.0.0.0/24 iifname "wlp1s0" accept
                iifname "wlp1s0" oifname "wlp1s0" accept
                iifname "wlp1s0" reject
                oifname "wlp1s0" reject
        }
}

Then run
sudo nft flush ruleset
to make ros multi-pc work again.
