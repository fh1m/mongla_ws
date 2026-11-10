#!/bin/sh
# Share the dev box's wlan0 internet with the Pi on the 10.42.0.0/24 link.
#
# NetworkManager's "Shared to other computers" gives the link an address and DHCP
# but NOT a masquerade rule for this subnet -- Docker's rules were present and the
# Pi's was not, which is why the Pi could resolve DNS (systemd-resolved on the dev
# box answered) while every packet to an external IP was dropped. That split is
# the confusing part: name lookups work, nothing else does.
set -e
LAN=enp59s0u1c2      # the Pi link
WAN=wlan0            # the internet-facing interface
sysctl -qw net.ipv4.ip_forward=1
iptables -t nat -C POSTROUTING -s 10.42.0.0/24 -o "$WAN" -j MASQUERADE 2>/dev/null || \
  iptables -t nat -A POSTROUTING -s 10.42.0.0/24 -o "$WAN" -j MASQUERADE
iptables -C FORWARD -i "$LAN" -o "$WAN" -j ACCEPT 2>/dev/null || \
  iptables -A FORWARD -i "$LAN" -o "$WAN" -j ACCEPT
iptables -C FORWARD -i "$WAN" -o "$LAN" -m state --state RELATED,ESTABLISHED -j ACCEPT 2>/dev/null || \
  iptables -A FORWARD -i "$WAN" -o "$LAN" -m state --state RELATED,ESTABLISHED -j ACCEPT
echo "Pi internet sharing: $LAN -> $WAN (10.42.0.0/24 masqueraded)"
