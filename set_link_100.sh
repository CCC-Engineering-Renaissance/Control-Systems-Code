#!/usr/bin/env bash
# set_link_100.sh — force the Pi's wired Ethernet to 100 Mbps full-duplex.
#
# Why: an ROV tether typically uses only 2 of the 4 twisted pairs, which cannot
# carry gigabit. Left on autonegotiation the link tries 1000 Mbps, fails, and
# comes up flaky or not at all. Locking it to 100 Mbps full-duplex gives a
# stable link over 2 pairs.
#
# Run automatically at boot by rov-launcher.service (ExecStartPre, as root).
# Can also be run by hand:  sudo ./set_link_100.sh
#
# Override the interface or speed via environment:
#   ROV_LINK_IFACE=eth0  ROV_LINK_SPEED=100  sudo ./set_link_100.sh
#
# NOTE: with autoneg off, set the TOPSIDE end to 100/full too. If the other end
# autonegotiates it will parallel-detect to half-duplex (a duplex mismatch that
# tanks throughput). If you can't fix the far end, switch the command below to
# autoneg-on, 100-only:  ethtool -s "$IFACE" autoneg on advertise 0x008
set -u

SPEED="${ROV_LINK_SPEED:-100}"
IFACE="${ROV_LINK_IFACE:-}"

# Auto-detect the wired interface if not given: Pi 5 = end0, older Pi = eth0,
# USB-Ethernet = enx*/en*.
if [ -z "$IFACE" ]; then
  for cand in end0 eth0; do
    [ -e "/sys/class/net/$cand" ] && IFACE="$cand" && break
  done
fi
if [ -z "$IFACE" ]; then
  for p in /sys/class/net/en*; do
    [ -e "$p" ] && IFACE="$(basename "$p")" && break
  done
fi

if [ -z "$IFACE" ]; then
  echo "set_link_100: no wired interface found — skipping" >&2
  exit 0   # don't fail the launcher service over a missing NIC
fi
if ! command -v ethtool >/dev/null 2>&1; then
  echo "set_link_100: ethtool not installed (sudo apt install ethtool) — skipping" >&2
  exit 0
fi

echo "set_link_100: forcing $IFACE to ${SPEED} Mbps full-duplex (autoneg off)"
ethtool -s "$IFACE" speed "$SPEED" duplex full autoneg off
