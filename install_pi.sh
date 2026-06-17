#!/usr/bin/env bash
# One-time install of the ROV launcher daemon on the Raspberry Pi.
#
# Run this ON the Pi, from inside the repo:
#   cd ~/Control-Systems-Code && bash install_pi.sh
#
# The systemd unit is generated for the CURRENT user and repo path, so it
# works regardless of the Pi's username (it does NOT assume "pi"). Override
# the port with: ROV_DAEMON_PORT=5011 bash install_pi.sh
set -euo pipefail

cd "$(dirname "$0")"

# ── Depth/pressure sensor I2C bus ────────────────────────────────────────────
# The MS5837 (Bar30/Bar02) is wired to GPIO 12 (SDA) / GPIO 13 (SCL) — physical
# pins 32/33 — which is NOT a default I2C bus. Create it as a software-I2C bus
# (/dev/i2c-5) via the i2c-gpio device-tree overlay. Idempotent: only appended
# once. Takes effect after a reboot.
if [ -f /boot/firmware/config.txt ]; then
  BOOT_CONFIG=/boot/firmware/config.txt   # Raspberry Pi OS Bookworm and later
elif [ -f /boot/config.txt ]; then
  BOOT_CONFIG=/boot/config.txt            # older Raspberry Pi OS
else
  BOOT_CONFIG=""
  echo "WARNING: no Pi boot config found (/boot/firmware/config.txt or /boot/config.txt)."
  echo "         Add this line manually so the depth sensor bus exists:"
  echo "           dtoverlay=i2c-gpio,bus=5,i2c_gpio_sda=12,i2c_gpio_scl=13"
fi

if [ -n "$BOOT_CONFIG" ]; then
  OVERLAY="dtoverlay=i2c-gpio,bus=5,i2c_gpio_sda=12,i2c_gpio_scl=13"
  if grep -qF "$OVERLAY" "$BOOT_CONFIG"; then
    echo "Depth-sensor I2C overlay already present in $BOOT_CONFIG."
  else
    echo "Adding depth-sensor I2C overlay to $BOOT_CONFIG (reboot required to apply)."
    printf '\n# ROV depth/pressure sensor (MS5837) on GPIO 12/13 = pins 32/33\n%s\n' \
      "$OVERLAY" | sudo tee -a "$BOOT_CONFIG" >/dev/null
    echo "NOTE: reboot the Pi, then verify with:  i2cdetect -y 5   (expect 0x76)"
  fi
fi

# ── Launcher daemon systemd unit ─────────────────────────────────────────────
# Generated from the current user + repo path so it never assumes "pi".
REPO="$(pwd)"
RUN_USER="$(whoami)"
PYTHON="$(command -v python3 || true)"
PORT="${ROV_DAEMON_PORT:-5010}"

if [ ! -f "$REPO/pi_launcher.py" ]; then
  echo "error: pi_launcher.py not found in $REPO — run this from the repo on the Pi." >&2
  exit 1
fi
if [ -z "$PYTHON" ]; then
  echo "error: python3 not found in PATH." >&2
  exit 1
fi

# ── Wired-link speed (100 Mbps for 2-pair ROV tethers) ───────────────────────
# ethtool is needed by set_link_100.sh, which the launcher service runs at boot.
if ! command -v ethtool >/dev/null 2>&1; then
  echo "Installing ethtool (needed to force the wired link to 100 Mbps)…"
  sudo apt-get install -y ethtool || echo "WARNING: could not install ethtool — link speed will not be forced."
fi
chmod +x "$REPO/set_link_100.sh" 2>/dev/null || true

echo "Installing rov-launcher.service: user=$RUN_USER repo=$REPO python=$PYTHON port=$PORT"

sudo tee /etc/systemd/system/rov-launcher.service >/dev/null <<EOF
[Unit]
Description=ROV task launcher daemon (pi_launcher.py)
After=network-online.target
Wants=network-online.target

[Service]
User=$RUN_USER
# Force the wired link to 100 Mbps full-duplex before the daemon starts.
# '+' runs it as root (ethtool needs root even though the daemon runs as
# $RUN_USER); '-' makes a failure non-fatal so a missing NIC/ethtool won't
# block the launcher.
ExecStartPre=-+$REPO/set_link_100.sh
ExecStart=$PYTHON -u $REPO/pi_launcher.py --dir $REPO --port $PORT
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl reset-failed rov-launcher 2>/dev/null || true
sudo systemctl enable --now rov-launcher
systemctl status rov-launcher --no-pager

# Apply the 100 Mbps link now too, so it takes effect without a reboot.
echo "Forcing wired link to 100 Mbps now…"
sudo "$REPO/set_link_100.sh" || true
