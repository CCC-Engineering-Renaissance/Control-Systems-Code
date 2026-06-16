#!/usr/bin/env bash
# One-time install of the ROV launcher daemon on the Raspberry Pi.
#
# Usage (after this repo is pulled onto the Pi at ~/Control-Systems-Code):
#   ssh pi@192.168.8.128 'cd ~/Control-Systems-Code && bash install_pi.sh'
#
# If the repo lives elsewhere or the user is not "pi", edit the paths in
# rov-launcher.service before running this.
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

sudo cp rov-launcher.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now rov-launcher
systemctl status rov-launcher --no-pager
