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

sudo cp rov-launcher.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now rov-launcher
systemctl status rov-launcher --no-pager
