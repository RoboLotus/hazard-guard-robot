#!/usr/bin/env bash
# Install the always-on HazardGuard IoT stack as systemd services.
#
#   scripts/install_services.sh <workspace> [<overlay workspace> ...]
#   scripts/install_services.sh --uninstall
#
# Each workspace's install/setup.bash is sourced in order after ROS itself.
# Paths are substituted at install time so nothing machine-specific is
# committed. Needs sudo for /etc/systemd, /etc/sudoers.d and /etc/avahi.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
UNITS=(hazardguard-base hazardguard-iot hazardguard-status)
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
DOMAIN="${ROS_DOMAIN_ID:-61}"

if [[ "${1:-}" == "--uninstall" ]]; then
  sudo systemctl disable --now "${UNITS[@]}" 2>/dev/null || true
  for u in "${UNITS[@]}"; do sudo rm -f "/etc/systemd/system/$u.service"; done
  sudo rm -f /etc/sudoers.d/hazardguard /etc/avahi/services/hazardguard.service
  sudo systemctl daemon-reload
  sudo systemctl reload avahi-daemon 2>/dev/null || true
  echo "removed"
  exit 0
fi

[[ $# -ge 1 ]] || { echo "usage: $0 <workspace> [<overlay> ...] | --uninstall" >&2; exit 1; }

source_line="source $ROS_SETUP;"
for ws in "$@"; do
  setup="$(cd "$ws" && pwd)/install/setup.bash"
  [[ -f "$setup" ]] || { echo "not built: $setup" >&2; exit 1; }
  source_line+=" source $setup;"
done

for u in "${UNITS[@]}"; do
  sed -e "s|@USER@|$USER|g" -e "s|@ROS_DOMAIN_ID@|$DOMAIN|g" -e "s|@SOURCE@|$source_line|g" \
    "$HERE/systemd/$u.service.in" | sudo tee "/etc/systemd/system/$u.service" >/dev/null
done

# physical_m1_bringup.launch.py stops/starts the serial-port service without a password
printf '%s ALL=(root) NOPASSWD: /usr/bin/systemctl stop hazardguard-base.service, /usr/bin/systemctl start hazardguard-base.service\n' "$USER" \
  | sudo tee /etc/sudoers.d/hazardguard >/dev/null
sudo chmod 440 /etc/sudoers.d/hazardguard

# advertise the status server so the SmartThings hub finds this robot over mDNS
sudo cp "$HERE/systemd/hazardguard-avahi.service" /etc/avahi/services/hazardguard.service
sudo systemctl reload avahi-daemon

sudo systemctl daemon-reload
sudo systemctl enable --now "${UNITS[@]}"
systemctl --no-pager --no-legend list-units 'hazardguard-*'
