#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_WORKSPACE="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
CONSOLE_ROOT="${HAZARD_GUARD_CONSOLE_ROOT:-${ROBOT_WORKSPACE%/hazard-guard-robot}/hazard-guard-console}"
BACKEND_DIR="${CONSOLE_ROOT}/backend"
BACKEND_PYTHON="${HAZARD_GUARD_BACKEND_PYTHON:-/home/jetson/venvs/hazard-guard-web/bin/python}"

HOST=""
PORT=8000
MAX_WEB_POINTS=100000
VOXEL_SIZE=0.03

if [[ "${1:-}" == "--list" || "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  show_usage=1
else
  show_usage=0
fi

PROFILE=""

if [[ $# -gt 0 && "$1" != --* ]]; then
  PROFILE="$1"
  shift
elif [[ -z "${PROFILE}" ]]; then
  PROFILE="pc-9000"
fi

usage() {
  cat <<'USAGE'
Usage:
  tools/run_pointcloud_benchmark.sh [PROFILE] [options]
  tools/run_pointcloud_benchmark.sh --list

Options:
  --host IP             Backend bind address (default: Tailscale IPv4)
  --port PORT           Backend port (default: 8000)
  --voxel-size METERS   Accumulated map voxel size (default: 0.03)

Profiles:
  pc-3000    3,000 points/frame, decimation 4
  pc-6000    6,000 points/frame, decimation 4
  pc-9000    9,000 points/frame, decimation 2 (default)
  pc-12000  12,000 points/frame, decimation 2

This command runs only the FastAPI backend. In the WebUI select:
  지도 운용 모드 -> 2D + RGB-D 3D -> 새 맵 생성

Stop the current backend and WebUI-managed map before changing profiles.
USAGE
}

die() {
  printf 'error: %s\n' "$*" >&2
  exit 2
}

if ((show_usage == 1)); then
  usage
  exit 0
fi

while [[ $# -gt 0 ]]; do
  case "$1" in
    --host)
      [[ $# -ge 2 ]] || die "--host requires an IP address"
      HOST="$2"
      shift 2
      ;;
    --port)
      [[ $# -ge 2 ]] || die "--port requires a number"
      PORT="$2"
      shift 2
      ;;
    --voxel-size)
      [[ $# -ge 2 ]] || die "--voxel-size requires a value in meters"
      VOXEL_SIZE="$2"
      shift 2
      ;;
    *)
      die "unknown option: $1"
      ;;
  esac
done

case "${PROFILE}" in
  pc-3000)
    POINTS=3000; DECIMATION=4 ;;
  pc-6000)
    POINTS=6000; DECIMATION=4 ;;
  pc-9000)
    POINTS=9000; DECIMATION=2 ;;
  pc-12000)
    POINTS=12000; DECIMATION=2 ;;
  *)
    die "unknown profile '${PROFILE}' (use --list)"
    ;;
esac

[[ "${PORT}" =~ ^[1-9][0-9]*$ ]] || die "--port must be a positive integer"
[[ "${VOXEL_SIZE}" =~ ^(0|[1-9][0-9]*)([.][0-9]+)?$ ]] \
  || die "--voxel-size must be numeric"
awk -v value="${VOXEL_SIZE}" 'BEGIN { exit !(value > 0.0) }' \
  || die "--voxel-size must be greater than zero"
[[ -d "${BACKEND_DIR}/app" ]] || die "console backend not found: ${BACKEND_DIR}"
[[ -x "${BACKEND_PYTHON}" ]] || die "backend Python not found: ${BACKEND_PYTHON}"
[[ -f /opt/ros/humble/setup.bash ]] || die "ROS Humble setup not found"
[[ -f "${ROBOT_WORKSPACE}/install/setup.bash" ]] || die "build the robot workspace first"

if [[ -z "${HOST}" ]] && command -v tailscale >/dev/null 2>&1; then
  HOST="$(tailscale ip -4 2>/dev/null | head -n 1 || true)"
fi
HOST="${HOST:-0.0.0.0}"

HIGH_LOAD_POINTS=$((POINTS / 2))

set +u
source /opt/ros/humble/setup.bash
source "${ROBOT_WORKSPACE}/install/setup.bash"
set -u

export HAZARD_GUARD_MODE_CONTROL_ENABLED=1
export HAZARD_GUARD_DEPLOYMENT_TARGET=physical
export HAZARD_GUARD_WORKSPACE="${ROBOT_WORKSPACE}"
export HAZARD_GUARD_ROS_ENABLED=1
export HAZARD_GUARD_RGB_TOPIC=/ascamera_hp60c/camera_publisher/rgb0/image
export HAZARD_GUARD_RGB_INFO_TOPIC=/ascamera_hp60c/camera_publisher/rgb0/camera_info
export HAZARD_GUARD_DEPTH_TOPIC=/ascamera_hp60c/camera_publisher/depth0/image_raw
export HAZARD_GUARD_DEPTH_INFO_TOPIC=/ascamera_hp60c/camera_publisher/depth0/camera_info
export HAZARD_GUARD_SCAN_TOPIC=/scan
export HAZARD_GUARD_IMU_TOPIC=/imu/data_raw
export HAZARD_GUARD_ODOM_TOPIC=/odom
export HAZARD_GUARD_POINT_CLOUD_TOPIC=/hazard_guard/rtabmap/cloud_surface
export HAZARD_GUARD_POINT_CLOUD_MAX_POINTS="${MAX_WEB_POINTS}"
export HAZARD_GUARD_POINT_CLOUD_INTERVAL_SEC=0.75
export HAZARD_GUARD_CLOUD_NORMAL_POINTS="${POINTS}"
export HAZARD_GUARD_CLOUD_HIGH_LOAD_POINTS="${HIGH_LOAD_POINTS}"
export HAZARD_GUARD_CLOUD_NORMAL_INPUT_HZ=8.0
export HAZARD_GUARD_CLOUD_HIGH_LOAD_INPUT_HZ=4.0
export HAZARD_GUARD_CLOUD_NORMAL_SURFACE_HZ=1.0
export HAZARD_GUARD_CLOUD_HIGH_LOAD_SURFACE_HZ=0.5
export HAZARD_GUARD_CLOUD_DECIMATION="${DECIMATION}"
export HAZARD_GUARD_CLOUD_VOXEL_SIZE="${VOXEL_SIZE}"
export HAZARD_GUARD_CLOUD_LINEAR_UPDATE=0.10
export HAZARD_GUARD_CLOUD_ANGULAR_UPDATE=0.10472

printf 'HazardGuard WebUI benchmark profile\n'
printf '  profile:       %s\n' "${PROFILE}"
printf '  points/frame:  %s (high load: %s)\n' "${POINTS}" "${HIGH_LOAD_POINTS}"
printf '  input rate:    8 Hz (high load: 4 Hz)\n'
printf '  decimation:    %s\n' "${DECIMATION}"
printf '  voxel size:    %s m\n' "${VOXEL_SIZE}"
printf '  deployment:    physical / WebUI managed\n'
printf '  backend:       http://%s:%s\n' "${HOST}" "${PORT}"
printf '\nWebUI에서 2D + RGB-D 3D를 선택하고 새 맵 생성을 누르세요.\n\n'

cd "${BACKEND_DIR}"
exec /usr/bin/python3 "${SCRIPT_DIR}/pointcloud_webui_supervisor.py" \
  --backend-python "${BACKEND_PYTHON}" \
  --backend-dir "${BACKEND_DIR}" \
  --host "${HOST}" \
  --port "${PORT}" \
  --profile "${PROFILE}" \
  --points "${POINTS}" \
  --input-hz 8.0 \
  --surface-hz 1.0 \
  --voxel-size "${VOXEL_SIZE}" \
  --decimation "${DECIMATION}"
