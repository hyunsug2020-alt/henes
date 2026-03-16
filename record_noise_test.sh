#!/usr/bin/env bash
set -euo pipefail

# Auto-record ROS 2 bag for MPC/noise tuning with a guided driving timeline.
# Usage:
#   ./record_noise_test.sh
#   ./record_noise_test.sh --manual
#   ./record_noise_test.sh --output my_test_name

WORKSPACE="${HOME}/henes_ws_ros2"
BAG_BASE="${WORKSPACE}/bags"
MANUAL_MODE=0
CUSTOM_NAME=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --manual)
      MANUAL_MODE=1
      shift
      ;;
    --output)
      CUSTOM_NAME="${2:-}"
      shift 2
      ;;
    *)
      echo "Unknown option: $1"
      echo "Usage: $0 [--manual] [--output NAME]"
      exit 1
      ;;
  esac
done

if ! command -v ros2 >/dev/null 2>&1; then
  # Try common ROS 2 setup paths.
  source /opt/ros/humble/setup.bash
  if [[ -f "${WORKSPACE}/install/setup.bash" ]]; then
    source "${WORKSPACE}/install/setup.bash"
  fi
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 command not found. Source ROS 2 first."
  exit 1
fi

mkdir -p "${BAG_BASE}"
STAMP="$(date +%Y%m%d_%H%M%S)"
if [[ -n "${CUSTOM_NAME}" ]]; then
  BAG_NAME="${CUSTOM_NAME}"
else
  BAG_NAME="noise_test_${STAMP}"
fi
BAG_PATH="${BAG_BASE}/${BAG_NAME}"

TOPICS=(
  /odometry/filtered
  /fix
  /ublox_gps/heading
  /ublox_gps/fix_velocity
  /cmd_vel
  /steering_angle
  /pwm_output
  /imu/data
)

echo "[noise_test] output: ${BAG_PATH}"
echo "[noise_test] topics:"
printf '  - %s\n' "${TOPICS[@]}"
echo

cleanup() {
  if [[ -n "${REC_PID:-}" ]] && kill -0 "${REC_PID}" 2>/dev/null; then
    echo
    echo "[noise_test] stopping ros2 bag record..."
    kill -INT "${REC_PID}" 2>/dev/null || true
    wait "${REC_PID}" || true
  fi
  echo "[noise_test] done"
  echo "[noise_test] bag: ${BAG_PATH}"
}
trap cleanup EXIT INT TERM

ros2 bag record -o "${BAG_PATH}" "${TOPICS[@]}" &
REC_PID=$!
sleep 2

if ! kill -0 "${REC_PID}" 2>/dev/null; then
  echo "[noise_test] recorder failed to start."
  exit 1
fi

if [[ "${MANUAL_MODE}" -eq 1 ]]; then
  echo "[noise_test] manual mode: drive as needed, then press Ctrl+C here."
  wait "${REC_PID}"
  exit 0
fi

echo "[noise_test] guided mode start"
echo "  1) 30s 정지"
sleep 30
echo "  2) 30s 직진(저속)"
sleep 30
echo "  3) 30s 제자리 저속 원회전(좌)"
sleep 30
echo "  4) 30s 제자리 저속 원회전(우)"
sleep 30
echo "  5) 60s 8자 주행"
sleep 60
echo "  6) 30s 정지"
sleep 30

echo "[noise_test] guided sequence complete. saving..."
kill -INT "${REC_PID}" 2>/dev/null || true
wait "${REC_PID}" || true
