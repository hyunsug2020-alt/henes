#!/usr/bin/env bash
set -eo pipefail

PORT="${1:-/dev/henes_arduino}"
WS_DIR="${HOME}/henes_ws_ros2"
LOG_DIR="/tmp/henes_json_verify"
mkdir -p "${LOG_DIR}"

echo "[1/6] Check serial port: ${PORT}"
if [[ ! -e "${PORT}" ]]; then
  echo "[FAIL] Serial port not found: ${PORT}"
  exit 1
fi
ls -l "${PORT}" || true

echo "[2/6] Source ROS 2 Humble + workspace"
# setup.bash can reference unset vars; avoid nounset here.
set +u
source /opt/ros/humble/setup.bash
source "${WS_DIR}/install/setup.bash"
set -u

echo "[3/6] Start serial_bridge_node"
ros2 run jeju serial_bridge_node.py --ros-args -p port:="${PORT}" \
  > "${LOG_DIR}/serial_bridge.log" 2>&1 &
SPID=$!
trap 'kill ${SPID} >/dev/null 2>&1 || true' EXIT
sleep 3

echo "[4/6] Publish cmd_vel once"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 80.0}, angular: {z: 0.0}}" \
  > "${LOG_DIR}/cmd_pub.log" 2>&1 || true

echo "[5/6] Wait arduino heartbeat"
set +e
timeout 6 ros2 topic echo /arduino_heartbeat --once > "${LOG_DIR}/hb.log" 2>&1
HB_RC=$?
set -e

echo "[6/6] Result"
if [[ ${HB_RC} -eq 0 ]]; then
  echo "[PASS] /arduino_heartbeat received."
  sed -n '1,40p' "${LOG_DIR}/hb.log"
else
  echo "[FAIL] No /arduino_heartbeat."
  echo "--- serial_bridge.log ---"
  sed -n '1,120p' "${LOG_DIR}/serial_bridge.log"
  echo "--- hb.log ---"
  sed -n '1,80p' "${LOG_DIR}/hb.log"
  exit 2
fi
