#!/bin/bash

set -e

WS_DIR="${HOME}/henes_ws_ros2"
APP_DIR="${WS_DIR}/src/jeju"

if [ ! -d "${APP_DIR}" ]; then
  echo "[ERROR] App directory not found: ${APP_DIR}"
  exit 1
fi

# 터미널 1: Joy + Teleop + Serial Bridge (권장 런치)
gnome-terminal -- bash -c "
    source /opt/ros/humble/setup.bash
    source ${WS_DIR}/install/setup.bash
    cd ${APP_DIR}
    ros2 launch jeju joy.launch.py arduino_port:=/dev/henes_arduino
    exec bash"
