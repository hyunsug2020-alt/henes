#!/bin/bash

set -e

WS_DIR="${HOME}/henes_ws_ros2"
APP_DIR="${WS_DIR}/src/jeju"

if [ ! -d "${APP_DIR}" ]; then
  echo "[ERROR] App directory not found: ${APP_DIR}"
  exit 1
fi

# 터미널 1: Serial Bridge
gnome-terminal -- bash -c "
    source /opt/ros/humble/setup.bash
    source ${WS_DIR}/install/setup.bash
    cd ${APP_DIR}
    ros2 run jeju serial_bridge_node.py
    exec bash"

sleep 2

# 터미널 2: Joy Node
gnome-terminal -- bash -c "
    source /opt/ros/humble/setup.bash
    source ${WS_DIR}/install/setup.bash
    cd ${APP_DIR}
    ros2 run joy joy_node
    exec bash"

sleep 1

# 터미널 3: Teleop
gnome-terminal -- bash -c "
    source /opt/ros/humble/setup.bash
    source ${WS_DIR}/install/setup.bash
    cd ${APP_DIR}
    ros2 run jeju teleop_node.py
    exec bash"
