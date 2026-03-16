#!/bin/bash
set -e

echo "============================================"
echo " MORAI MPC Container (ROS2 Humble)"
echo " 차량: Ioniq 5 SUV"
echo " 제어: LTV-MPC (전진) / RTI-NMPC (주차)"
echo "============================================"

# ROS2 환경 로드
source /opt/ros/humble/setup.bash

# 워크스페이스 빌드 결과 로드
if [ -f /home/moraimpc/install/setup.bash ]; then
    source /home/moraimpc/install/setup.bash
    echo "[OK] 워크스페이스 로드 완료"
else
    echo "[WARN] 워크스페이스 빌드 필요: cd /home/moraimpc && colcon build"
fi

# X11 권한
if [ -n "$DISPLAY" ]; then
    xhost +local:root 2>/dev/null || true
    echo "[OK] X11 디스플레이: $DISPLAY"
fi

echo "--------------------------------------------"
echo "실행 명령 예시:"
echo "  ros2 launch moraimpc morai_full.launch.py"
echo "  ros2 launch moraimpc morai_mpc.launch.py"
echo "  ros2 launch moraimpc morai_parking.launch.py"
echo "--------------------------------------------"

exec "$@"
