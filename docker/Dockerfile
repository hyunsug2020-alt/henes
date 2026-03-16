# ============================================================
# MORAI + ROS2 Humble Docker Image
# Base: Ubuntu 22.04
# 차량: Ioniq 5 SUV
# 제어: 전진=LTV-MPC / 후진주차=RTI-NMPC
# 시각화: Iridescence (pyridescence)
# 항법: ESKF (GPS+IMU 헤딩 융합)
# ============================================================
FROM osrf/ros:humble-desktop-full

ARG DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=humble

# ── 시스템 패키지 ─────────────────────────────────────────
RUN apt-get update && apt-get install -y \
    # 빌드 도구
    build-essential cmake git wget curl \
    python3-pip python3-dev \
    python3-colcon-common-extensions \
    python3-rosdep \
    # Eigen3 + OSQP
    libeigen3-dev \
    # OpenCV
    libopencv-dev python3-opencv \
    # ROS2 패키지
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-robot-localization \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-sensor-msgs \
    ros-humble-nav-msgs \
    ros-humble-geometry-msgs \
    ros-humble-visualization-msgs \
    ros-humble-std-msgs \
    ros-humble-nav2-msgs \
    # 직렬 통신 (시뮬레이터 연결)
    ros-humble-serial-driver \
    # 기타
    python3-tk \
    libglfw3-dev libpng-dev libjpeg-dev \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# ── OSQP 설치 ─────────────────────────────────────────────
RUN pip3 install osqp

# ── Iridescence (시각화 라이브러리) 설치 ──────────────────
# https://github.com/koide3/iridescence
RUN add-apt-repository -y ppa:koide3/iridescence && \
    apt-get update && \
    apt-get install -y libiridescence-dev && \
    rm -rf /var/lib/apt/lists/*

# pyridescence (Python 바인딩)
RUN pip3 install pyridescence

# ── Python 패키지 ─────────────────────────────────────────
RUN pip3 install \
    numpy \
    scipy \
    pyproj \
    matplotlib \
    transforms3d \
    opencv-python-headless

# ── MORAI 메시지 패키지 ────────────────────────────────────
# morai_msgs (ROS2 Humble 버전)
RUN mkdir -p /home/moraimpc/src && \
    cd /home/moraimpc/src && \
    git clone https://github.com/morai-developergroup/morai_msgs.git || \
    echo "[WARN] morai_msgs 클론 실패 - 수동 설치 필요"

# ── 워크스페이스 구성 ─────────────────────────────────────
WORKDIR /home/moraimpc

# 소스 복사
COPY src/ /home/moraimpc/src/

# bisa 패키지 (LTV-MPC + RTI-NMPC C++ 라이브러리) 심링크
# jeju 패키지의 mpc/upstream/src/bisa 를 직접 사용
RUN if [ -d "/home/moraimpc/src/jeju/mpc/upstream/src/bisa" ]; then \
        ln -s /home/moraimpc/src/jeju/mpc/upstream/src/bisa /home/moraimpc/src/bisa; \
    fi

# rosdep 초기화
RUN rosdep init 2>/dev/null || true && rosdep update

# ── 빌드 ──────────────────────────────────────────────────
RUN /bin/bash -c "\
    source /opt/ros/humble/setup.bash && \
    cd /home/moraimpc && \
    colcon build --symlink-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
    " || echo "[WARN] 빌드 일부 실패 - 로그 확인"

# ── 환경 설정 ─────────────────────────────────────────────
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /home/moraimpc/install/setup.bash 2>/dev/null || true" >> /root/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> /root/.bashrc && \
    echo "export RCUTILS_COLORIZED_OUTPUT=1" >> /root/.bashrc

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
