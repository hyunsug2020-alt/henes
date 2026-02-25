# HENES T870 ROS 2 Humble Control Package

HENES T870 차량의 ROS 2 Humble 제어 패키지입니다.

## 시스템 요구사항

- Ubuntu 22.04 LTS
- ROS 2 Humble
- Python 3.10+
- Arduino IDE 1.8.x or 2.x

## 하드웨어 구성

- **MCU**: Arduino Mega 2560
- **모터 드라이버**: MD20A (구동), 조향 드라이버
- **센서**: 엔코더 (ENC_A, ENC_B), 조향 가변저항 (POT A8)
- **조이스틱**: PS5 듀얼센스
- **통신**: USB Serial (`/dev/henes_arduino`, 57600 baud)

## 설치 방법

### 1. ROS 2 Humble 설치

```bash
# ROS 2 Humble이 이미 설치되어 있다고 가정
source /opt/ros/humble/setup.bash
```

### 2. 워크스페이스 생성

```bash
mkdir -p ~/henes_ws_ros2/src
cd ~/henes_ws_ros2/src
```

### 3. 패키지 복사

이 `jeju` 폴더를 `~/henes_ws_ros2/src/`에 복사합니다.

```bash
cp -r jeju ~/henes_ws_ros2/src/
```

### 4. 의존성 설치

```bash
cd ~/henes_ws_ros2
sudo apt update
sudo apt install -y \
    ros-humble-joy \
    ros-humble-tf2-ros \
    python3-serial \
    python3-pip

pip3 install pyserial
```

### 5. 빌드

```bash
cd ~/henes_ws_ros2
colcon build --symlink-install
source install/setup.bash
```

## Arduino 설정

### 1. ArduinoJson 라이브러리 설치

Arduino IDE에서:
1. `Sketch` → `Include Library` → `Manage Libraries...`
2. "ArduinoJson" 검색
3. 버전 6.x 설치 (예: 6.21.3)

### 2. Arduino 코드 업로드

```bash
# Arduino IDE에서 다음 파일 열기
~/henes_ws_ros2/src/jeju/arduino/henes_control_ros2.ino

# Board: Arduino Mega 2560
# Port: /dev/henes_arduino
# Upload
```

### 3. 시리얼 포트 권한 설정

```bash
sudo usermod -a -G dialout $USER
sudo chmod 666 /dev/henes_arduino

# 재부팅 또는 로그아웃 후 다시 로그인
```

## 조이스틱 (PS5 듀얼센스) 설정

### 1. 블루투스 페어링

```bash
# 블루투스 설정에서 PS5 듀얼센스 페어링
# Share + PS 버튼 동시에 3초간 누르기
```

### 2. 조이스틱 테스트

```bash
# 조이스틱이 인식되는지 확인
ls /dev/input/js*

# 조이스틱 데이터 확인
ros2 run joy joy_node
ros2 topic echo /joy
```

## 사용 방법

### 1. 전체 시스템 실행

```bash
source ~/henes_ws_ros2/install/setup.bash
ros2 launch jeju joy_control.launch.py

# 권장 고정 alias
#   GPS:      /dev/henes_gps
#   IMU:      /dev/henes_imu
#   Arduino:  /dev/henes_arduino
# 전체 스택 실행 시 (기본값 이미 위 alias로 설정됨)
ros2 launch jeju henes_full_stack.launch.py \
  gps_device:=/dev/henes_gps \
  imu_device:=/dev/henes_imu
```

### 2. 개별 노드 실행

```bash
# 터미널 1: Joy 노드
ros2 run joy joy_node

# 터미널 2: Teleop 노드
ros2 run jeju teleop_node

# 터미널 3: Serial Bridge 노드
ros2 run jeju serial_bridge_node
```

### 3. 파라미터 변경

```bash
# 최대 속도 변경
ros2 param set /teleop_node max_velocity 30

# 시리얼 포트 변경
ros2 run jeju serial_bridge_node.py --ros-args -p port:=/dev/henes_arduino
```

## 조이스틱 매핑 (PS5 듀얼센스)

| 버튼/스틱 | 기능 |
|-----------|------|
| 왼쪽 아날로그 스틱 (상하) | 속도 제어 |
| 오른쪽 아날로그 스틱 (좌우) | 조향 제어 |
| R1 버튼 | 수동 모드 활성화 |
| L1 버튼 | 자율주행 모드 활성화 |
| Triangle (△) | 최대 속도 +10 |
| X (✕) | 최대 속도 -10 |

## 토픽 구조

## 제어 경로 정리

- 현재 모터/조향 제어의 단일 경로는 `jeju/control/serial_bridge_node.py` 입니다.
- `cmd_vel` 명령은 `serial_bridge_node`를 통해 제어보드로 전달됩니다.
- 기존 C++ 제어 노드는 더 이상 사용하지 않으며 제거되었습니다.

### Published Topics (by Serial Bridge)
- `/encoder1` (std_msgs/Int32) - 전방 엔코더 카운트
- `/encoder2` (std_msgs/Int32) - 후방 엔코더 카운트
- `/steering_angle` (std_msgs/Float64) - 현재 조향 각도
- `/steering_error` (std_msgs/Float64) - PID 제어 오차
- `/raw_sensor` (std_msgs/Float64) - 조향 센서 원본 값
- `/pwm_output` (std_msgs/Float64) - 조향 모터 PWM 출력

### Subscribed Topics (by Serial Bridge)
- `/cmd_vel` (geometry_msgs/Twist) - 속도 및 조향 명령
- `/pid/kp` (std_msgs/Float64) - PID Kp 값
- `/pid/ki` (std_msgs/Float64) - PID Ki 값
- `/pid/kd` (std_msgs/Float64) - PID Kd 값
- `/steering/neutral_angle` (std_msgs/Float64) - 조향 중립 각도
- `/filter/size` (std_msgs/Float64) - 센서 필터 크기
- `/steering/min_sensor` (std_msgs/Float64) - 센서 최소값
- `/steering/max_sensor` (std_msgs/Float64) - 센서 최대값

### Published Topics (by Teleop)
- `/cmd_vel` (geometry_msgs/Twist) - 조이스틱 제어 명령
- `/teleop_mode` (std_msgs/Bool) - 텔레옵 모드 상태

## 트러블슈팅

### 1. 시리얼 포트 연결 실패

```bash
# 포트 확인
ls -l /dev/henes_arduino

# 권한 확인
sudo chmod 666 /dev/henes_arduino

# 포트가 없으면 Arduino USB 케이블 재연결
```

### 2. 조이스틱 인식 안됨

```bash
# 조이스틱 장치 확인
ls /dev/input/js*

# joy 노드 로그 확인
ros2 run joy joy_node --ros-args --log-level debug
```

### 3. Arduino 통신 오류

```bash
# 시리얼 모니터로 JSON 메시지 확인
# Arduino IDE: Tools → Serial Monitor (57600 baud)

# ROS 2에서 수동으로 명령 전송 테스트
echo '{"cmd":"vel","linear":50,"angular":0}' > /dev/henes_arduino
```

### 4. PID 튜닝

```bash
# Kp 값 조정
ros2 topic pub /pid/kp std_msgs/Float64 "{data: 3.0}" --once

# 실시간 조향 오차 모니터링
ros2 topic echo /steering_error
```

## 통신 프로토콜

### ROS 2 → Arduino (JSON)
```json
{
  "cmd": "vel",
  "linear": 100,
  "angular": 30
}
```

### Arduino → ROS 2 (JSON)
```json
{
  "enc1": 1234,
  "enc2": 5678,
  "steer": 15.5,
  "error": 2.3,
  "raw_sensor": 450,
  "pwm": 120
}
```

## ROS 2 Humble 기준

이 패키지는 ROS 2 Humble 기준으로만 유지됩니다.

## 라이선스

TODO

## 문의

- Maintainer: kmu@todo.todo
