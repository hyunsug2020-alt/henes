# HENES MPC (C++ / ROS 2)

이 폴더는 HENES T870용 경로 생성/추종을 C++로 구성한 MPC 모듈입니다.
구성은 `hyunsug2020-alt/mpc` 레포의 `controller + tracker` 분리 방식을 참고해 실차 1대용으로 단순화했습니다.

## 구성
- `src/mpc_path_maker_node.cpp`: `/odometry/filtered`, `/gps/quality`, `/ublox_gps/fix_velocity` 기반 경로 생성
- `src/mpc_controller_cpp.cpp`: MPC 후보 평가/최적 조향 계산
- `src/mpc_path_tracker_cpp.cpp`: 실차 입력 토픽 구독 + `/cmd_vel` 발행
- `src/mpc_path_follower_node.cpp`: tracker 실행 엔트리포인트
- `include/jeju_mpc/*.hpp`: 노드 인터페이스

## 아두이노 연동
- 기존 `serial_bridge_node.py`가 `/cmd_vel`의 `linear`/`angular`를 Arduino JSON으로 전달합니다.
- MPC follower는 `cmd.linear.x`를 PWM 스케일(기존 코드와 동일), `cmd.angular.z`를 조향각(deg)으로 발행합니다.
- 목표점 도착 시 `goal_tolerance_m` 이내에서 정지 명령을 래치하고 유지합니다.

## 핵심 파라미터
- `wheelbase_m`: 차량 축거
- `forward_speed_kmh`: 목표 속도
- `prediction_horizon`: 예측 스텝 수
- `w_d`, `w_theta`, `w_kappa`: 비용함수 가중치
- `kappa_min`, `kappa_max`, `kappa_samples`: 곡률 탐색 범위/해상도
- `max_steer_deg`: 조향각 제한

## 참고한 MPC 접근
- Kinematic bicycle model 기반 예측
- finite-horizon 비용 최소 후보 곡률 탐색 방식

### 참고 자료 (논문/구현)
- OSQP 논문: https://web.stanford.edu/~boyd/papers/osqp.html
- OSQP 문서: https://osqp.org/docs/
- PythonRobotics MPC 설명: https://atsushisakai.github.io/PythonRobotics/modules/6_path_tracking/model_predictive_speed_and_steering_control/model_predictive_speed_and_steering_control.html
- Autoware MPC Lateral Controller 설계: https://autowarefoundation.github.io/autoware_universe/main/control/autoware_mpc_lateral_controller/
- ROS2 MPC 구현 레퍼런스: https://github.com/kuralme/mpc_ros2
- ROS/ROS2 C++ MPC Planner 레퍼런스: https://github.com/tud-amr/mpc_planner
