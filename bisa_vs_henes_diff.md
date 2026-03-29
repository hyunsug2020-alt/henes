# bisa vs HENES MPC 구현 비교

> 작성일: 2026-03-28
> 기준: `hyunsug2020-alt/bisa` (시뮬레이션 레퍼런스) vs `/home/coss/henes_ws_ros2/src/jeju` (HENES 실차)

---

## 1. 상태 공간 (State Space)

| 항목 | bisa | HENES |
|------|------|-------|
| 상태 벡터 크기 | 5 | 3 |
| 상태 변수 | `[dr, θ, κ, θr, κr]` | `[e_y, e_psi, v]` |
| 의미 | dr=횡변위, θ=헤딩오차, κ=현재곡률, θr=경로헤딩, κr=경로곡률 | e_y=횡방향오차, e_psi=헤딩오차, v=속도 |
| 제어 입력 | `dκ/dt` (곡률 변화율, 1차원) | `[δ(rad), a(m/s²)]` (조향각+가속도, 2차원) |
| 파일 | `ltv_types.hpp` | `ltv_mpc.hpp` |

**핵심 차이:** bisa는 곡률을 상태에 포함시켜 연속성을 보장하는 반면, HENES는 조향각과 가속도를 직접 입력으로 사용.

---

## 2. 이산화 방법 (Discretization)

| 항목 | bisa | HENES |
|------|------|-------|
| 방법 | **정확 행렬 지수** (exact matrix exponential) | **오일러 근사** (Euler: A=I+Ac·dt) |
| 연속 행렬 | `buildContinuousMatrices()` → Ac, Bc 계산 후 expm | 직접 이산 행렬 A, B 구성 |
| 코드 | `ltv_model.cpp`: `Eigen::Matrix<>... expm(Ac*dt)` | `ltv_mpc.cpp`: `A[k] = Identity; A[k](0,1)=vr*dt` |
| 정확도 | 고속/대 dt에서도 정확 | dt가 작을 때(≤0.1s)는 충분 |

```
// bisa (exact)
Eigen::MatrixXd Ac = buildContinuousMatrices(v_ref, kappa);
Eigen::MatrixXd Ad = (Ac * dt).exp();   // Padé approximation

// HENES (Euler)
A[k] = Eigen::MatrixXd::Identity(n, n);
A[k](0, 1) = vr * dt;
B[k](1, 0) = vr * dt / L;
B[k](2, 1) = dt;
```

---

## 3. QP 구성 방식

| 항목 | bisa | HENES |
|------|------|-------|
| 방식 | Condensed + Sparse (batch dynamics) | Condensed Dense |
| Batch 행렬 | `buildBatchDynamics()` → S_x, S_u (sparse) | `buildCondensed()` → Phi_x0, Theta (dense) |
| 비용 | `buildQPObjective()` → Q_bar, R_bar 분리 | `ltv_mpc.cpp` 내부 인라인 구성 |
| 모듈화 | ltv_model / ltv_cost / ltv_mpc 3개 파일 분리 | ltv_mpc.cpp 1개 파일에 통합 |

---

## 4. 제약 조건 (Constraints)

### 4.1 bisa
```
buildInputConstraints()    : dκ/dt 상하한
buildKappaConstraints()    : 하드 κ 제약 (Gutjahr 2017) — QP 내부에서 곡률 상한 강제
buildLateralSlackConstraints() : 횡방향 소프트 슬랙 (slack variable, penalty)
```
- κ 제약으로 인해 **물리적으로 불가능한 곡률 명령이 QP 내에서 차단됨**
- 슬랙 변수로 경로 이탈 시 MPC 실패 대신 완화 가능

### 4.2 HENES
```
// ltv_mpc.cpp
l_osqp[k*m+0] = delta_min;  u_osqp[k*m+0] = delta_max;  // ±55 deg
l_osqp[k*m+1] = a_min;      u_osqp[k*m+1] = a_max;      // [-3, 2] m/s²
```
- 단순 박스 제약만 존재 (조향각 범위, 가속도 범위)
- **κ 하드 제약 없음, 슬랙 변수 없음**

---

## 5. 속도 계획 (Velocity Planning)

| 항목 | bisa | HENES |
|------|------|-------|
| 방식 | **VelocityPlanner** (별도 QP) | PI 제어기 + 곡률 감속 휴리스틱 |
| 횡가속도 제한 | `a_lat_max = 0.08 m/s²` (파라미터) | 없음 |
| 곡률 반영 | QP 내에서 최적 속도 프로파일 계산 | `v_ref = v_cmd / (1 + kappa_factor * |κ|)` |
| 속도 스무딩 | Sigmoid velocity smoother | 없음 |
| 파일 | `mpc_controller_cpp.hpp`: VelocityPlanner 클래스 | `mpc_path_tracker_cpp.cpp`: speed_integral_ |

```cpp
// bisa VelocityPlanner (QP 기반)
VelocityPlannerParams vp_params;
vp_params.v_max       = 2.0;   // m/s
vp_params.a_lat_max   = 0.08;  // m/s² (곡선에서 자동 감속)
vp_params.a_lon_max   = 0.5;   // m/s²

// HENES 휴리스틱
double v_ref = forward_speed_ / (1.0 + kappa_speed_factor_ * std::abs(kappa));
```

---

## 6. α 블렌딩 (Curvature Blending)

| 항목 | bisa | HENES |
|------|------|-------|
| 구현 여부 | **있음** | 없음 |
| 방식 | `kappa_cmd = alpha * kappa_prev + (1-alpha) * kappa_new` | 직접 조향각 출력 |
| alpha 값 | 0.40 | — |
| 목적 | 곡률 명령의 급격한 변동 억제 | — |

---

## 7. 복구 동작 (Recovery Behaviors)

| 항목 | bisa | HENES |
|------|------|-------|
| path_hold | O (경로 이탈 시 현재 자세 유지) | X |
| oscillation guard | O (조향 진동 감지 → 제동) | X |
| off_path_recovery | O (경로에서 멀어지면 최근접점으로 복귀) | X |
| 곡선 감속 | O (curve speed reduction, a_lat 기반) | 부분적 (kappa_factor 휴리스틱) |
| 시그모이드 스무더 | O | X |

---

## 8. 제어 출력 형식

| 항목 | bisa | HENES |
|------|------|-------|
| 출력 타입 | `MPCCommand { v_cmd [m/s], omega [rad/s] }` | `MPCControlOutput { steer_deg [deg], torque_cmd [m/s²] }` |
| 구동 방식 | 차동구동 (Twist → linear/angular) | 아커만 조향 (steering angle + PWM) |
| 최종 토픽 | `/cmd_vel` (geometry_msgs/Twist) | `/cmd_vel` (linear.x=속도PWM, angular.z=조향PWM) |

---

## 9. 경로 입력 형식

| 항목 | bisa | HENES |
|------|------|-------|
| 토픽 | `/path` (nav_msgs/Path) | `/path` (nav_msgs/Path) |
| 내부 형식 | PoseStamped에서 곡률 별도 계산 | `RefPoint { x, y, yaw, kappa_r }` |
| 곡률 계산 | `buildReferenceProfiles()` (경로 로드 시 1회) | `mpc_path_maker_node`에서 사전 계산 후 kappa_r 저장 |

---

## 10. 파라미터 구조

### bisa `LTVMPCConfig`
```cpp
struct LTVMPCConfig {
    int    N         = 20;
    double dt        = 0.1;
    double Q_dr      = 100.0;
    double Q_theta   = 10.0;
    double Q_kappa   = 1.0;
    double R_dkappa  = 0.1;
    double kappa_max = 0.5;   // 하드 κ 제약
    // ...
};
```

### HENES `LTVMPCParams`
```cpp
struct LTVMPCParams {
    int    N         = 10;
    double dt        = 0.05;
    double L         = 1.04;  // 축거
    double Q_ey      = 10.0;
    double Q_epsi    = 8.0;
    double Q_v       = 1.0;
    double R_delta   = 0.5;
    double R_a       = 0.1;
    double delta_max = 0.9599;  // ±55 deg
    double a_max     = 2.0;
    double a_min     = -3.0;
};
```

---

## 11. 파일 구조 비교

### bisa
```
src/bisa/src/ltv_mpc/
  ltv_types.hpp       ← 타입 정의 (5-state, MPCCommand)
  ltv_model.hpp/cpp   ← buildReferenceProfiles, 정확 이산화
  ltv_cost.hpp/cpp    ← buildQPObjective (Q, R 가중치)
  ltv_mpc.hpp/cpp     ← buildInputConstraints, κ하드제약, 슬랙, computeControl

src/bisa/include/bisa/
  mpc_controller_cpp.hpp  ← VelocityPlanner, MPCControllerCpp (α블렌딩)
  mpc_path_tracker_cpp.hpp ← path_hold, oscillation guard, off_path_recovery
```

### HENES
```
src/jeju/include/jeju_mpc/
  ltv_mpc.hpp              ← 3-state 타입 + LTVMPC 클래스 선언
  mpc_controller_cpp.hpp   ← RefPoint, VehicleState, MPCControllerCpp
  mpc_path_tracker_cpp.hpp ← MPCPathTrackerCpp (ROS 노드)

src/jeju/src/
  ltv_mpc.cpp              ← buildCondensed + OSQP solve (모놀리식)
  mpc_controller_cpp.cpp   ← computeControl (nearest, Frenet, MPC 호출)
  mpc_path_tracker_cpp.cpp ← 경로 로드, 오돔 콜백, control loop, PI속도
  mpc_path_follower_node.cpp ← main
```

---

## 12. bisa → HENES 포팅 시 수정이 필요한 파일 목록

bisa 방식으로 HENES를 업그레이드하려면 아래 작업이 필요:

| 우선순위 | 작업 | 대상 파일 | 난이도 |
|---------|------|-----------|--------|
| 1 | 5-state 모델로 교체 (dr, θ, κ, θr, κr) | `ltv_mpc.hpp`, `ltv_mpc.cpp` | 높음 |
| 2 | 정확 이산화 (matrix exponential) | `ltv_mpc.cpp` → `ltv_model.cpp` 분리 | 중간 |
| 3 | κ 하드 제약 추가 (Gutjahr 2017) | `ltv_mpc.cpp` | 중간 |
| 4 | VelocityPlanner QP 추가 | `mpc_controller_cpp.hpp/cpp` 신규 | 높음 |
| 5 | α 블렌딩 추가 | `mpc_controller_cpp.cpp` | 낮음 |
| 6 | 출력 형식 변환 (v_cmd, ω → steer_deg, accel) | `mpc_path_tracker_cpp.cpp` | 낮음 |
| 7 | Recovery 동작 추가 (path_hold, oscillation) | `mpc_path_tracker_cpp.cpp` | 중간 |
| 8 | 슬랙 변수 추가 | `ltv_mpc.cpp` | 중간 |

---

## 13. 현재 HENES의 장단점

### 장점
- 구조가 단순 → 디버깅 용이
- 아커만 조향 차량에 직접 적합 (δ 직접 출력)
- OSQP 연동 완료, 실차 검증됨

### 단점
- κ 하드 제약 없음 → 물리적으로 불가능한 곡률 명령 가능
- 속도 계획이 휴리스틱 → 곡선에서 급가속/급감속 발생 가능
- 이산화 오차 (Euler) → 빠른 속도/큰 dt에서 불안정
- 슬랙 없음 → 경로 이탈 시 QP infeasible 가능성
- 복구 로직 미흡 → 큰 e_y 상황에서 진동/발산 가능
