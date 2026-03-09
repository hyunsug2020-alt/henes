# Next: 실전 주행 고도화 우선 항목

## 목적
- 현재 MPC + Pure Pursuit 기반 제어를 실전 환경에서 더 안정적으로 동작시키기
- 실패(`FAILED`, `NO_DATA`, `LOCAL_PATH_TOO_SHORT`) 상황에서도 안전한 동작 보장

## 우선 도입 기능 (요청 순서 반영)

## 1) 상태추정 강화 (`EKF/UKF`)  [기존 제안 6번]

### 왜 필요한가
- 현재 제어기는 센서 노이즈/지연에 민감해 yaw, 곡률, 횡오차가 튀면 QP 실패율이 증가할 수 있음
- 특히 저속-급회전 구간에서 heading/CTE 추정 오차가 커지면 보정 로직이 과하게 반응함

### 적용 포인트
- 추정 상태: \(x, y, \psi, v, \kappa\) (필요 시 slip angle 추가)
- 관측 입력: pose, wheel/imu 기반 yaw-rate, 속도
- 제어기 입력은 raw pose 대신 필터 출력 사용

### 기대 효과
- `FAILED` 빈도 감소
- 조향 진동 감소
- 코너 진입/복귀 구간의 추종 안정화

---

## 2) Fallback 정책 명확화 (`FAILED/NO_DATA/PATH_SHORT` 분리 대응)  [기존 제안 7번]

### 왜 필요한가
- 현재는 상태 라벨은 구분되지만, 운용 관점에서 "어떤 상황에 어떤 제어를 할지" 정책 테이블이 명확히 문서화되어야 함

### 권장 정책 테이블
- `SOLVED`: 정상 MPC 출력 사용
- `FAILED`: 속도 단계 감속 + 안전 조향 제한 + 재시도 카운터
- `LOCAL_PATH_TOO_SHORT`: 즉시 저속 모드 + 경로 재수신 대기
- `NO_DATA`: 안전 정지(브레이크/제로 커맨드)

### 추가 권장
- 상태 전이 히스테리시스(깜빡임 방지)
- 상태별 최소 유지 시간(`dwell time`)
- 이벤트 로그 자동 저장(원인 추적)

### 기대 효과
- 비정상 상황에서 차량 거동 예측 가능
- 현장 디버깅 시간 단축
- 안전성 향상

---

## 3) Stanley 또는 LQR 보조 조향  [기존 제안 1번]

### 왜 필요한가
- Pure Pursuit는 기하학적 직관은 좋지만 일부 상황에서 CTE/heading 오차 수렴이 느리거나 과도할 수 있음
- Stanley/LQR는 횡오차 피드백이 명확해 중저속 안정화에 유리

### 권장 통합 방식
- 기본: MPC
- 보조: Stanley 또는 LQR
- 혼합: \(\omega = (1-\beta)\omega_{mpc} + \beta\omega_{aux}\), \(\beta\)는 오차/속도 기반 가변

### 기대 효과
- 경로 재진입 성능 개선
- S자/급곡선 구간의 좌우 흔들림 감소

---

## 4) 곡률 Feedforward + MPC Feedback  [기존 제안 2번]

### 왜 필요한가
- 코너에서 오차가 생긴 뒤에만 반응하면 늦음
- 기준 경로 곡률을 선제 반영하면 조향이 부드럽고 예측 가능해짐

### 기본 구조
- Feedforward: \(\omega_{ff} = v \cdot \kappa_{ref}\)
- Feedback: MPC가 오차 보정 \(\omega_{fb}\)
- 최종: \(\omega = \omega_{ff} + \omega_{fb}\)

### 기대 효과
- 코너 진입 지연 감소
- 과도 응답(overshoot) 완화
- 동일 추종 성능 대비 조향 effort 감소

---

## 실행 우선순위 (권장)
1. 상태추정(EKF/UKF) + Fallback 정책 정식화
2. Feedforward + MPC feedback 결합
3. Stanley/LQR 보조 조향 A/B 테스트 후 채택

## 성공 지표(KPI) 제안
- `FAILED` 비율 (%)
- 경로 횡오차 RMS (m)
- heading error RMS (rad)
- 급조향 횟수/크기 (\(|\Delta \omega|\))
- recovery 시간 (오프패스 -> 정상 복귀)
