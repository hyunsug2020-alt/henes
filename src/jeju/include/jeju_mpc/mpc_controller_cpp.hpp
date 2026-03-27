// -*- coding: utf-8 -*-
// ============================================================
// mpc_controller_cpp.hpp
//
// Frenet-frame 3-state LTV-MPC 래퍼
//   상태 [e_y, e_psi, v], 입력 [delta (rad), a (m/s²)]
// OSQP condensed QP, moraimpc 방식 이식
// ============================================================
#pragma once

#include <memory>
#include <vector>

#include <Eigen/Dense>

#include "jeju_mpc/ltv_mpc.hpp"

namespace jeju_mpc
{

// ── 파라미터 ──────────────────────────────────────────────────
struct MPCControllerParams
{
    double wheelbase_m        {1.04};
    double dt                 {0.05};
    int    prediction_horizon {10};
    double max_steer_deg      {55.0};

    // LTV-MPC 비용 가중치
    double Q_ey     {10.0};
    double Q_epsi   { 8.0};
    double Q_v      { 1.0};
    double QN_ey    {20.0};
    double QN_epsi  {16.0};
    double QN_v     { 2.0};
    double R_delta  { 0.5};
    double R_a      { 0.1};
    double a_max    { 2.0};
    double a_min    {-3.0};

    // 곡률별 속도 감속: v_ref = |v_cmd| / (1 + factor * |kappa|)
    double kappa_speed_factor {3.0};
};

// ── 참조 경로 점 ─────────────────────────────────────────────
struct RefPoint
{
    double x       {0.0};
    double y       {0.0};
    double yaw     {0.0};
    double kappa_r {0.0};
};

// ── 차량 상태 ────────────────────────────────────────────────
struct VehicleState
{
    double x       {0.0};
    double y       {0.0};
    double yaw     {0.0};
    double kappa   {0.0};
    double vx      {0.0};
    double vy      {0.0};
    double omega   {0.0};
    double delta_c {0.0};
    // 아래 필드는 하위 호환성 유지용 (LTV-MPC에서는 미사용)
    double omega_f {0.0};
    double omega_r {0.0};
};

// ── 제어 출력 ────────────────────────────────────────────────
struct MPCControlOutput
{
    double curvature  {0.0};  // [1/m]
    double steer_deg  {0.0};  // [deg]
    double torque_cmd {0.0};  // LTV-MPC accel [m/s²]
    double cost       {0.0};
};

// ── 컨트롤러 클래스 ──────────────────────────────────────────
class MPCControllerCpp
{
public:
    MPCControllerCpp();

    void updateParameters(const MPCControllerParams & params);

    MPCControlOutput computeControl(
        const VehicleState &          state,
        const std::vector<RefPoint> & path,
        int                           nearest_idx,
        double                        speed_mps);

    static double normalizeAngle(double angle);

private:
    MPCControllerParams    params_;
    std::unique_ptr<LTVMPC> mpc_;
};

}  // namespace jeju_mpc
