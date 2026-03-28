// -*- coding: utf-8 -*-
// ============================================================
// mpc_controller_cpp.cpp
//
// Frenet-frame 3-state LTV-MPC
//   x(3) = [e_y, e_psi, v]
//   u(2) = [delta (rad), a (m/s²)]
//
// moraimpc/path_follower_node.cpp 의 egoCallback 로직을
// ROS2 jeju 패키지용 MPCControllerCpp 인터페이스에 이식
// ============================================================
#include "jeju_mpc/mpc_controller_cpp.hpp"

#include <algorithm>
#include <cmath>

namespace jeju_mpc
{

MPCControllerCpp::MPCControllerCpp() = default;

void MPCControllerCpp::updateParameters(const MPCControllerParams & params)
{
    params_ = params;

    LTVMPCParams mp;
    mp.N         = std::max(1, params_.prediction_horizon);
    mp.dt        = params_.dt;
    mp.L         = std::max(0.1, params_.wheelbase_m);
    mp.Q_ey      = params_.Q_ey;
    mp.Q_epsi    = params_.Q_epsi;
    mp.Q_v       = params_.Q_v;
    mp.QN_ey     = params_.QN_ey;
    mp.QN_epsi   = params_.QN_epsi;
    mp.QN_v      = params_.QN_v;
    mp.R_delta   = params_.R_delta;
    mp.R_a       = params_.R_a;
    mp.delta_max = params_.max_steer_deg * M_PI / 180.0;
    mp.delta_min = -mp.delta_max;
    mp.a_max     = params_.a_max;
    mp.a_min     = params_.a_min;

    mpc_ = std::make_unique<LTVMPC>(mp);
}

double MPCControllerCpp::normalizeAngle(double angle)
{
    while (angle >  M_PI) { angle -= 2.0 * M_PI; }
    while (angle < -M_PI) { angle += 2.0 * M_PI; }
    return angle;
}

MPCControlOutput MPCControllerCpp::computeControl(
    const VehicleState &          state,
    const std::vector<RefPoint> & path,
    int                           nearest_idx,
    double                        speed_mps)
{
    MPCControlOutput out;
    if (!mpc_ || path.empty()) {
        return out;
    }

    int N     = params_.prediction_horizon;
    int n_pts = static_cast<int>(path.size());
    int idx   = std::clamp(nearest_idx, 0, n_pts - 1);

    // ── Frenet 오차 계산 ─────────────────────────────────
    const RefPoint & ref = path[idx];
    double psi_ref = ref.yaw;
    double dx = state.x - ref.x;
    double dy = state.y - ref.y;
    // 횡방향 오차 (부호: 참조 경로 왼쪽 +)
    double e_y   = -std::sin(psi_ref) * dx + std::cos(psi_ref) * dy;
    double e_psi = normalizeAngle(state.yaw - psi_ref);
    // 속도 (절대값, 방향은 speed_mps 부호로 처리)
    double v_now = std::hypot(state.vx, state.vy);
    if (v_now < 0.05 && std::abs(speed_mps) > 0.1) {
        v_now = std::abs(speed_mps);
    }

    Eigen::Vector3d x0(e_y, e_psi, v_now);

    // ── 예측 구간 곡률 / 참조 속도 ───────────────────────
    std::vector<double> kappas(N), v_refs(N);
    const double v_cmd = std::max(0.5, std::abs(speed_mps));
    const double kfac  = std::max(0.0, params_.kappa_speed_factor);
    for (int k = 0; k < N; ++k) {
        int ki     = std::min(idx + k, n_pts - 1);
        kappas[k]  = path[ki].kappa_r;
        v_refs[k]  = std::max(0.5, v_cmd / (1.0 + kfac * std::abs(kappas[k])));
    }

    // ── LTV-MPC 풀기 ─────────────────────────────────────
    LTVMPCResult res = mpc_->solve(x0, kappas, v_refs);
    if (!res.success) {
        return out;
    }

    out.steer_deg   = res.steering * 180.0 / M_PI;
    out.curvature   = std::tan(res.steering) / std::max(params_.wheelbase_m, 0.01);
    out.torque_cmd  = res.accel;
    out.cost        = e_y * e_y + e_psi * e_psi;
    out.e_y         = e_y;
    out.e_psi       = e_psi;
    out.nearest_idx = nearest_idx;

    return out;
}

}  // namespace jeju_mpc
