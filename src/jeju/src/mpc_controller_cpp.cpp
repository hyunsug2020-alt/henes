// -*- coding: utf-8 -*-
// ============================================================
// mpc_controller_cpp.cpp
//
// bisa-style Frenet 3-state LTV-MPC 래퍼
//   VelocityPlanner (a_lat 기반 속도 프로파일)
//   α-블렌딩 (κ_cmd = α·κ_prev + (1-α)·κ_new)
//   κ → δ = atan(L·κ) 변환 후 도 단위 출력
// ============================================================
#include "jeju_mpc/mpc_controller_cpp.hpp"

#include <algorithm>
#include <cmath>

namespace jeju_mpc
{

// ─────────────────────────────────────────────────────────────
//  VelocityPlanner
// ─────────────────────────────────────────────────────────────
std::vector<double> VelocityPlanner::compute(
    const std::vector<double> & kappas,
    double v_des,
    double a_lat_max)
{
    const int N = static_cast<int>(kappas.size());
    std::vector<double> v_refs(N);

    // v_max_k = sqrt(a_lat_max / max(|κr_k|, ε))
    // 최소 0.3 m/s (정지 방지)
    constexpr double kEps = 1e-4;
    for (int k = 0; k < N; ++k) {
        const double kabs = std::max(std::abs(kappas[k]), kEps);
        const double v_lat = std::sqrt(a_lat_max / kabs);
        v_refs[k] = std::clamp(v_des, 0.3, v_lat);
    }
    return v_refs;
}

// ─────────────────────────────────────────────────────────────
//  MPCControllerCpp
// ─────────────────────────────────────────────────────────────
MPCControllerCpp::MPCControllerCpp() = default;

void MPCControllerCpp::updateParameters(const MPCControllerParams & params)
{
    params_ = params;

    LTVMPCConfig cfg;
    cfg.N          = std::max(2, params_.prediction_horizon);
    cfg.dt         = params_.dt;
    cfg.L          = std::max(0.1, params_.wheelbase_m);
    cfg.Q_ey       = params_.Q_ey;
    cfg.Q_epsi     = params_.Q_epsi;
    cfg.Q_kappa    = params_.Q_kappa;
    cfg.QN_ey      = params_.QN_ey;
    cfg.QN_epsi    = params_.QN_epsi;
    cfg.QN_kappa   = params_.QN_kappa;
    cfg.R_dkappa   = params_.R_dkappa;
    cfg.kappa_max  = std::tan(params_.max_steer_deg * M_PI / 180.0)
                     / std::max(params_.wheelbase_m, 0.1);
    cfg.dkappa_max = params_.dkappa_max;

    // 파라미터에 kappa_max 가 명시된 경우 그 값 사용
    if (params_.kappa_max > 0.0) {
        cfg.kappa_max = params_.kappa_max;
    }

    mpc_ = std::make_unique<LTVMPC>(cfg);
    has_prev_kappa_ = false;
}

double MPCControllerCpp::normalizeAngle(double angle)
{
    while (angle >  M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

MPCControlOutput MPCControllerCpp::computeControl(
    const VehicleState          & state,
    const std::vector<RefPoint> & path,
    int                           nearest_idx,
    double                        speed_mps)
{
    MPCControlOutput out;
    if (!mpc_ || path.empty()) return out;

    const int N     = params_.prediction_horizon;
    const int n_pts = static_cast<int>(path.size());
    const int idx   = std::clamp(nearest_idx, 0, n_pts - 1);

    // ── Frenet 오차 계산 ─────────────────────────────────
    const RefPoint & ref = path[idx];
    const double dx   = state.x - ref.x;
    const double dy   = state.y - ref.y;
    const double e_y  = -std::sin(ref.yaw) * dx + std::cos(ref.yaw) * dy;
    const double e_psi = normalizeAngle(state.yaw - ref.yaw);

    // 현재 곡률 (조향각 피드백)
    const double kappa_now = state.kappa;

    const Eigen::Vector3d x0(e_y, e_psi, kappa_now);

    // ── 예측 구간 곡률 / 참조 속도 ───────────────────────
    const double v_cmd = std::max(0.3, std::abs(speed_mps));
    std::vector<double> kappa_refs(N), v_refs_raw(N);
    for (int k = 0; k < N; ++k) {
        const int ki = std::min(idx + k, n_pts - 1);
        kappa_refs[k]  = path[ki].kappa_r;
        v_refs_raw[k]  = v_cmd;
    }

    // VelocityPlanner: a_lat 제약 기반 속도 감속
    const auto v_refs = VelocityPlanner::compute(
        kappa_refs, v_cmd, params_.a_lat_max);

    // ── LTV-MPC 풀기 ─────────────────────────────────────
    const LTVMPCResult res = mpc_->solve(x0, kappa_refs, v_refs);
    if (!res.success) return out;

    // ── α-블렌딩 ─────────────────────────────────────────
    double kappa_cmd = res.kappa_cmd;
    if (has_prev_kappa_) {
        const double alpha = std::clamp(params_.kappa_alpha, 0.0, 1.0);
        kappa_cmd = alpha * kappa_prev_ + (1.0 - alpha) * kappa_cmd;
    }
    kappa_prev_     = kappa_cmd;
    has_prev_kappa_ = true;

    // kappa_max 클램프
    const double km = (params_.kappa_max > 0.0)
                      ? params_.kappa_max
                      : std::tan(params_.max_steer_deg * M_PI / 180.0)
                        / std::max(params_.wheelbase_m, 0.1);
    kappa_cmd = std::clamp(kappa_cmd, -km, km);

    // κ → δ (아커만 변환)
    const double delta_rad = std::atan(kappa_cmd * std::max(params_.wheelbase_m, 0.1));

    out.curvature   = kappa_cmd;
    out.steer_deg   = delta_rad * 180.0 / M_PI;
    out.torque_cmd  = 0.0;   // 속도는 tracker의 PI 제어기가 담당
    out.cost        = res.cost;
    out.e_y         = e_y;
    out.e_psi       = e_psi;
    out.nearest_idx = nearest_idx;

    return out;
}

}  // namespace jeju_mpc
