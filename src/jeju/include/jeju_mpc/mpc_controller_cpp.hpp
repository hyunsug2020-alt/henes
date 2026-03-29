// -*- coding: utf-8 -*-
// ============================================================
// mpc_controller_cpp.hpp
//
// bisa-style Frenet 3-state LTV-MPC 래퍼
//   상태 [e_y, e_psi, κ],  입력 dκ/dt
//   VelocityPlanner: a_lat 기반 속도 프로파일
//   α-블렌딩: κ_cmd = α·κ_prev + (1-α)·κ_new
//   출력: κ → δ = atan(L·κ) [deg]
// ============================================================
#pragma once

#include <memory>
#include <vector>

#include <Eigen/Dense>

#include "jeju_mpc/ltv_mpc.hpp"

namespace jeju_mpc
{

// ── 컨트롤러 파라미터 ─────────────────────────────────────────
struct MPCControllerParams
{
    double wheelbase_m        {1.04};
    double dt                 {0.1};
    int    prediction_horizon {15};
    double max_steer_deg      {55.0};

    // bisa-style 비용 가중치
    double Q_ey        {100.0};
    double Q_epsi      { 10.0};
    double Q_kappa     {  1.0};
    double QN_ey       {200.0};
    double QN_epsi     { 20.0};
    double QN_kappa    {  2.0};
    double R_dkappa    {  0.5};

    // 하드 제약
    double kappa_max   {1.37};   // [1/m]
    double dkappa_max  {1.0};    // [1/m/s]

    // VelocityPlanner
    double a_lat_max   {1.0};    // 최대 횡가속도 [m/s²]

    // α-블렌딩 (0=신호만 사용, 1=이전만 유지)
    double kappa_alpha {0.40};
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
    double kappa   {0.0};   // 현재 곡률 [1/m]
    double vx      {0.0};
    double vy      {0.0};
    double omega   {0.0};
    double delta_c {0.0};   // 현재 조향각 [rad]
    // 하위 호환
    double omega_f {0.0};
    double omega_r {0.0};
};

// ── 제어 출력 ────────────────────────────────────────────────
struct MPCControlOutput
{
    double curvature  {0.0};  // κ_cmd [1/m]
    double steer_deg  {0.0};  // δ = atan(L·κ) [deg]
    double torque_cmd {0.0};  // (속도 PI 제어기 입력 – 미사용, 호환용)
    double cost       {0.0};
    double e_y        {0.0};  // 횡방향 오차 [m]
    double e_psi      {0.0};  // 헤딩 오차 [rad]
    int    nearest_idx{0};
};

// ── VelocityPlanner ──────────────────────────────────────────
// a_lat 제약 기반 속도 프로파일 (분석적 풀이)
//   v_k ≤ sqrt(a_lat_max / max(|κr_k|, ε))
class VelocityPlanner
{
public:
    // kappas  : 경로 곡률 (size N)
    // v_des   : 목표 속도 [m/s]
    // a_lat   : 최대 횡가속도 [m/s²]
    // returns : v_refs (size N), 각 스텝별 참조 속도
    static std::vector<double> compute(
        const std::vector<double> & kappas,
        double v_des,
        double a_lat_max);
};

// ── 컨트롤러 클래스 ──────────────────────────────────────────
class MPCControllerCpp
{
public:
    MPCControllerCpp();

    void updateParameters(const MPCControllerParams & params);

    MPCControlOutput computeControl(
        const VehicleState          & state,
        const std::vector<RefPoint> & path,
        int                           nearest_idx,
        double                        speed_mps);

    static double normalizeAngle(double angle);

private:
    MPCControllerParams     params_;
    std::unique_ptr<LTVMPC> mpc_;
    double                  kappa_prev_{0.0};
    bool                    has_prev_kappa_{false};
};

}  // namespace jeju_mpc
