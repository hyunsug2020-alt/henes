// -*- coding: utf-8 -*-
// ============================================================
// ltv_mpc.hpp  –  Frenet-frame 3-state LTV-MPC
//
// 상태  x(3) = [e_y, e_psi, v]
// 입력  u(2) = [delta (rad), a (m/s²)]
//
// moraimpc/ltv_mpc.hpp 을 ROS2 jeju 패키지로 이식
// ============================================================
#pragma once
#include <Eigen/Dense>
#include <vector>

namespace jeju_mpc
{

struct LTVMPCParams {
    int    N         = 10;
    double dt        = 0.05;
    double L         = 1.04;     // 축거 [m]

    double Q_ey      = 10.0;
    double Q_epsi    =  8.0;
    double Q_v       =  1.0;
    double QN_ey     = 20.0;
    double QN_epsi   = 16.0;
    double QN_v      =  2.0;

    double R_delta   =  0.5;
    double R_a       =  0.1;

    double delta_max =  0.9599;  // 55 deg in rad
    double delta_min = -0.9599;
    double a_max     =  2.0;     // [m/s^2]
    double a_min     = -3.0;
};

struct LTVMPCResult {
    double steering = 0.0;   // [rad]
    double accel    = 0.0;   // [m/s^2]
    bool   success  = false;
};

class LTVMPC
{
public:
    explicit LTVMPC(const LTVMPCParams & p);
    ~LTVMPC() = default;

    // x0     : [e_y (m), e_psi (rad), v (m/s)]
    // kappas : 참조 곡률 (size >= N)
    // v_refs : 참조 속도 (size >= N)
    LTVMPCResult solve(const Eigen::Vector3d &     x0,
                       const std::vector<double> & kappas,
                       const std::vector<double> & v_refs);

private:
    LTVMPCParams p_;
    int n_ = 3, m_ = 2;

    void buildCondensed(const std::vector<double> & kappas,
                        const std::vector<double> & v_refs,
                        Eigen::MatrixXd & Phi_x0,
                        Eigen::MatrixXd & Theta,
                        Eigen::VectorXd & FF,
                        Eigen::VectorXd & X_ref);
};

}  // namespace jeju_mpc
