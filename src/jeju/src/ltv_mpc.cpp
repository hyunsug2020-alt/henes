// -*- coding: utf-8 -*-
// ============================================================
// ltv_mpc.cpp  –  Frenet-frame 3-state LTV-MPC  (OSQP 기반)
//
// moraimpc/ltv_mpc.cpp 을 ROS2 jeju 패키지로 이식
//   - namespace jeju_mpc 추가
//   - MPCParams → LTVMPCParams, MPCResult → LTVMPCResult
// ============================================================
#include "jeju_mpc/ltv_mpc.hpp"

#include <osqp.h>
#include <algorithm>
#include <cmath>
#include <iostream>

namespace jeju_mpc
{

LTVMPC::LTVMPC(const LTVMPCParams & p) : p_(p) {}

// ─────────────────────────────────────────────────────────────
//  밀집 행렬 → CSC (상삼각, P 행렬용)
// ─────────────────────────────────────────────────────────────
static csc * toCSC_upper(const Eigen::MatrixXd & M,
                          std::vector<c_int> &   Ap,
                          std::vector<c_int> &   Ai,
                          std::vector<c_float> & Ax)
{
    int n = static_cast<int>(M.rows());
    Ap.clear(); Ai.clear(); Ax.clear();
    Ap.push_back(0);
    for (int col = 0; col < n; ++col) {
        for (int row = 0; row <= col; ++row) {
            double v = M(row, col);
            if (std::abs(v) > 1e-12) {
                Ai.push_back(row);
                Ax.push_back(static_cast<c_float>(v));
            }
        }
        Ap.push_back(static_cast<c_int>(Ai.size()));
    }
    return csc_matrix(n, n,
                      static_cast<c_int>(Ai.size()),
                      Ax.data(), Ai.data(), Ap.data());
}

// ─────────────────────────────────────────────────────────────
//  밀집 행렬 → CSC (전체, A 행렬용)
// ─────────────────────────────────────────────────────────────
static csc * toCSC_full(const Eigen::MatrixXd & M,
                         std::vector<c_int> &   Ap,
                         std::vector<c_int> &   Ai,
                         std::vector<c_float> & Ax)
{
    int rows = static_cast<int>(M.rows());
    int cols = static_cast<int>(M.cols());
    Ap.clear(); Ai.clear(); Ax.clear();
    Ap.push_back(0);
    for (int col = 0; col < cols; ++col) {
        for (int row = 0; row < rows; ++row) {
            double v = M(row, col);
            if (std::abs(v) > 1e-12) {
                Ai.push_back(row);
                Ax.push_back(static_cast<c_float>(v));
            }
        }
        Ap.push_back(static_cast<c_int>(Ai.size()));
    }
    return csc_matrix(rows, cols,
                      static_cast<c_int>(Ai.size()),
                      Ax.data(), Ai.data(), Ap.data());
}

// ─────────────────────────────────────────────────────────────
//  Condensed 행렬 생성
//  X = Phi_x0 * x0 + Theta * U + FF
// ─────────────────────────────────────────────────────────────
void LTVMPC::buildCondensed(const std::vector<double> & kappas,
                              const std::vector<double> & v_refs,
                              Eigen::MatrixXd & Phi_x0,
                              Eigen::MatrixXd & Theta,
                              Eigen::VectorXd & FF,
                              Eigen::VectorXd & X_ref)
{
    int N  = p_.N, n = n_, m = m_;
    int Nn = N * n, Nm = N * m;

    std::vector<Eigen::MatrixXd> A(N), B(N);
    std::vector<Eigen::VectorXd> f(N);
    X_ref.resize(Nn);

    for (int k = 0; k < N; ++k) {
        double vr    = v_refs[k];
        double kappa = kappas[k];
        double dt    = p_.dt, L = p_.L;

        A[k] = Eigen::MatrixXd::Identity(n, n);
        A[k](0, 1) = vr * dt;

        B[k] = Eigen::MatrixXd::Zero(n, m);
        B[k](1, 0) = vr * dt / L;
        B[k](2, 1) = dt;

        // 선형화 잔여항
        f[k] = Eigen::VectorXd::Zero(n);
        f[k](1) = -vr * kappa * dt;

        // 참조 상태: e_y=0, e_psi=0, v=v_ref
        X_ref.segment(k * n, n) << 0.0, 0.0, vr;
    }

    // Phi_x0 (Nn×n): Phi_x0[k] = A[k-1]…A[0]
    Phi_x0.resize(Nn, n);
    Eigen::MatrixXd Psi = Eigen::MatrixXd::Identity(n, n);
    for (int k = 0; k < N; ++k) {
        Psi = A[k] * Psi;
        Phi_x0.block(k * n, 0, n, n) = Psi;
    }

    // Theta (Nn×Nm)
    Theta.setZero(Nn, Nm);
    for (int j = 0; j < N; ++j) {
        Theta.block(j * n, j * m, n, m) = B[j];
        for (int i = j + 1; i < N; ++i) {
            Theta.block(i * n, j * m, n, m) =
                A[i] * Theta.block((i - 1) * n, j * m, n, m);
        }
    }

    // FF (Nn)
    FF.resize(Nn);
    FF.segment(0, n) = f[0];
    for (int i = 1; i < N; ++i) {
        FF.segment(i * n, n) = A[i] * FF.segment((i - 1) * n, n) + f[i];
    }
}

// ─────────────────────────────────────────────────────────────
//  MPC Solve
// ─────────────────────────────────────────────────────────────
LTVMPCResult LTVMPC::solve(const Eigen::Vector3d &     x0,
                             const std::vector<double> & kappas,
                             const std::vector<double> & v_refs)
{
    LTVMPCResult result;
    int N  = p_.N, n = n_, m = m_;
    int Nm = N * m;

    Eigen::MatrixXd Phi_x0, Theta;
    Eigen::VectorXd FF, X_ref;
    buildCondensed(kappas, v_refs, Phi_x0, Theta, FF, X_ref);

    Eigen::VectorXd E = Phi_x0 * x0 + FF - X_ref;

    // 가중치 행렬
    int Nn = N * n;
    Eigen::MatrixXd Q_bar = Eigen::MatrixXd::Zero(Nn, Nn);
    Eigen::Matrix3d Q, QN;
    Q  << p_.Q_ey, 0,         0,
          0,       p_.Q_epsi, 0,
          0,       0,         p_.Q_v;
    QN << p_.QN_ey, 0,          0,
          0,        p_.QN_epsi, 0,
          0,        0,          p_.QN_v;
    for (int k = 0; k < N - 1; ++k) {
        Q_bar.block(k * n, k * n, n, n) = Q;
    }
    Q_bar.block((N - 1) * n, (N - 1) * n, n, n) = QN;

    Eigen::MatrixXd R_bar = Eigen::MatrixXd::Zero(Nm, Nm);
    Eigen::Matrix2d R;
    R << p_.R_delta, 0, 0, p_.R_a;
    for (int k = 0; k < N; ++k) {
        R_bar.block(k * m, k * m, m, m) = R;
    }

    // QP 행렬: cost = U^T*H*U + 2*f_cost^T*U
    Eigen::MatrixXd H = Theta.transpose() * Q_bar * Theta + R_bar;
    H = 0.5 * (H + H.transpose());
    Eigen::VectorXd f_cost = Theta.transpose() * Q_bar * E;

    // OSQP: 0.5*U^T*P*U + q^T*U  →  P=2H, q=2*f_cost
    Eigen::MatrixXd P_mat = 2.0 * H;

    // 입력 제약
    std::vector<c_float> q_osqp(Nm), l_osqp(Nm), u_osqp(Nm);
    for (int k = 0; k < N; ++k) {
        q_osqp[k * m + 0] = static_cast<c_float>(2.0 * f_cost(k * m + 0));
        q_osqp[k * m + 1] = static_cast<c_float>(2.0 * f_cost(k * m + 1));
        l_osqp[k * m + 0] = static_cast<c_float>(p_.delta_min);
        u_osqp[k * m + 0] = static_cast<c_float>(p_.delta_max);
        l_osqp[k * m + 1] = static_cast<c_float>(p_.a_min);
        u_osqp[k * m + 1] = static_cast<c_float>(p_.a_max);
    }

    std::vector<c_int> Pp, Pi; std::vector<c_float> Px;
    std::vector<c_int> Ap, Ai; std::vector<c_float> Ax;
    csc * P_csc = toCSC_upper(P_mat, Pp, Pi, Px);
    Eigen::MatrixXd I_mat = Eigen::MatrixXd::Identity(Nm, Nm);
    csc * A_csc = toCSC_full(I_mat, Ap, Ai, Ax);

    OSQPData data;
    data.n = Nm; data.m = Nm;
    data.P = P_csc; data.q = q_osqp.data();
    data.A = A_csc; data.l = l_osqp.data(); data.u = u_osqp.data();

    OSQPSettings settings;
    osqp_set_default_settings(&settings);
    settings.verbose  = 0;
    settings.eps_abs  = 1e-4;
    settings.eps_rel  = 1e-4;
    settings.max_iter = 2000;

    OSQPWorkspace * ws = nullptr;
    if (osqp_setup(&ws, &data, &settings) != 0 || ws == nullptr) {
        c_free(P_csc); c_free(A_csc);
        return result;
    }

    osqp_solve(ws);

    if (ws->info->status_val == OSQP_SOLVED ||
        ws->info->status_val == OSQP_SOLVED_INACCURATE)
    {
        result.steering = std::max(p_.delta_min,
                          std::min(p_.delta_max,
                          static_cast<double>(ws->solution->x[0])));
        result.accel    = std::max(p_.a_min,
                          std::min(p_.a_max,
                          static_cast<double>(ws->solution->x[1])));
        result.success  = true;
    }

    osqp_cleanup(ws);
    c_free(P_csc);
    c_free(A_csc);

    return result;
}

}  // namespace jeju_mpc
