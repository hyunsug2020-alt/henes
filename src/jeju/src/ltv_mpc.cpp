// -*- coding: utf-8 -*-
// ============================================================
// ltv_mpc.cpp  –  Frenet-frame 3-state LTV-MPC  (bisa 방식)
//
// 상태  x(3) = [e_y, e_psi, κ]
// 입력  u(1) = dκ/dt
//
// 이산화: Ac 가 멱영(nilpotent) → expm(Ac·dt) 닫힌형 정확 계산
//   Ac = [0, v, 0]    →  Ac² = [0, 0, v²]  →  Ac³ = 0
//        [0, 0, v]             [0, 0, 0 ]
//        [0, 0, 0]             [0, 0, 0 ]
//   Ad = I + Ac·dt + Ac²·dt²/2  (정확)
//   Bd = ∫₀^dt Ad(τ)·Bc dτ      (정확)
//   fd = ∫₀^dt Ad(τ)·fc dτ      (정확)
//
// 제약:
//   1) dκ/dt 박스: |u_k| ≤ dκ_max
//   2) κ 하드 제약 (Gutjahr 2017): κ_min ≤ κ_k ≤ κ_max
// ============================================================
#include "jeju_mpc/ltv_mpc.hpp"

#include <osqp.h>
#include <algorithm>
#include <cmath>
#include <vector>

namespace jeju_mpc
{

// ─────────────────────────────────────────────────────────────
//  CSC 변환 헬퍼
// ─────────────────────────────────────────────────────────────
static csc * toCSC_upper(const Eigen::MatrixXd & M,
                          std::vector<c_int>   & Ap,
                          std::vector<c_int>   & Ai,
                          std::vector<c_float> & Ax)
{
    int n = static_cast<int>(M.rows());
    Ap.clear(); Ai.clear(); Ax.clear();
    Ap.push_back(0);
    for (int col = 0; col < n; ++col) {
        for (int row = 0; row <= col; ++row) {
            double v = M(row, col);
            if (std::abs(v) > 1e-12) { Ai.push_back(row); Ax.push_back(static_cast<c_float>(v)); }
        }
        Ap.push_back(static_cast<c_int>(Ai.size()));
    }
    return csc_matrix(n, n, static_cast<c_int>(Ai.size()), Ax.data(), Ai.data(), Ap.data());
}

static csc * toCSC_full(const Eigen::MatrixXd & M,
                         std::vector<c_int>   & Ap,
                         std::vector<c_int>   & Ai,
                         std::vector<c_float> & Ax)
{
    int rows = static_cast<int>(M.rows());
    int cols = static_cast<int>(M.cols());
    Ap.clear(); Ai.clear(); Ax.clear();
    Ap.push_back(0);
    for (int col = 0; col < cols; ++col) {
        for (int row = 0; row < rows; ++row) {
            double v = M(row, col);
            if (std::abs(v) > 1e-12) { Ai.push_back(row); Ax.push_back(static_cast<c_float>(v)); }
        }
        Ap.push_back(static_cast<c_int>(Ai.size()));
    }
    return csc_matrix(rows, cols, static_cast<c_int>(Ai.size()), Ax.data(), Ai.data(), Ap.data());
}

// ─────────────────────────────────────────────────────────────
LTVMPC::LTVMPC(const LTVMPCConfig & cfg) : cfg_(cfg) {}

// ─────────────────────────────────────────────────────────────
//  닫힌형 정확 이산화 (nilpotent Ac)
//
//  Ac = [0, v, 0; 0, 0, v; 0, 0, 0]
//  Bc = [0; 0; 1]
//  fc = [0; -v·κr; 0]    (선형화 잔여항)
//
//  Ad = I + Ac·dt + Ac²·dt²/2
//     = [1, v·dt,  v²·dt²/2]
//       [0, 1,     v·dt     ]
//       [0, 0,     1        ]
//
//  Bd = ∫₀^dt Ad(τ)·Bc dτ = [v²·dt³/6; v·dt²/2; dt]
//
//  fd = ∫₀^dt Ad(τ)·fc dτ = [-v²·κr·dt²/2; -v·κr·dt; 0]
// ─────────────────────────────────────────────────────────────
LTVMPC::DiscreteMats LTVMPC::discretize(double v, double kappa_r) const
{
    const double dt  = cfg_.dt;
    const double vdt = v * dt;
    const double v2  = v * v;

    DiscreteMats m;

    m.Ad << 1.0,  vdt,          0.5 * v2 * dt * dt,
            0.0,  1.0,          vdt,
            0.0,  0.0,          1.0;

    m.Bd << v2 * dt * dt * dt / 6.0,
            v * dt * dt * 0.5,
            dt;

    m.fd << -0.5 * v2 * kappa_r * dt * dt,
            -v * kappa_r * dt,
            0.0;

    return m;
}

// ─────────────────────────────────────────────────────────────
//  배치 동역학: X = S_x·x0 + S_u·U + d_ff
//    X  (3N×1) = [x_1; x_2; ... ; x_N]
//    U  ( N×1) = [u_0; u_1; ... ; u_{N-1}]
// ─────────────────────────────────────────────────────────────
void LTVMPC::buildBatch(
    const std::vector<double> & kappa_refs,
    const std::vector<double> & v_refs,
    Eigen::MatrixXd & S_x,
    Eigen::MatrixXd & S_u,
    Eigen::VectorXd & d_ff) const
{
    const int N  = cfg_.N;
    const int nx = 3;

    S_x.setZero(N * nx, nx);
    S_u.setZero(N * nx, N);
    d_ff.setZero(N * nx);

    // 각 스텝 이산 행렬 미리 계산
    std::vector<DiscreteMats> mats(N);
    for (int k = 0; k < N; ++k) {
        mats[k] = discretize(v_refs[k], kappa_refs[k]);
    }

    // S_x, d_ff: 점화 Phi_k = Ad_k · Phi_{k-1}
    Eigen::Matrix3d Phi  = Eigen::Matrix3d::Identity();
    Eigen::Vector3d dacc = Eigen::Vector3d::Zero();
    for (int k = 0; k < N; ++k) {
        Phi  = mats[k].Ad * Phi;
        dacc = mats[k].Ad * dacc + mats[k].fd;
        S_x.block(k * nx, 0, nx, nx) = Phi;
        d_ff.segment(k * nx, nx)     = dacc;
    }

    // S_u: 열 j (u_j 기여)
    //   S_u[k, j] = Ad_k · ... · Ad_{j+1} · Bd_j   (k > j)
    //             = Bd_j                             (k == j)
    for (int j = 0; j < N; ++j) {
        Eigen::Vector3d col = mats[j].Bd;
        S_u.block(j * nx, j, nx, 1) = col;
        for (int k = j + 1; k < N; ++k) {
            col = mats[k].Ad * col;
            S_u.block(k * nx, j, nx, 1) = col;
        }
    }
}

// ─────────────────────────────────────────────────────────────
//  MPC Solve
// ─────────────────────────────────────────────────────────────
LTVMPCResult LTVMPC::solve(
    const Eigen::Vector3d     & x0,
    const std::vector<double> & kappa_refs,
    const std::vector<double> & v_refs)
{
    LTVMPCResult result;
    const int N  = cfg_.N;
    const int nx = 3;

    // ── 배치 동역학 ────────────────────────────────────────
    Eigen::MatrixXd S_x, S_u;
    Eigen::VectorXd d_ff;
    buildBatch(kappa_refs, v_refs, S_x, S_u, d_ff);

    // ── 참조 상태 (e_y=0, e_psi=0, κ=κr) ──────────────────
    Eigen::VectorXd X_ref = Eigen::VectorXd::Zero(N * nx);
    for (int k = 0; k < N; ++k) {
        X_ref(k * nx + 2) = kappa_refs[k];
    }

    Eigen::VectorXd E = S_x * x0 + d_ff - X_ref;

    // ── 가중치 행렬 ────────────────────────────────────────
    Eigen::Matrix3d Q, QN;
    Q  << cfg_.Q_ey,  0.0,           0.0,
          0.0,         cfg_.Q_epsi,   0.0,
          0.0,         0.0,           cfg_.Q_kappa;
    QN << cfg_.QN_ey, 0.0,            0.0,
          0.0,         cfg_.QN_epsi,   0.0,
          0.0,         0.0,            cfg_.QN_kappa;

    Eigen::MatrixXd Q_bar = Eigen::MatrixXd::Zero(N * nx, N * nx);
    for (int k = 0; k < N - 1; ++k) {
        Q_bar.block(k * nx, k * nx, nx, nx) = Q;
    }
    Q_bar.block((N - 1) * nx, (N - 1) * nx, nx, nx) = QN;

    // ── QP 목적 함수: 0.5·U^T·P·U + q^T·U ─────────────────
    Eigen::MatrixXd H = S_u.transpose() * Q_bar * S_u
                      + cfg_.R_dkappa * Eigen::MatrixXd::Identity(N, N);
    H = 0.5 * (H + H.transpose());

    Eigen::VectorXd f_cost = S_u.transpose() * Q_bar * E;

    Eigen::MatrixXd P_mat = 2.0 * H;    // OSQP: min 0.5·U^T·P·U + q^T·U

    // ── κ 추출 행렬 C_kappa (N×3N): C[k, 3k+2] = 1 ────────
    // κ_vec = C_kappa · (S_x·x0 + S_u·U + d_ff)
    Eigen::MatrixXd C_kappa = Eigen::MatrixXd::Zero(N, N * nx);
    for (int k = 0; k < N; ++k) {
        C_kappa(k, k * nx + 2) = 1.0;
    }
    Eigen::VectorXd kappa_offset = C_kappa * (S_x * x0 + d_ff);

    // ── 제약 행렬 A_con (2N×N) ─────────────────────────────
    //   [ I_N           ]  (dκ/dt 박스)
    //   [ C_kappa · S_u ]  (κ 하드 제약, Gutjahr 2017)
    Eigen::MatrixXd A_con = Eigen::MatrixXd::Zero(2 * N, N);
    A_con.topRows(N)    = Eigen::MatrixXd::Identity(N, N);
    A_con.bottomRows(N) = C_kappa * S_u;

    // 상하한
    const double km  = cfg_.kappa_max;
    const double dm  = cfg_.dkappa_max;
    std::vector<c_float> l_osqp(2 * N), u_osqp(2 * N);
    for (int k = 0; k < N; ++k) {
        l_osqp[k]     = static_cast<c_float>(-dm);
        u_osqp[k]     = static_cast<c_float>(+dm);
        l_osqp[N + k] = static_cast<c_float>(-km - kappa_offset(k));
        u_osqp[N + k] = static_cast<c_float>(+km - kappa_offset(k));
    }

    // ── OSQP 세팅 ──────────────────────────────────────────
    std::vector<c_float> q_osqp(N);
    for (int k = 0; k < N; ++k) {
        q_osqp[k] = static_cast<c_float>(2.0 * f_cost(k));
    }

    std::vector<c_int> Pp, Pi; std::vector<c_float> Px;
    std::vector<c_int> Ap, Ai; std::vector<c_float> Ax;
    csc * P_csc = toCSC_upper(P_mat, Pp, Pi, Px);
    csc * A_csc = toCSC_full(A_con, Ap, Ai, Ax);

    OSQPData data;
    data.n = N;       data.m = 2 * N;
    data.P = P_csc;   data.q = q_osqp.data();
    data.A = A_csc;   data.l = l_osqp.data(); data.u = u_osqp.data();

    OSQPSettings settings;
    osqp_set_default_settings(&settings);
    settings.verbose  = 0;
    settings.eps_abs  = 1e-4;
    settings.eps_rel  = 1e-4;
    settings.max_iter = 2000;
    settings.warm_start    = 1;

    OSQPWorkspace * ws = nullptr;
    if (osqp_setup(&ws, &data, &settings) != 0 || ws == nullptr) {
        c_free(P_csc); c_free(A_csc);
        return result;
    }
    osqp_solve(ws);

    if (ws->info->status_val == OSQP_SOLVED ||
        ws->info->status_val == OSQP_SOLVED_INACCURATE)
    {
        result.kappa_cmd = std::clamp(
            x0(2) + static_cast<double>(ws->solution->x[0]) * cfg_.dt,
            -km, km);
        result.cost    = ws->info->obj_val;
        result.success = true;
    }

    osqp_cleanup(ws);
    c_free(P_csc);
    c_free(A_csc);

    return result;
}

}  // namespace jeju_mpc
