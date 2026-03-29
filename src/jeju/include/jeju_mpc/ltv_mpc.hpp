// -*- coding: utf-8 -*-
// ============================================================
// ltv_mpc.hpp  –  Frenet-frame 3-state LTV-MPC  (bisa 방식)
//
// 상태  x(3) = [e_y (m), e_psi (rad), κ (1/m)]
// 입력  u(1) = dκ/dt  [1/m/s]
//
// 선형화 연속 모델 (자전거 Frenet):
//   ė_y   =  v_ref · e_psi
//   ė_psi =  v_ref · κ  −  v_ref · κr(t)
//   κ̇    =  u
//
// 이산화: nilpotent Ac 닫힌형 행렬 지수 (정확)
// 제약  : dκ/dt 박스 + κ 하드 제약 (Gutjahr 2017)
// ============================================================
#pragma once
#include <Eigen/Dense>
#include <vector>

namespace jeju_mpc
{

// ── 설정 파라미터 ────────────────────────────────────────────
struct LTVMPCConfig
{
    int    N          = 15;
    double dt         = 0.1;
    double L          = 1.04;     // 축거 [m] (κ→δ 변환에 사용)

    // 상태 비용 가중치 [e_y, e_psi, κ]
    double Q_ey       = 100.0;
    double Q_epsi     =  10.0;
    double Q_kappa    =   1.0;
    // 종단 비용
    double QN_ey      = 200.0;
    double QN_epsi    =  20.0;
    double QN_kappa   =   2.0;
    // 입력 비용
    double R_dkappa   =   0.5;

    // 하드 제약
    double kappa_max  = 1.37;    // [1/m] ≈ tan(55°) / 1.04
    double dkappa_max = 1.0;     // [1/m/s]
};

// ── 결과 ─────────────────────────────────────────────────────
struct LTVMPCResult
{
    double kappa_cmd = 0.0;   // 곡률 명령 [1/m]
    double cost      = 0.0;
    bool   success   = false;
};

// ── LTVMPC 클래스 ─────────────────────────────────────────────
class LTVMPC
{
public:
    explicit LTVMPC(const LTVMPCConfig & cfg);

    // x0          : [e_y (m), e_psi (rad), kappa_vehicle (1/m)]
    // kappa_refs  : 예측 구간별 참조 곡률 (size >= N)
    // v_refs      : 예측 구간별 참조 속도 (size >= N)  [m/s]
    LTVMPCResult solve(
        const Eigen::Vector3d     & x0,
        const std::vector<double> & kappa_refs,
        const std::vector<double> & v_refs);

private:
    LTVMPCConfig cfg_;

    struct DiscreteMats {
        Eigen::Matrix3d Ad;
        Eigen::Vector3d Bd;
        Eigen::Vector3d fd;
    };

    // nilpotent Ac → 닫힌형 정확 이산화
    DiscreteMats discretize(double v_ref, double kappa_r) const;

    // 배치 동역학 구성: X = S_x·x0 + S_u·U + d_ff
    void buildBatch(
        const std::vector<double> & kappa_refs,
        const std::vector<double> & v_refs,
        Eigen::MatrixXd & S_x,
        Eigen::MatrixXd & S_u,
        Eigen::VectorXd & d_ff) const;
};

}  // namespace jeju_mpc
