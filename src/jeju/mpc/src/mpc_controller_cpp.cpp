// ============================================================
// mpc_controller_cpp.cpp
//
// Wagner & Normey-Rico (2024) arXiv:2410.12170
// 9-state Dynamic Bicycle Model RTI-NMPC
//
// 상태 x(9) = [px, py, ψ, vx, vy, ω, δc, ωf, ωr]
// 입력 u(2) = [v_delta, T_drive]
//
// 동역학 (논문 식 26-29 + 액추에이터/바퀴 동역학):
//
//   ṗx   = vx*cos(ψ) - vy*sin(ψ)
//   ṗy   = vx*sin(ψ) + vy*cos(ψ)
//   ψ̇    = ω
//   v̇x   = (Fx_f*cos(δc) - Fy_f*sin(δc) + Fx_r)/m + vy*ω
//   v̇y   = (Fx_f*sin(δc) + Fy_f*cos(δc) + Fy_r)/m - vx*ω
//   ω̇    = (lf*(Fx_f*sin(δc) + Fy_f*cos(δc)) - lr*Fy_r)/Iz
//   δ̇c   = v_delta
//   ω̇_f  = (-r_w*Fx_f) / Jw
//   ω̇_r  = (T_drive - r_w*Fx_r) / Jw
//
// 타이어 모델 (선형):
//   슬립각:   α_f = δc - atan2(vy+lf*ω, v_safe)
//             α_r = -atan2(vy-lr*ω, v_safe)
//   횡방향:   Fy_f = Ca*α_f,  Fy_r = Ca*α_r
//   종방향 슬립: σ_f = (ωf*r_w - vx)/v_safe
//              σ_r = (ωr*r_w - vx)/v_safe
//   종방향:   Fx_f = Cx*σ_f,  Fx_r = Cx*σ_r
//
// 전진/후진: vx_cmd 부호, v_safe=max(|vx|, v_min)
// ============================================================
#include "jeju_mpc/mpc_controller_cpp.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <osqp.h>

namespace jeju_mpc
{

namespace
{
constexpr double kPi = 3.14159265358979323846;

// warm start 전역 저장
static std::vector<Eigen::Matrix<double,2,1>> g_dbm_warm_u;
static std::vector<Eigen::Matrix<double,9,1>> g_dbm_warm_x;
static bool                                   g_dbm_warm_valid = false;
static std::vector<double>                    g_fr_warm_kappa;
static bool                                   g_fr_warm_valid  = false;
}  // namespace

MPCControllerCpp::MPCControllerCpp() = default;

void MPCControllerCpp::updateParameters(const MPCControllerParams & params)
{
  params_ = params;
  params_.prediction_horizon = std::max(1, params_.prediction_horizon);
  params_.kappa_samples = std::max(3, params_.kappa_samples);

  g_dbm_warm_valid = false;
  g_fr_warm_valid = false;
  g_dbm_warm_u.clear();
  g_dbm_warm_x.clear();
  g_fr_warm_kappa.clear();
}

double MPCControllerCpp::normalizeAngle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

// ============================================================
// 내부 헬퍼: 타이어 힘 계산
// ============================================================
struct TireForces
{
  double alpha_f;
  double alpha_r;
  double sigma_f;
  double sigma_r;
  double Fy_f;
  double Fy_r;
  double Fx_f;
  double Fx_r;
  double denom_f;
  double denom_r;
  double v_safe;
  double daf_dvy;
  double daf_dom;
  double dar_dvy;
  double dar_dom;
};

static TireForces computeTireForces(
  const Eigen::Matrix<double,9,1> & x,
  const MPCControllerParams & p)
{
  const double vx = x(3);
  const double vy = x(4);
  const double omega = x(5);
  const double dc = x(6);
  const double wf = x(7);
  const double wr = x(8);

  TireForces t;
  t.v_safe = std::max(std::abs(vx), p.v_min_mps);
  t.denom_f = t.v_safe * t.v_safe + (vy + p.lf_m * omega) * (vy + p.lf_m * omega);
  t.denom_r = t.v_safe * t.v_safe + (vy - p.lr_m * omega) * (vy - p.lr_m * omega);
  t.daf_dvy = -t.v_safe / t.denom_f;
  t.daf_dom = -p.lf_m * t.v_safe / t.denom_f;
  t.dar_dvy = -t.v_safe / t.denom_r;
  t.dar_dom = p.lr_m * t.v_safe / t.denom_r;

  t.alpha_f = dc - std::atan2(vy + p.lf_m * omega, t.v_safe);
  t.alpha_r = -std::atan2(vy - p.lr_m * omega, t.v_safe);
  t.Fy_f = p.Ca_N_rad * t.alpha_f;
  t.Fy_r = p.Ca_N_rad * t.alpha_r;

  t.sigma_f = (wf * p.r_w_m - vx) / t.v_safe;
  t.sigma_r = (wr * p.r_w_m - vx) / t.v_safe;
  t.Fx_f = p.Cx_N * t.sigma_f;
  t.Fx_r = p.Cx_N * t.sigma_r;

  return t;
}

// ============================================================
// 논문 식(26-29): ẋ = f(x, u)
// ============================================================
Eigen::Matrix<double,9,1> MPCControllerCpp::dbmF(
  const Eigen::Matrix<double,9,1> & x,
  const Eigen::Matrix<double,2,1> & u) const
{
  const double psi = x(2);
  const double vx = x(3);
  const double vy = x(4);
  const double omega = x(5);
  const double dc = x(6);
  const double v_delta = u(0);
  const double T_dr = u(1);

  const double cos_d = std::cos(dc);
  const double sin_d = std::sin(dc);

  const auto t = computeTireForces(x, params_);

  Eigen::Matrix<double,9,1> xdot;
  xdot(0) = vx * std::cos(psi) - vy * std::sin(psi);
  xdot(1) = vx * std::sin(psi) + vy * std::cos(psi);
  xdot(2) = omega;
  xdot(3) = (t.Fx_f * cos_d - t.Fy_f * sin_d + t.Fx_r) / params_.mass_kg + vy * omega;
  xdot(4) = (t.Fx_f * sin_d + t.Fy_f * cos_d + t.Fy_r) / params_.mass_kg - vx * omega;
  xdot(5) = (params_.lf_m * (t.Fx_f * sin_d + t.Fy_f * cos_d) - params_.lr_m * t.Fy_r)
    / params_.inertia_kgm2;
  xdot(6) = v_delta;
  xdot(7) = (-params_.r_w_m * t.Fx_f) / params_.Jw_kgm2;
  xdot(8) = (T_dr - params_.r_w_m * t.Fx_r) / params_.Jw_kgm2;

  return xdot;
}

// ============================================================
// 논문 식(12-14): Jacobian Ac(9×9), Bc(9×2)
// ============================================================
void MPCControllerCpp::dbmJacobian(
  const Eigen::Matrix<double,9,1> & x,
  const Eigen::Matrix<double,2,1> & /*u*/,
  Eigen::Matrix<double,9,9>       & Ac,
  Eigen::Matrix<double,9,2>       & Bc) const
{
  const double psi = x(2);
  const double vx = x(3);
  const double vy = x(4);
  const double omega = x(5);
  const double dc = x(6);
  const double cos_d = std::cos(dc);
  const double sin_d = std::sin(dc);

  const auto t = computeTireForces(x, params_);
  const double v_s = t.v_safe;
  const double m = params_.mass_kg;
  const double Iz = params_.inertia_kgm2;
  const double lf = params_.lf_m;
  const double lr = params_.lr_m;
  const double Ca = params_.Ca_N_rad;
  const double Cx = params_.Cx_N;
  const double rw = params_.r_w_m;
  const double Jw = params_.Jw_kgm2;

  const double daf_dvy = t.daf_dvy;
  const double daf_dom = t.daf_dom;

  const double dar_dvy = t.dar_dvy;
  const double dar_dom = t.dar_dom;

  const double dFyf_dvy = Ca * daf_dvy;
  const double dFyf_dom = Ca * daf_dom;
  const double dFyf_ddc = Ca;

  const double dFyr_dvy = Ca * dar_dvy;
  const double dFyr_dom = Ca * dar_dom;

  // v_safe는 선형화 시 상수 취급
  const double dFxf_dvx = Cx * (-1.0 / v_s);
  const double dFxf_dwf = Cx * (rw / v_s);
  const double dFxr_dvx = Cx * (-1.0 / v_s);
  const double dFxr_dwr = Cx * (rw / v_s);

  Ac = Eigen::Matrix<double,9,9>::Zero();

  Ac(0,2) = -vx * std::sin(psi) - vy * std::cos(psi);
  Ac(0,3) = std::cos(psi);
  Ac(0,4) = -std::sin(psi);

  Ac(1,2) = vx * std::cos(psi) - vy * std::sin(psi);
  Ac(1,3) = std::sin(psi);
  Ac(1,4) = std::cos(psi);

  Ac(2,5) = 1.0;

  Ac(3,3) = (dFxf_dvx * cos_d + dFxr_dvx) / m;
  Ac(3,4) = -dFyf_dvy * sin_d / m + omega;
  Ac(3,5) = -dFyf_dom * sin_d / m + vy;
  Ac(3,6) = (-t.Fx_f * sin_d - dFyf_ddc * sin_d - t.Fy_f * cos_d) / m;
  Ac(3,7) = dFxf_dwf * cos_d / m;
  Ac(3,8) = dFxr_dwr / m;

  Ac(4,3) = dFxf_dvx * sin_d / m - omega;
  Ac(4,4) = (dFyf_dvy * cos_d + dFyr_dvy) / m;
  Ac(4,5) = (dFyf_dom * cos_d + dFyr_dom) / m - vx;
  Ac(4,6) = (t.Fx_f * cos_d + dFyf_ddc * cos_d - t.Fy_f * sin_d) / m;
  Ac(4,7) = dFxf_dwf * sin_d / m;

  Ac(5,3) = lf * dFxf_dvx * sin_d / Iz;
  Ac(5,4) = (lf * dFyf_dvy * cos_d - lr * dFyr_dvy) / Iz;
  Ac(5,5) = (lf * dFyf_dom * cos_d - lr * dFyr_dom) / Iz;
  Ac(5,6) = lf * (t.Fx_f * cos_d + dFyf_ddc * cos_d - t.Fy_f * sin_d) / Iz;
  Ac(5,7) = lf * dFxf_dwf * sin_d / Iz;

  Ac(7,3) = -rw * dFxf_dvx / Jw;
  Ac(7,7) = -rw * dFxf_dwf / Jw;

  Ac(8,3) = -rw * dFxr_dvx / Jw;
  Ac(8,8) = -rw * dFxr_dwr / Jw;

  Bc = Eigen::Matrix<double,9,2>::Zero();
  Bc(6,0) = 1.0;
  Bc(7,1) = 0.0;
  Bc(8,1) = 1.0 / Jw;
}

// ============================================================
// 논문 식(2),(8): Implicit Euler (fixed-point 1회)
// ============================================================
Eigen::Matrix<double,9,1> MPCControllerCpp::dbmImplicitEuler(
  const Eigen::Matrix<double,9,1> & xk,
  const Eigen::Matrix<double,2,1> & uk) const
{
  const double dt = params_.dt;

  Eigen::Matrix<double,9,1> x0 = xk + dbmF(xk, uk) * dt;
  x0(2) = normalizeAngle(x0(2));

  Eigen::Matrix<double,9,1> x1 = xk + dbmF(x0, uk) * dt;
  x1(2) = normalizeAngle(x1(2));
  return x1;
}

// ============================================================
// DBM RTI-NMPC 메인 솔버 (논문 Algorithm 1)
// ============================================================
std::optional<MPCControlOutput> MPCControllerCpp::solveDBM(
  const VehicleState & state,
  const std::vector<RefPoint> & path,
  int nearest_idx,
  double speed_mps) const
{
  const int N = params_.prediction_horizon;
  const int nx = 9;
  const int nu = 2;
  const double rw = params_.r_w_m;

  if (N < 2 || static_cast<int>(path.size()) < 3) {
    return std::nullopt;
  }

  Eigen::Matrix<double,9,1> x0;
  x0(0) = state.x;
  x0(1) = state.y;
  x0(2) = state.yaw;
  x0(3) = state.vx;
  x0(4) = state.vy;
  x0(5) = state.omega;
  x0(6) = state.delta_c;
  x0(7) = state.omega_f;
  x0(8) = state.omega_r;

  const double dc_max = params_.max_steer_deg * kPi / 180.0;
  const double v_delta_max = std::max(params_.v_delta_max, 0.0);

  if (!g_dbm_warm_valid ||
    static_cast<int>(g_dbm_warm_u.size()) != N ||
    static_cast<int>(g_dbm_warm_x.size()) != N + 1)
  {
    g_dbm_warm_u.resize(N);
    for (int k = 0; k < N; ++k) {
      g_dbm_warm_u[k](0) = 0.0;
      g_dbm_warm_u[k](1) = 0.0;
    }

    g_dbm_warm_x.resize(N + 1);
    g_dbm_warm_x[0] = x0;
    for (int k = 0; k < N; ++k) {
      g_dbm_warm_x[k + 1] = dbmImplicitEuler(g_dbm_warm_x[k], g_dbm_warm_u[k]);
    }
    g_dbm_warm_valid = true;
  } else {
    g_dbm_warm_x[0] = x0;
  }

  std::vector<Eigen::Matrix<double,9,1>> x_ref(N);
  const double omega_ref = speed_mps / std::max(rw, 0.01);
  for (int k = 0; k < N; ++k) {
    const int idx = std::min(nearest_idx + k, static_cast<int>(path.size()) - 1);
    const double dc_ref = std::atan(params_.wheelbase_m * path[idx].kappa_r);
    x_ref[k](0) = path[idx].x;
    x_ref[k](1) = path[idx].y;
    x_ref[k](2) = path[idx].yaw;
    x_ref[k](3) = speed_mps;
    x_ref[k](4) = 0.0;
    x_ref[k](5) = 0.0;
    x_ref[k](6) = dc_ref;
    x_ref[k](7) = omega_ref;
    x_ref[k](8) = omega_ref;
  }

  using M99 = Eigen::Matrix<double,9,9>;
  using M92 = Eigen::Matrix<double,9,2>;
  using V9 = Eigen::Matrix<double,9,1>;

  std::vector<M99> Ad(N);
  std::vector<M92> Bd(N);
  std::vector<V9> fd(N);

  const M99 I9 = M99::Identity();

  for (int k = 0; k < N; ++k) {
    const V9 & xg = g_dbm_warm_x[k];
    const Eigen::Matrix<double,2,1> & ug = g_dbm_warm_u[k];

    M99 Ac;
    M92 Bc;
    dbmJacobian(xg, ug, Ac, Bc);

    M99 lhs = I9 - Ac * params_.dt;
    M99 lhs_inv = lhs.inverse();

    Ad[k] = lhs_inv;
    Bd[k] = lhs_inv * (Bc * params_.dt);

    V9 fd_raw = g_dbm_warm_x[k + 1] - Ad[k] * xg - Bd[k] * ug;
    fd_raw(2) = normalizeAngle(fd_raw(2));
    fd[k] = fd_raw;
  }

  const int Nx = nx * N;
  const int Nu = nu * N;

  Eigen::MatrixXd A_batch = Eigen::MatrixXd::Zero(Nx, nx);
  Eigen::MatrixXd B_batch = Eigen::MatrixXd::Zero(Nx, Nu);
  Eigen::VectorXd F_batch = Eigen::VectorXd::Zero(Nx);

  M99 Phi = I9;
  for (int k = 0; k < N; ++k) {
    A_batch.block(k * nx, 0, nx, nx) = Phi;

    if (k == 0) {
      F_batch.segment(0, nx) = fd[0];
    } else {
      F_batch.segment(k * nx, nx) = Ad[k] * F_batch.segment((k - 1) * nx, nx) + fd[k];
    }
    F_batch.segment(k * nx, nx)(2) = normalizeAngle(F_batch.segment(k * nx, nx)(2));

    B_batch.block(k * nx, k * nu, nx, nu) = Bd[k];
    for (int j = k + 1; j < N; ++j) {
      B_batch.block(j * nx, k * nu, nx, nu) =
        Ad[j - 1] * B_batch.block((j - 1) * nx, k * nu, nx, nu);
    }

    Phi = Ad[k] * Phi;
  }

  Eigen::VectorXd q_diag(nx);
  q_diag << params_.w_xy, params_.w_xy, params_.w_psi,
    params_.w_vx, params_.w_vy, params_.w_omega,
    params_.w_delta_c, params_.w_wheel, params_.w_wheel;

  Eigen::VectorXd r_diag(nu);
  r_diag << params_.w_u_delta, params_.w_u_torque;

  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(Nx, Nx);
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(Nu, Nu);
  for (int k = 0; k < N; ++k) {
    Q.block(k * nx, k * nx, nx, nx) = q_diag.asDiagonal();
    R.block(k * nu, k * nu, nu, nu) = r_diag.asDiagonal();
  }

  V9 delta_x0 = x0 - g_dbm_warm_x[0];
  delta_x0(2) = normalizeAngle(delta_x0(2));

  Eigen::VectorXd E(Nx);
  for (int k = 0; k < N; ++k) {
    E.segment(k * nx, nx) = g_dbm_warm_x[k] - x_ref[k];
    E.segment(k * nx, nx)(2) = normalizeAngle(E.segment(k * nx, nx)(2));
  }

  const Eigen::VectorXd eps = A_batch * delta_x0 + F_batch + E;

  Eigen::VectorXd Ug(Nu);
  for (int k = 0; k < N; ++k) {
    Ug(k * nu + 0) = g_dbm_warm_u[k](0);
    Ug(k * nu + 1) = g_dbm_warm_u[k](1);
  }

  Eigen::MatrixXd H = B_batch.transpose() * Q * B_batch + R;
  Eigen::VectorXd gv = B_batch.transpose() * Q * eps + R * Ug;
  H = 0.5 * (H + H.transpose());

  const int n_var = Nu;
  const int n_input_con = Nu;
  const int n_dc_con = N;
  const int n_fc_con = N;
  const int n_con = n_input_con + n_dc_con + n_fc_con;

  Eigen::MatrixXd A_dc = Eigen::MatrixXd::Zero(N, Nu);
  Eigen::VectorXd l_dc = Eigen::VectorXd::Zero(N);
  Eigen::VectorXd u_dc = Eigen::VectorXd::Zero(N);
  for (int k = 0; k < N; ++k) {
    A_dc.row(k) = B_batch.row(k * nx + 6);
    l_dc(k) = -dc_max - g_dbm_warm_x[k](6);
    u_dc(k) = dc_max - g_dbm_warm_x[k](6);
  }

  Eigen::MatrixXd A_fc = Eigen::MatrixXd::Zero(N, Nu);
  Eigen::VectorXd l_fc = Eigen::VectorXd::Zero(N);
  Eigen::VectorXd u_fc = Eigen::VectorXd::Zero(N);
  const double Fz = params_.mass_kg * 9.81;
  const double mu_Fz = std::max(params_.mu, 0.0) * Fz;

  for (int k = 0; k < N; ++k) {
    const auto t = computeTireForces(g_dbm_warm_x[k], params_);
    const double Fx_total = t.Fx_f + t.Fx_r;
    const double Fy_total = t.Fy_f + t.Fy_r;
    const double F_norm = std::hypot(Fx_total, Fy_total);

    const double nx_k = (F_norm > 1e-3) ? Fx_total / F_norm : 1.0;
    const double ny_k = (F_norm > 1e-3) ? Fy_total / F_norm : 0.0;

    const double dFx_dvx = params_.Cx_N * (-2.0 / t.v_safe);
    const double dFx_dwf = params_.Cx_N * (params_.r_w_m / t.v_safe);
    const double dFx_dwr = params_.Cx_N * (params_.r_w_m / t.v_safe);
    const double dFy_dvy = params_.Ca_N_rad * (t.daf_dvy + t.dar_dvy);
    const double dFy_dom = params_.Ca_N_rad * (t.daf_dom + t.dar_dom);
    const double dFy_ddc = params_.Ca_N_rad;

    Eigen::Matrix<double,1,9> dF_dx = Eigen::Matrix<double,1,9>::Zero();
    dF_dx(0,3) = nx_k * dFx_dvx;
    dF_dx(0,4) = ny_k * dFy_dvy;
    dF_dx(0,5) = ny_k * dFy_dom;
    dF_dx(0,6) = ny_k * dFy_ddc;
    dF_dx(0,7) = nx_k * dFx_dwf;
    dF_dx(0,8) = nx_k * dFx_dwr;

    A_fc.row(k) = dF_dx * B_batch.block(k * nx, 0, nx, Nu);

    const double F_proj_warm = nx_k * Fx_total + ny_k * Fy_total;
    l_fc(k) = -mu_Fz - F_proj_warm;
    u_fc(k) = mu_Fz - F_proj_warm;
  }

  Eigen::MatrixXd A_full = Eigen::MatrixXd::Zero(n_con, Nu);
  A_full.block(0, 0, Nu, Nu) = Eigen::MatrixXd::Identity(Nu, Nu);
  A_full.block(Nu, 0, N, Nu) = A_dc;
  A_full.block(Nu + N, 0, N, Nu) = A_fc;

  Eigen::SparseMatrix<double> P_sp = H.sparseView();
  Eigen::SparseMatrix<double> A_sp = A_full.sparseView();
  P_sp.makeCompressed();
  A_sp.makeCompressed();

  std::vector<c_float> Pd;
  std::vector<c_float> Av;
  std::vector<c_float> qv;
  std::vector<c_float> lv;
  std::vector<c_float> uv;
  std::vector<c_int> Pi;
  std::vector<c_int> Pp;
  std::vector<c_int> Ai;
  std::vector<c_int> Ap;

  for (int i = 0; i < P_sp.nonZeros(); ++i) {
    Pd.push_back(static_cast<c_float>(*(P_sp.valuePtr() + i)));
    Pi.push_back(static_cast<c_int>(*(P_sp.innerIndexPtr() + i)));
  }
  for (int i = 0; i <= n_var; ++i) {
    Pp.push_back(static_cast<c_int>(*(P_sp.outerIndexPtr() + i)));
  }
  for (int i = 0; i < A_sp.nonZeros(); ++i) {
    Av.push_back(static_cast<c_float>(*(A_sp.valuePtr() + i)));
    Ai.push_back(static_cast<c_int>(*(A_sp.innerIndexPtr() + i)));
  }
  for (int i = 0; i <= n_var; ++i) {
    Ap.push_back(static_cast<c_int>(*(A_sp.outerIndexPtr() + i)));
  }

  for (int k = 0; k < N; ++k) {
    lv.push_back(static_cast<c_float>(-v_delta_max - g_dbm_warm_u[k](0)));
    uv.push_back(static_cast<c_float>(v_delta_max - g_dbm_warm_u[k](0)));
    lv.push_back(static_cast<c_float>(-50.0 - g_dbm_warm_u[k](1)));
    uv.push_back(static_cast<c_float>(50.0 - g_dbm_warm_u[k](1)));
    qv.push_back(static_cast<c_float>(gv(k * nu + 0)));
    qv.push_back(static_cast<c_float>(gv(k * nu + 1)));
  }
  for (int k = 0; k < N; ++k) {
    lv.push_back(static_cast<c_float>(l_dc(k)));
    uv.push_back(static_cast<c_float>(u_dc(k)));
  }
  for (int k = 0; k < N; ++k) {
    lv.push_back(static_cast<c_float>(l_fc(k)));
    uv.push_back(static_cast<c_float>(u_fc(k)));
  }

  OSQPData data {};
  data.n = n_var;
  data.m = n_con;
  data.P = csc_matrix(n_var, n_var, P_sp.nonZeros(), Pd.data(), Pi.data(), Pp.data());
  data.q = qv.data();
  data.A = csc_matrix(n_con, n_var, A_sp.nonZeros(), Av.data(), Ai.data(), Ap.data());
  data.l = lv.data();
  data.u = uv.data();

  OSQPSettings settings {};
  osqp_set_default_settings(&settings);
  settings.verbose = 0;
  settings.polish = 1;
  settings.max_iter = 2000;
  settings.eps_abs = 1e-4;
  settings.eps_rel = 1e-4;

  OSQPWorkspace * work = nullptr;
  if (osqp_setup(&work, &data, &settings) != 0) {
    c_free(data.P);
    c_free(data.A);
    g_dbm_warm_valid = false;
    return std::nullopt;
  }
  osqp_solve(work);

  const bool ok = (work->info->status_val == OSQP_SOLVED ||
    work->info->status_val == OSQP_SOLVED_INACCURATE);

  MPCControlOutput out;
  if (ok) {
    const double du_v_delta = static_cast<double>(work->solution->x[0]);
    const double du_T = static_cast<double>(work->solution->x[1]);
    const double v_delta_opt = std::clamp(g_dbm_warm_u[0](0) + du_v_delta, -v_delta_max, v_delta_max);
    const double T_opt = std::clamp(g_dbm_warm_u[0](1) + du_T, -50.0, 50.0);
    const double dc_opt = std::clamp(x0(6) + v_delta_opt * params_.dt, -dc_max, dc_max);

    out.steer_deg = dc_opt * 180.0 / kPi;
    out.curvature = std::tan(dc_opt) / std::max(params_.wheelbase_m, 0.01);
    out.torque_cmd = T_opt;
    out.cost = work->info->obj_val;

    std::vector<Eigen::Matrix<double,2,1>> new_u(N);
    std::vector<V9> new_x(N + 1);
    new_x[0] = x0;

    for (int i = 0; i < N - 1; ++i) {
      new_u[i](0) = std::clamp(
        g_dbm_warm_u[i + 1](0) + static_cast<double>(work->solution->x[(i + 1) * nu + 0]),
        -v_delta_max, v_delta_max);
      new_u[i](1) = std::clamp(
        g_dbm_warm_u[i + 1](1) + static_cast<double>(work->solution->x[(i + 1) * nu + 1]),
        -50.0, 50.0);
    }
    new_u[N - 1] = new_u[N - 2];

    for (int k = 0; k < N; ++k) {
      new_x[k + 1] = dbmImplicitEuler(new_x[k], new_u[k]);
    }
    g_dbm_warm_u = new_u;
    g_dbm_warm_x = new_x;
  } else {
    g_dbm_warm_valid = false;
  }

  osqp_cleanup(work);
  c_free(data.P);
  c_free(data.A);

  if (!ok) {
    return std::nullopt;
  }
  return out;
}

// ============================================================
// Frenet LTV-MPC fallback
// ============================================================
FrenetState MPCControllerCpp::cartesianToFrenet(
  const VehicleState & state,
  const std::vector<RefPoint> & path,
  int nearest_idx) const
{
  const auto & ref = path[nearest_idx];
  const double dx = state.x - ref.x;
  const double dy = state.y - ref.y;

  FrenetState fs;
  fs.dr = -std::sin(ref.yaw) * dx + std::cos(ref.yaw) * dy;
  fs.dtheta = normalizeAngle(state.yaw - ref.yaw);
  fs.kappa = state.kappa;
  fs.theta_r = ref.yaw;
  fs.kappa_r = ref.kappa_r;
  return fs;
}

FrenetState MPCControllerCpp::propagateFrenetImplicit(
  const FrenetState & fs,
  double kappa_cmd,
  double v,
  double kappa_r_next) const
{
  const double dtheta_est = normalizeAngle(
    fs.dtheta + (v * kappa_cmd - v * fs.kappa_r) * params_.dt);

  FrenetState next;
  next.dr = fs.dr + v * std::sin(dtheta_est) * params_.dt;
  next.dtheta = dtheta_est;
  next.kappa = kappa_cmd;
  next.theta_r = fs.theta_r;
  next.kappa_r = kappa_r_next;
  return next;
}

double MPCControllerCpp::evaluateCandidate(
  const VehicleState & state,
  const std::vector<RefPoint> & path,
  int nearest_idx,
  double speed_mps,
  double curvature) const
{
  FrenetState fs = cartesianToFrenet(state, path, nearest_idx);
  double cost = 0.0;
  for (int k = 0; k < params_.prediction_horizon; ++k) {
    const int ref_idx = std::min(nearest_idx + k + 1, static_cast<int>(path.size()) - 1);
    fs = propagateFrenetImplicit(fs, curvature, speed_mps, path[ref_idx].kappa_r);
    cost += params_.w_d * fs.dr * fs.dr;
    cost += params_.w_theta * fs.dtheta * fs.dtheta;
    cost += params_.w_kappa * (fs.kappa - fs.kappa_r) * (fs.kappa - fs.kappa_r);
    cost += params_.w_u * curvature * curvature;
  }
  return cost;
}

std::optional<MPCControlOutput> MPCControllerCpp::solveFrenet(
  const VehicleState & state,
  const std::vector<RefPoint> & path,
  int nearest_idx,
  double speed_mps) const
{
  const int N = params_.prediction_horizon;
  const int nx = 3;
  const int nu = 1;
  const double v = speed_mps;

  if (N < 2 || static_cast<int>(path.size()) < 3) {
    return std::nullopt;
  }

  if (!g_fr_warm_valid || static_cast<int>(g_fr_warm_kappa.size()) != N) {
    g_fr_warm_kappa.assign(N, std::clamp(state.kappa, params_.kappa_min, params_.kappa_max));
    g_fr_warm_valid = true;
  }

  FrenetState fs0 = cartesianToFrenet(state, path, nearest_idx);
  std::vector<FrenetState> traj(N + 1);
  traj[0] = fs0;
  for (int k = 0; k < N; ++k) {
    const int ref_idx = std::min(nearest_idx + k, static_cast<int>(path.size()) - 1);
    traj[k + 1] = propagateFrenetImplicit(traj[k], g_fr_warm_kappa[k], v, path[ref_idx].kappa_r);
  }

  using M3 = Eigen::Matrix3d;
  using V3 = Eigen::Vector3d;
  using M31 = Eigen::Matrix<double,3,1>;

  std::vector<M3> Ad_fr(N);
  std::vector<M31> Bd_fr(N);
  std::vector<M31> fd_fr(N);

  for (int k = 0; k < N; ++k) {
    const FrenetState & xg_k1 = traj[k + 1];
    const FrenetState & xg_k = traj[k];
    const double ug = g_fr_warm_kappa[k];

    M3 A0 = M3::Identity();
    M3 A1 = M3::Zero();
    A1(0,1) = v * std::cos(xg_k1.dtheta) * params_.dt;

    M31 Bk = M31::Zero();
    Bk(2,0) = 1.0;
    Bk(1,0) = v * params_.dt;

    M3 IminusA1 = M3::Identity() - A1;
    M3 Iinv = IminusA1.inverse();
    Ad_fr[k] = Iinv * A0;
    Bd_fr[k] = Iinv * Bk;

    V3 fk;
    fk(0) = (xg_k.dr + v * std::sin(xg_k1.dtheta) * params_.dt) - xg_k1.dr;
    fk(1) = normalizeAngle(
      (xg_k.dtheta + (v * ug - v * xg_k.kappa_r) * params_.dt) - xg_k1.dtheta);
    fk(2) = ug - xg_k1.kappa;
    fd_fr[k] = Iinv * fk;
  }

  Eigen::MatrixXd A_batch = Eigen::MatrixXd::Zero(nx * N, nx);
  Eigen::MatrixXd B_batch = Eigen::MatrixXd::Zero(nx * N, nu * N);
  Eigen::VectorXd F_batch = Eigen::VectorXd::Zero(nx * N);
  M3 Phi = M3::Identity();

  for (int k = 0; k < N; ++k) {
    A_batch.block(k * nx, 0, nx, nx) = Phi;
    if (k == 0) {
      F_batch.segment(0, nx) = fd_fr[0];
    } else {
      F_batch.segment(k * nx, nx) = Ad_fr[k] * F_batch.segment((k - 1) * nx, nx) + fd_fr[k];
    }
    B_batch.block(k * nx, k * nu, nx, nu) = Bd_fr[k];
    for (int j = k + 1; j < N; ++j) {
      B_batch.block(j * nx, k * nu, nx, nu) =
        Ad_fr[j - 1] * B_batch.block((j - 1) * nx, k * nu, nx, nu);
    }
    Phi = Ad_fr[k] * Phi;
  }

  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(nx * N, nx * N);
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(nu * N, nu * N);
  for (int k = 0; k < N; ++k) {
    Q(k * nx + 0, k * nx + 0) = params_.w_d;
    Q(k * nx + 1, k * nx + 1) = params_.w_theta;
    Q(k * nx + 2, k * nx + 2) = params_.w_kappa;
    R(k * nu, k * nu) = params_.w_u;
  }

  V3 delta_x0;
  delta_x0 << fs0.dr - traj[0].dr,
    normalizeAngle(fs0.dtheta - traj[0].dtheta),
    fs0.kappa - traj[0].kappa;

  Eigen::VectorXd Ug(nu * N);
  for (int k = 0; k < N; ++k) {
    Ug(k) = g_fr_warm_kappa[k];
  }

  const Eigen::VectorXd eps = A_batch * delta_x0 + F_batch;
  Eigen::MatrixXd H = B_batch.transpose() * Q * B_batch + R;
  Eigen::VectorXd gv = B_batch.transpose() * Q * eps + R * Ug;
  H = 0.5 * (H + H.transpose());

  const int n_var = nu * N;
  Eigen::SparseMatrix<double> P_sp = H.sparseView();
  Eigen::SparseMatrix<double> A_sp(n_var, n_var);
  A_sp.setIdentity();
  P_sp.makeCompressed();
  A_sp.makeCompressed();

  std::vector<c_float> Pd;
  std::vector<c_float> Av;
  std::vector<c_float> qv2;
  std::vector<c_float> lv;
  std::vector<c_float> uv;
  std::vector<c_int> Pi;
  std::vector<c_int> Pp;
  std::vector<c_int> Ai;
  std::vector<c_int> Ap;

  for (int i = 0; i < P_sp.nonZeros(); ++i) {
    Pd.push_back(static_cast<c_float>(*(P_sp.valuePtr() + i)));
    Pi.push_back(static_cast<c_int>(*(P_sp.innerIndexPtr() + i)));
  }
  for (int i = 0; i <= n_var; ++i) {
    Pp.push_back(static_cast<c_int>(*(P_sp.outerIndexPtr() + i)));
  }
  for (int i = 0; i < A_sp.nonZeros(); ++i) {
    Av.push_back(static_cast<c_float>(*(A_sp.valuePtr() + i)));
    Ai.push_back(static_cast<c_int>(*(A_sp.innerIndexPtr() + i)));
  }
  for (int i = 0; i <= n_var; ++i) {
    Ap.push_back(static_cast<c_int>(*(A_sp.outerIndexPtr() + i)));
  }
  for (int i = 0; i < n_var; ++i) {
    qv2.push_back(static_cast<c_float>(gv(i)));
    lv.push_back(static_cast<c_float>(params_.kappa_min - g_fr_warm_kappa[i]));
    uv.push_back(static_cast<c_float>(params_.kappa_max - g_fr_warm_kappa[i]));
  }

  OSQPData data {};
  data.n = n_var;
  data.m = n_var;
  data.P = csc_matrix(n_var, n_var, P_sp.nonZeros(), Pd.data(), Pi.data(), Pp.data());
  data.q = qv2.data();
  data.A = csc_matrix(n_var, n_var, A_sp.nonZeros(), Av.data(), Ai.data(), Ap.data());
  data.l = lv.data();
  data.u = uv.data();

  OSQPSettings settings {};
  osqp_set_default_settings(&settings);
  settings.verbose = 0;
  settings.polish = 1;
  settings.max_iter = 1000;
  settings.eps_abs = 1e-4;
  settings.eps_rel = 1e-4;

  OSQPWorkspace * work = nullptr;
  if (osqp_setup(&work, &data, &settings) != 0) {
    c_free(data.P);
    c_free(data.A);
    return std::nullopt;
  }
  osqp_solve(work);

  const bool ok = (work->info->status_val == OSQP_SOLVED ||
    work->info->status_val == OSQP_SOLVED_INACCURATE);

  MPCControlOutput out;
  if (ok) {
    const double du0 = static_cast<double>(work->solution->x[0]);
    out.curvature = std::clamp(g_fr_warm_kappa[0] + du0, params_.kappa_min, params_.kappa_max);
    const double sr = std::atan(params_.wheelbase_m * out.curvature);
    out.steer_deg = std::clamp(sr * 180.0 / kPi, -params_.max_steer_deg, params_.max_steer_deg);
    out.cost = work->info->obj_val;

    std::vector<double> new_warm(N);
    for (int i = 0; i < N - 1; ++i) {
      new_warm[i] = std::clamp(
        g_fr_warm_kappa[i + 1] + static_cast<double>(work->solution->x[i + 1]),
        params_.kappa_min, params_.kappa_max);
    }
    new_warm[N - 1] = new_warm[N - 2];
    g_fr_warm_kappa = new_warm;
  } else {
    g_fr_warm_valid = false;
  }

  osqp_cleanup(work);
  c_free(data.P);
  c_free(data.A);

  if (!ok) {
    return std::nullopt;
  }
  return out;
}

// ============================================================
// 진입점
// ============================================================
MPCControlOutput MPCControllerCpp::computeControl(
  const VehicleState & state,
  const std::vector<RefPoint> & path,
  int nearest_idx,
  double speed_mps) const
{
  if (params_.use_dbm) {
    if (auto r = solveDBM(state, path, nearest_idx, speed_mps)) {
      return *r;
    }
  }
  if (auto r = solveFrenet(state, path, nearest_idx, speed_mps)) {
    return *r;
  }

  MPCControlOutput out;
  out.cost = std::numeric_limits<double>::max();
  for (int i = 0; i < params_.kappa_samples; ++i) {
    const double r = static_cast<double>(i) / static_cast<double>(params_.kappa_samples - 1);
    const double kp = params_.kappa_min + r * (params_.kappa_max - params_.kappa_min);
    const double c = evaluateCandidate(state, path, nearest_idx, speed_mps, kp);
    if (c < out.cost) {
      out.cost = c;
      out.curvature = kp;
    }
  }
  const double sr = std::atan(params_.wheelbase_m * out.curvature);
  out.steer_deg = std::clamp(sr * 180.0 / kPi, -params_.max_steer_deg, params_.max_steer_deg);
  return out;
}

}  // namespace jeju_mpc
