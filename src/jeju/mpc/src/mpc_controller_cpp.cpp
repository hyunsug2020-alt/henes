#include "jeju_mpc/mpc_controller_cpp.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <Eigen/Sparse>
#include <osqp.h>

namespace jeju_mpc
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
}

MPCControllerCpp::MPCControllerCpp() = default;

void MPCControllerCpp::updateParameters(const MPCControllerParams & params)
{
  params_ = params;
  params_.prediction_horizon = std::max(1, params_.prediction_horizon);
  params_.kappa_samples = std::max(3, params_.kappa_samples);
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
    const double kappa_r_next = path[ref_idx].kappa_r;

    fs = propagateFrenet(fs, curvature, speed_mps, kappa_r_next);

    cost += params_.w_d * fs.dr * fs.dr;
    cost += params_.w_theta * fs.dtheta * fs.dtheta;
    cost += params_.w_kappa * (fs.kappa - fs.kappa_r) * (fs.kappa - fs.kappa_r);
    cost += params_.w_u * curvature * curvature;
  }

  return cost;
}

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

FrenetState MPCControllerCpp::propagateFrenet(
  const FrenetState & fs,
  double kappa_cmd,
  double v,
  double kappa_r_next) const
{
  FrenetState next;
  next.dr = fs.dr + v * std::sin(fs.dtheta) * params_.dt;
  next.dtheta = normalizeAngle(fs.dtheta + (v * fs.kappa - v * fs.kappa_r) * params_.dt);
  next.kappa = kappa_cmd;
  next.theta_r = fs.theta_r;
  next.kappa_r = kappa_r_next;
  return next;
}

MPCControlOutput MPCControllerCpp::computeControl(
  const VehicleState & state,
  const std::vector<RefPoint> & path,
  int nearest_idx,
  double speed_mps) const
{
  if (auto qp_out = solveWithOsqp(state, path, nearest_idx, speed_mps)) {
    return *qp_out;
  }

  MPCControlOutput out;
  out.cost = std::numeric_limits<double>::max();

  for (int i = 0; i < params_.kappa_samples; ++i) {
    const double r = static_cast<double>(i) / static_cast<double>(params_.kappa_samples - 1);
    const double kappa = params_.kappa_min + r * (params_.kappa_max - params_.kappa_min);
    const double cost = evaluateCandidate(state, path, nearest_idx, speed_mps, kappa);
    if (cost < out.cost) {
      out.cost = cost;
      out.curvature = kappa;
    }
  }

  const double steer_rad = std::atan(params_.wheelbase_m * out.curvature);
  const double steer_deg = steer_rad * 180.0 / kPi;
  out.steer_deg = std::clamp(steer_deg, -params_.max_steer_deg, params_.max_steer_deg);

  return out;
}

std::optional<MPCControlOutput> MPCControllerCpp::solveWithOsqp(
  const VehicleState & state,
  const std::vector<RefPoint> & path,
  int nearest_idx,
  double speed_mps) const
{
  const int N = params_.prediction_horizon;
  if (N < 2 || static_cast<int>(path.size()) < 3) {
    return std::nullopt;
  }

  // 초기 Frenet 상태
  FrenetState fs0 = cartesianToFrenet(state, path, nearest_idx);

  // 상태 차원: [dr, dtheta, kappa] = 3
  // 입력 차원: [kappa_cmd] = 1
  const int nx = 3;
  const int nu = 1;

  // 예측 행렬 구성 (LTV: 매 스텝 v, kappa_r이 달라질 수 있음)
  // X = [x0; x1; ... xN], U = [u0; u1; ... u_{N-1}]
  // xi+1 = Ai*xi + Bi*ui + fi
  Eigen::MatrixXd A_batch = Eigen::MatrixXd::Zero(nx * N, nx);
  Eigen::MatrixXd B_batch = Eigen::MatrixXd::Zero(nx * N, nu * N);
  Eigen::VectorXd f_batch = Eigen::VectorXd::Zero(nx * N);

  const double dt = params_.dt;
  FrenetState fs = fs0;

  for (int k = 0; k < N; ++k) {
    const int ref_idx = std::min(nearest_idx + k, static_cast<int>(path.size()) - 1);
    const double v = speed_mps;
    const double kappa_r = path[ref_idx].kappa_r;

    // Ak = d(f)/d(x)
    Eigen::MatrixXd Ak = Eigen::MatrixXd::Identity(nx, nx);
    Ak(0, 1) = v * std::cos(fs.dtheta) * dt;  // d(dr)/d(dtheta)
    Ak(1, 2) = v * dt;                        // d(dtheta)/d(kappa)

    // Bk = d(f)/d(u)
    Eigen::MatrixXd Bk = Eigen::MatrixXd::Zero(nx, nu);
    Bk(2, 0) = 1.0;  // kappa = kappa_cmd 직접 대입

    // fk = 비선형 나머지 (affine term)
    Eigen::VectorXd fk = Eigen::VectorXd::Zero(nx);
    fk(0) = v * std::sin(fs.dtheta) * dt - Ak(0, 1) * fs.dtheta;
    fk(1) = -v * kappa_r * dt;

    // A_batch 누적
    Eigen::MatrixXd Ak_power = Eigen::MatrixXd::Identity(nx, nx);
    for (int j = 0; j <= k; ++j) {
      if (j == k) {
        A_batch.block(k * nx, 0, nx, nx) = Ak_power;
      }
      Ak_power = Ak * Ak_power;
    }

    // B_batch
    B_batch.block(k * nx, k * nu, nx, nu) = Bk;
    for (int j = k + 1; j < N; ++j) {
      B_batch.block(j * nx, k * nu, nx, nu) = Ak * B_batch.block((j - 1) * nx, k * nu, nx, nu);
    }

    f_batch.segment(k * nx, nx) = fk;

    // 다음 스텝 예측 (중앙값 kappa 사용)
    fs = propagateFrenet(fs, 0.0, v, kappa_r);
  }

  // 비용함수 가중치 행렬
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(nx * N, nx * N);
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(nu * N, nu * N);
  for (int k = 0; k < N; ++k) {
    Q(k * nx + 0, k * nx + 0) = params_.w_d;
    Q(k * nx + 1, k * nx + 1) = params_.w_theta;
    Q(k * nx + 2, k * nx + 2) = params_.w_kappa;
    R(k * nu, k * nu) = params_.w_u;
  }

  // x0 벡터
  Eigen::VectorXd x0(nx);
  x0 << fs0.dr, fs0.dtheta, fs0.kappa;

  // 예측: X = A_batch*x0 + B_batch*U + F
  // 비용: min 0.5*U^T*H*U + g^T*U
  Eigen::MatrixXd H = B_batch.transpose() * Q * B_batch + R;
  Eigen::VectorXd Ax0f = A_batch * x0 + f_batch;
  Eigen::VectorXd g = B_batch.transpose() * Q * Ax0f;

  // H 대칭화
  H = 0.5 * (H + H.transpose());

  // OSQP 입력 변환
  const int n_var = nu * N;
  const int n_con = nu * N;

  // 입력 제약: kappa_min <= u <= kappa_max
  Eigen::SparseMatrix<double> P_sp = H.sparseView();
  Eigen::SparseMatrix<double> A_sp(n_con, n_var);
  A_sp.setIdentity();

  std::vector<c_float> P_data_v;
  std::vector<c_float> A_data_v;
  std::vector<c_float> q_data_v;
  std::vector<c_float> l_data_v;
  std::vector<c_float> u_data_v;
  std::vector<c_int> P_idx_v;
  std::vector<c_int> P_ptr_v;
  std::vector<c_int> A_idx_v;
  std::vector<c_int> A_ptr_v;

  P_sp.makeCompressed();
  for (int i = 0; i < P_sp.nonZeros(); ++i) {
    P_data_v.push_back(static_cast<c_float>(*(P_sp.valuePtr() + i)));
    P_idx_v.push_back(static_cast<c_int>(*(P_sp.innerIndexPtr() + i)));
  }
  for (int i = 0; i <= n_var; ++i) {
    P_ptr_v.push_back(static_cast<c_int>(*(P_sp.outerIndexPtr() + i)));
  }

  A_sp.makeCompressed();
  for (int i = 0; i < A_sp.nonZeros(); ++i) {
    A_data_v.push_back(static_cast<c_float>(*(A_sp.valuePtr() + i)));
    A_idx_v.push_back(static_cast<c_int>(*(A_sp.innerIndexPtr() + i)));
  }
  for (int i = 0; i <= n_var; ++i) {
    A_ptr_v.push_back(static_cast<c_int>(*(A_sp.outerIndexPtr() + i)));
  }

  for (int i = 0; i < n_var; ++i) {
    q_data_v.push_back(static_cast<c_float>(g(i)));
    l_data_v.push_back(static_cast<c_float>(params_.kappa_min));
    u_data_v.push_back(static_cast<c_float>(params_.kappa_max));
  }

  OSQPData data {};
  data.n = n_var;
  data.m = n_con;
  data.P = csc_matrix(
    n_var, n_var, P_sp.nonZeros(),
    P_data_v.data(), P_idx_v.data(), P_ptr_v.data());
  data.q = q_data_v.data();
  data.A = csc_matrix(
    n_con, n_var, A_sp.nonZeros(),
    A_data_v.data(), A_idx_v.data(), A_ptr_v.data());
  data.l = l_data_v.data();
  data.u = u_data_v.data();

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
    out.curvature = std::clamp(
      static_cast<double>(work->solution->x[0]),
      params_.kappa_min, params_.kappa_max);
    const double steer_rad = std::atan(params_.wheelbase_m * out.curvature);
    out.steer_deg = std::clamp(
      steer_rad * 180.0 / kPi,
      -params_.max_steer_deg, params_.max_steer_deg);
    out.cost = work->info->obj_val;
  }

  osqp_cleanup(work);
  c_free(data.P);
  c_free(data.A);

  if (!ok) {
    return std::nullopt;
  }
  return out;
}

}  // namespace jeju_mpc
