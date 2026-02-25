#include "jeju_mpc/mpc_controller_cpp.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

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
  VehicleState pred = state;
  double cost = 0.0;

  for (int k = 0; k < params_.prediction_horizon; ++k) {
    pred.x += speed_mps * std::cos(pred.yaw) * params_.dt;
    pred.y += speed_mps * std::sin(pred.yaw) * params_.dt;
    pred.yaw = normalizeAngle(pred.yaw + speed_mps * curvature * params_.dt);

    const int ref_idx = std::min(nearest_idx + k, static_cast<int>(path.size()) - 1);
    const auto & ref = path[ref_idx];

    const double ex = pred.x - ref.x;
    const double ey = pred.y - ref.y;
    const double cte = std::sqrt(ex * ex + ey * ey);
    const double heading_err = normalizeAngle(pred.yaw - ref.yaw);

    cost += params_.w_d * cte * cte;
    cost += params_.w_theta * heading_err * heading_err;
    cost += params_.w_kappa * curvature * curvature;
  }

  return cost;
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
  (void)speed_mps;
  const int N = params_.prediction_horizon;
  if (N < 2 || path.size() < 3) {
    return std::nullopt;
  }

  const double heading_err = normalizeAngle(state.yaw - path[nearest_idx].yaw);
  std::vector<double> kappa_ref(N, 0.0);

  for (int k = 0; k < N; ++k) {
    const int idx = std::min(nearest_idx + k, static_cast<int>(path.size()) - 2);
    const int next = std::min(idx + 1, static_cast<int>(path.size()) - 1);
    const double dyaw = normalizeAngle(path[next].yaw - path[idx].yaw);
    const double ds = std::max(0.05, std::hypot(path[next].x - path[idx].x, path[next].y - path[idx].y));
    double kref = dyaw / ds;
    if (k == 0) {
      // Initial heading error correction for single-vehicle tracking.
      kref -= params_.w_d * heading_err / std::max(params_.wheelbase_m, 0.2);
    }
    kappa_ref[k] = std::clamp(kref, params_.kappa_min, params_.kappa_max);
  }

  const c_int n = static_cast<c_int>(N);
  const c_int m = static_cast<c_int>(N);

  auto * P = csc_spalloc(n, n, 3 * n - 2, 1, 0);
  auto * A = csc_spalloc(m, n, n, 1, 0);
  if (P == nullptr || A == nullptr) {
    if (P != nullptr) {
      csc_spfree(P);
    }
    if (A != nullptr) {
      csc_spfree(A);
    }
    return std::nullopt;
  }

  c_int nnzP = 0;
  c_int nnzA = 0;
  P->p[0] = 0;
  A->p[0] = 0;

  const double w_track = std::max(1e-6, params_.w_theta + params_.w_kappa);
  const double w_smooth = std::max(1e-6, params_.w_u);

  for (c_int j = 0; j < n; ++j) {
    double diag = 2.0 * w_track;
    if (j > 0) {
      diag += 2.0 * w_smooth;
    }
    if (j < n - 1) {
      diag += 2.0 * w_smooth;
    }

    P->i[nnzP] = j;
    P->x[nnzP] = static_cast<c_float>(diag);
    ++nnzP;

    if (j < n - 1) {
      P->i[nnzP] = j + 1;
      P->x[nnzP] = static_cast<c_float>(-2.0 * w_smooth);
      ++nnzP;
    }

    P->p[j + 1] = nnzP;

    A->i[nnzA] = j;
    A->x[nnzA] = 1.0;
    ++nnzA;
    A->p[j + 1] = nnzA;
  }

  P->nzmax = nnzP;
  A->nzmax = nnzA;

  std::vector<c_float> q(n, 0.0);
  std::vector<c_float> l(m, static_cast<c_float>(params_.kappa_min));
  std::vector<c_float> u(m, static_cast<c_float>(params_.kappa_max));
  for (int i = 0; i < N; ++i) {
    q[i] = static_cast<c_float>(-2.0 * w_track * kappa_ref[i]);
  }

  OSQPData data;
  data.n = n;
  data.m = m;
  data.P = P;
  data.A = A;
  data.q = q.data();
  data.l = l.data();
  data.u = u.data();

  OSQPSettings settings;
  osqp_set_default_settings(&settings);
  settings.verbose = false;
  settings.max_iter = 2000;
  settings.eps_abs = 1e-4;
  settings.eps_rel = 1e-4;
  settings.polish = false;

  OSQPWorkspace * work = nullptr;
  if (osqp_setup(&work, &data, &settings) != 0 || work == nullptr) {
    csc_spfree(P);
    csc_spfree(A);
    return std::nullopt;
  }

  if (osqp_solve(work) != 0 || work->info == nullptr || work->solution == nullptr) {
    osqp_cleanup(work);
    csc_spfree(P);
    csc_spfree(A);
    return std::nullopt;
  }

  if (!(work->info->status_val == OSQP_SOLVED || work->info->status_val == OSQP_SOLVED_INACCURATE)) {
    osqp_cleanup(work);
    csc_spfree(P);
    csc_spfree(A);
    return std::nullopt;
  }

  MPCControlOutput out;
  out.curvature = static_cast<double>(work->solution->x[0]);
  const double steer_rad = std::atan(params_.wheelbase_m * out.curvature);
  out.steer_deg = std::clamp(steer_rad * 180.0 / kPi, -params_.max_steer_deg, params_.max_steer_deg);
  out.cost = static_cast<double>(work->info->obj_val);

  osqp_cleanup(work);
  csc_spfree(P);
  csc_spfree(A);
  return out;
}

}  // namespace jeju_mpc
