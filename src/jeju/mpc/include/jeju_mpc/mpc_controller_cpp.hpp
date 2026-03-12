#pragma once

#include <Eigen/Dense>
#include <optional>
#include <vector>

namespace jeju_mpc
{

struct MPCControllerParams
{
  // ── 공통 ─────────────────────────────────────────────────
  double wheelbase_m        {1.04};
  double dt                 {0.1};
  int    prediction_horizon {15};
  double max_steer_deg      {55.0};
  double kappa_min          {-0.9};
  double kappa_max          {0.9};
  int    kappa_samples      {31};

  // ── Frenet LTV-MPC fallback 가중치 ───────────────────────
  double w_d                {1.0};
  double w_theta            {0.5};
  double w_kappa            {0.05};
  double w_u                {0.2};

  // ── 9-state DBM 물리 파라미터 ────────────────────────────
  double mass_kg            {150.0};
  double inertia_kgm2       {120.0};
  double lf_m               {0.52};
  double lr_m               {0.52};
  double Ca_N_rad           {25000.0};
  double Cx_N               {30000.0};
  double Jw_kgm2            {0.05};
  double r_w_m              {0.08};
  double v_delta_max        {0.5};
  double v_min_mps          {0.3};
  double mu                 {0.8};

  // ── 9-state DBM 비용 가중치 ──────────────────────────────
  double w_xy               {2.0};
  double w_psi              {1.0};
  double w_vx               {0.5};
  double w_vy               {0.3};
  double w_omega            {0.3};
  double w_delta_c          {0.1};
  double w_wheel            {0.01};
  double w_u_delta          {0.2};
  double w_u_torque         {1e-5};

  bool   use_dbm            {true};
};

struct RefPoint
{
  double x       {0.0};
  double y       {0.0};
  double yaw     {0.0};
  double kappa_r {0.0};
};

struct VehicleState
{
  double x       {0.0};
  double y       {0.0};
  double yaw     {0.0};
  double kappa   {0.0};
  double vx      {0.0};
  double vy      {0.0};
  double omega   {0.0};
  double delta_c {0.0};
  double omega_f {0.0};
  double omega_r {0.0};
};

struct FrenetState
{
  double dr      {0.0};
  double dtheta  {0.0};
  double kappa   {0.0};
  double theta_r {0.0};
  double kappa_r {0.0};
};

struct MPCControlOutput
{
  double curvature {0.0};
  double steer_deg {0.0};
  double torque_cmd {0.0};
  double cost      {0.0};
};

class MPCControllerCpp
{
public:
  MPCControllerCpp();
  void updateParameters(const MPCControllerParams & params);

  // speed_mps 부호: 양수=전진, 음수=후진
  MPCControlOutput computeControl(
    const VehicleState & state,
    const std::vector<RefPoint> & path,
    int nearest_idx,
    double speed_mps) const;

  static double normalizeAngle(double angle);

private:
  // ── 9-state DBM RTI-NMPC ─────────────────────────────────
  std::optional<MPCControlOutput> solveDBM(
    const VehicleState & state,
    const std::vector<RefPoint> & path,
    int nearest_idx,
    double speed_mps) const;

  // 논문 식(26-29) 확장: ẋ = f(x, u)
  // x(9) = [px, py, ψ, vx, vy, ω, δc, ωf, ωr]
  // u(2) = [v_delta, T_drive]
  Eigen::Matrix<double,9,1> dbmF(
    const Eigen::Matrix<double,9,1> & x,
    const Eigen::Matrix<double,2,1> & u) const;

  // 논문 식(12-14): Ac=∂f/∂x (9×9), Bc=∂f/∂u (9×2)
  void dbmJacobian(
    const Eigen::Matrix<double,9,1> & x,
    const Eigen::Matrix<double,2,1> & u,
    Eigen::Matrix<double,9,9>       & Ac,
    Eigen::Matrix<double,9,2>       & Bc) const;

  // 논문 식(2),(8): Implicit Euler 1-step (fixed-point 1회)
  Eigen::Matrix<double,9,1> dbmImplicitEuler(
    const Eigen::Matrix<double,9,1> & xk,
    const Eigen::Matrix<double,2,1> & uk) const;

  // ── Frenet LTV-MPC fallback ──────────────────────────────
  std::optional<MPCControlOutput> solveFrenet(
    const VehicleState & state,
    const std::vector<RefPoint> & path,
    int nearest_idx,
    double speed_mps) const;

  double evaluateCandidate(
    const VehicleState & state,
    const std::vector<RefPoint> & path,
    int nearest_idx,
    double speed_mps,
    double curvature) const;

  FrenetState cartesianToFrenet(
    const VehicleState & state,
    const std::vector<RefPoint> & path,
    int nearest_idx) const;

  FrenetState propagateFrenetImplicit(
    const FrenetState & fs,
    double kappa_cmd,
    double v,
    double kappa_r_next) const;

  MPCControllerParams params_;
};

}  // namespace jeju_mpc
