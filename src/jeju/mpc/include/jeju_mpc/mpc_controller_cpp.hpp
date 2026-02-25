#pragma once

#include <optional>
#include <vector>

namespace jeju_mpc
{

struct MPCControllerParams
{
  double wheelbase_m {1.04};
  double dt {0.1};
  int prediction_horizon {15};
  double w_d {1.0};
  double w_theta {0.5};
  double w_kappa {0.05};
  double w_u {0.2};
  double kappa_min {-0.9};
  double kappa_max {0.9};
  int kappa_samples {31};
  double max_steer_deg {55.0};
};

struct RefPoint
{
  double x {0.0};
  double y {0.0};
  double yaw {0.0};
};

struct VehicleState
{
  double x {0.0};
  double y {0.0};
  double yaw {0.0};
};

struct MPCControlOutput
{
  double curvature {0.0};
  double steer_deg {0.0};
  double cost {0.0};
};

class MPCControllerCpp
{
public:
  MPCControllerCpp();
  void updateParameters(const MPCControllerParams & params);

  MPCControlOutput computeControl(
    const VehicleState & state,
    const std::vector<RefPoint> & path,
    int nearest_idx,
    double speed_mps) const;

  static double normalizeAngle(double angle);

private:
  std::optional<MPCControlOutput> solveWithOsqp(
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

  MPCControllerParams params_;
};

}  // namespace jeju_mpc
