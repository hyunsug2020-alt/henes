#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "jeju_mpc/mpc_path_tracker_cpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<jeju_mpc::MPCPathTrackerCpp>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

