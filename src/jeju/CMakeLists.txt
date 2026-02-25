cmake_minimum_required(VERSION 3.8)
project(jeju)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(osqp_vendor REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)

find_path(OSQP_INCLUDE_DIR
  NAMES osqp.h
  PATHS /opt/ros/humble/include
  PATH_SUFFIXES osqp
)
find_library(OSQP_LIBRARY
  NAMES osqp
  PATHS /opt/ros/humble/lib
)
if(NOT OSQP_INCLUDE_DIR OR NOT OSQP_LIBRARY)
  message(FATAL_ERROR "OSQP headers or library not found. Please install ros-humble-osqp-vendor.")
endif()

# C++ MPC nodes
add_executable(mpc_path_follower_node
  mpc/src/mpc_controller_cpp.cpp
  mpc/src/mpc_path_tracker_cpp.cpp
  mpc/src/mpc_path_follower_node.cpp
)
target_include_directories(mpc_path_follower_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/mpc/include
  ${OSQP_INCLUDE_DIR}
)
ament_target_dependencies(mpc_path_follower_node
  osqp_vendor
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
)
target_link_libraries(mpc_path_follower_node
  ${OSQP_LIBRARY}
)

add_executable(mpc_path_maker_node
  mpc/src/mpc_path_maker_node.cpp
)
target_include_directories(mpc_path_maker_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/mpc/include
)
ament_target_dependencies(mpc_path_maker_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
)

add_executable(gps_rtk_gui_node
  mpc/src/gps_rtk_gui_node.cpp
)
target_include_directories(gps_rtk_gui_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(gps_rtk_gui_node
  rclcpp
  std_msgs
  sensor_msgs
)
target_link_libraries(gps_rtk_gui_node
  ${OpenCV_LIBRARIES}
)

# Install Python scripts
install(PROGRAMS
  control/teleop_node.py
  control/serial_bridge_node.py
  sensing/status_info_node.py
  sensing/wheel_odom_node.py
  navigation/path_follower_node.py
  navigation/path_maker_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  mpc_path_follower_node
  mpc_path_maker_node
  gps_rtk_gui_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY mpc/include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install arduino files
install(DIRECTORY arduino
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY mpc
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
