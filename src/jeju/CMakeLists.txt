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
find_package(ublox_ubx_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

find_path(OSQP_INCLUDE_DIR NAMES osqp.h PATHS /opt/ros/humble/include PATH_SUFFIXES osqp)
find_library(OSQP_LIBRARY NAMES osqp PATHS /opt/ros/humble/lib)
if(NOT OSQP_INCLUDE_DIR OR NOT OSQP_LIBRARY)
  message(FATAL_ERROR "OSQP not found.")
endif()

set(INC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# ── MPC 노드 ──────────────────────────────────────────────────
add_executable(mpc_path_follower_node
  src/ltv_mpc.cpp
  src/mpc_controller_cpp.cpp
  src/mpc_path_tracker_cpp.cpp
  src/mpc_path_follower_node.cpp)
target_include_directories(mpc_path_follower_node PRIVATE
  ${INC} ${OSQP_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(mpc_path_follower_node
  osqp_vendor rclcpp std_msgs geometry_msgs sensor_msgs nav_msgs)
target_link_libraries(mpc_path_follower_node ${OSQP_LIBRARY} Eigen3::Eigen)

add_executable(mpc_logger_node src/mpc_logger_node.cpp)
target_include_directories(mpc_logger_node PRIVATE ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(mpc_logger_node rclcpp std_msgs geometry_msgs nav_msgs)
target_link_libraries(mpc_logger_node Eigen3::Eigen)

add_executable(mpc_path_maker_node src/mpc_path_maker_node.cpp)
target_include_directories(mpc_path_maker_node PRIVATE ${INC})
ament_target_dependencies(mpc_path_maker_node rclcpp std_msgs geometry_msgs nav_msgs)

add_executable(mpc_monitor_node src/mpc_monitor_node.cpp)
target_include_directories(mpc_monitor_node PRIVATE ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(mpc_monitor_node rclcpp rcl_interfaces std_msgs geometry_msgs nav_msgs)
target_link_libraries(mpc_monitor_node ${OpenCV_LIBRARIES})

add_executable(gps_rtk_gui_node src/gps_rtk_gui_node.cpp)
target_include_directories(gps_rtk_gui_node PRIVATE ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(gps_rtk_gui_node rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs ublox_ubx_msgs)
target_link_libraries(gps_rtk_gui_node ${OpenCV_LIBRARIES})

# ── GPS/IMU 노드 ───────────────────────────────────────────────
add_executable(gnss_velocity_bridge_node src/gnss_velocity_bridge_node.cpp)
ament_target_dependencies(gnss_velocity_bridge_node rclcpp geometry_msgs ublox_ubx_msgs)

add_executable(gnss_navsat_bridge_node src/gnss_navsat_bridge_node.cpp)
ament_target_dependencies(gnss_navsat_bridge_node rclcpp sensor_msgs ublox_ubx_msgs)

add_executable(wheel_odom_node src/wheel_odom_node.cpp)
ament_target_dependencies(wheel_odom_node rclcpp std_msgs nav_msgs)

add_executable(status_info_node src/status_info_node.cpp)
target_include_directories(status_info_node PRIVATE ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(status_info_node rclcpp std_msgs geometry_msgs sensor_msgs nav_msgs)
target_link_libraries(status_info_node Eigen3::Eigen)

add_executable(eskf_node src/eskf_node.cpp)
target_include_directories(eskf_node PRIVATE ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(eskf_node rclcpp std_msgs geometry_msgs sensor_msgs nav_msgs)
target_link_libraries(eskf_node Eigen3::Eigen)

add_executable(gps_odom_node src/gps_odom_node.cpp)
ament_target_dependencies(gps_odom_node rclcpp std_msgs geometry_msgs sensor_msgs nav_msgs)

# ── 로컬 경로 발행 노드 ─────────────────────────────────────
add_executable(local_path_pub_node src/local_path_pub_cpp.cpp)
target_include_directories(local_path_pub_node PRIVATE ${INC})
ament_target_dependencies(local_path_pub_node rclcpp std_msgs geometry_msgs nav_msgs)

# ── Python 스크립트 (GUI/제어/네비) ────────────────────────────
install(PROGRAMS
  control/teleop_node.py
  control/serial_bridge_node.py
  navigation/path_file_loader_node.py
  navigation/traveled_path_node.py
  sensing/gnss_nmea_node.py
  sensing/ntrip_client_node.py
  sensing/rtcm_serial_bridge_node.py
  sensing/heading_calib.py
  DESTINATION lib/${PROJECT_NAME})

# ── 바이너리 설치 ──────────────────────────────────────────────
install(TARGETS
  mpc_monitor_node
  mpc_path_follower_node
  mpc_logger_node
  mpc_path_maker_node
  gps_rtk_gui_node
  gnss_velocity_bridge_node
  gnss_navsat_bridge_node
  wheel_odom_node
  status_info_node
  eskf_node
  gps_odom_node
  local_path_pub_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/  DESTINATION include)
install(DIRECTORY launch    DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config    DESTINATION share/${PROJECT_NAME})
install(DIRECTORY arduino   DESTINATION share/${PROJECT_NAME})
install(DIRECTORY sensing   DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
