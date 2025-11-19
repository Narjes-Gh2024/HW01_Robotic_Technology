cmake_minimum_required(VERSION 3.8)
project(robot_estimation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ------------------ Dependencies ------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)

# ------------------ Executables ------------------

# 1) Low-pass filter node
add_executable(low_pass_filter_node scripts/low_pass_filter.cpp)
ament_target_dependencies(low_pass_filter_node
  rclcpp
  sensor_msgs
)

# 2) Bias correction node
add_executable(bias_correct_imu_node scripts/bias_correct_imu.cpp)
ament_target_dependencies(bias_correct_imu_node
  rclcpp
  sensor_msgs
)

# 3) Complementary yaw node
add_executable(complementary_yaw_node scripts/complementary_yaw_node.cpp)
ament_target_dependencies(complementary_yaw_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
)

# 4) Diff-drive controller node
add_executable(diff_drive_controller_node scripts/diff_drive_controller.cpp)
ament_target_dependencies(diff_drive_controller_node
  rclcpp
  geometry_msgs
  std_msgs
)

# 5) Motor command node
add_executable(motor_command_node scripts/motor_command.cpp)
ament_target_dependencies(motor_command_node
  rclcpp
  std_msgs
)


# 6) Motion model odometry node
add_executable(motion_model_odom_node scripts/motion_model_odom.cpp)
ament_target_dependencies(motion_model_odom_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)

# ------------------ Install ------------------

install(
  DIRECTORY scripts launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS
    low_pass_filter_node
    bias_correct_imu_node
    complementary_yaw_node
    diff_drive_controller_node
    motor_command_node
    motion_model_odom_node
  DESTINATION lib/${PROJECT_NAME}
)

# ------------------ Tests ------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

