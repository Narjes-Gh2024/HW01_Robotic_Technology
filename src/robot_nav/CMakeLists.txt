cmake_minimum_required(VERSION 3.8)
project(robot_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED) 

include_directories(
  include
)

# ---- Executables ----

add_executable(data_node scripts/data.cpp)
ament_target_dependencies(data_node
  rclcpp
  std_msgs
  sensor_msgs
)



add_executable(lpf_node scripts/lpf.cpp)
ament_target_dependencies(lpf_node
  rclcpp
  sensor_msgs
)



add_executable(bias_correct_node scripts/bias_correct.cpp)
ament_target_dependencies(bias_correct_node
  rclcpp
  sensor_msgs
)



add_executable(complementary_position_node scripts/complementary_position.cpp)
ament_target_dependencies(complementary_position_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)


# ---- Install ----


install(
  DIRECTORY scripts launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS 
    data_node
    lpf_node
    bias_correct_node
    complementary_position_node
  DESTINATION lib/${PROJECT_NAME}
)

# ---- Testing ----

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
