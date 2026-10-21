cmake_minimum_required(VERSION 3.8)
project(pick_and_place_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pick_and_place_interfaces REQUIRED)

add_executable(arm_sim_node src/arm_sim_node.cpp)

add_executable(gripper_server_node src/gripper_server_node.cpp)

ament_target_dependencies(arm_sim_node
  rclcpp
  sensor_msgs
  pick_and_place_interfaces
)

ament_target_dependencies(gripper_server_node
  rclcpp
  sensor_msgs
  pick_and_place_interfaces
)

install(TARGETS
  arm_sim_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  arm_sim_node
  gripper_server_node
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
