cmake_minimum_required(VERSION 3.8)
project(cpp_study)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pick_and_place_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

# joint_command_test_pub 노드 빌드 설정
add_executable(joint_command_test_pub src/joint_command_test_pub.cpp)
target_link_libraries(joint_command_test_pub
  PRIVATE
    rclcpp::rclcpp
    ${pick_and_place_interfaces_TARGETS}
)

install(TARGETS
  joint_command_test_pub
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()