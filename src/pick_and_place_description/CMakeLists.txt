cmake_minimum_required(VERSION 3.8)
project(pick_and_place_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# launch, rviz, urdf 폴더를 install 디렉터리로 복사하는 규칙
install(
  DIRECTORY launch rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()