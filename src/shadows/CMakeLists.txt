cmake_minimum_required(VERSION 3.8)
project(shadows)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(Open3D_DIR "/home/lxianglabxing/open3d_install/lib/cmake/Open3D")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tomato_xarm6 REQUIRED)
find_package(Open3D REQUIRED)

add_executable(shadow_data_collect src/shadow.cpp)
ament_target_dependencies(shadow_data_collect rclcpp tomato_xarm6)
target_link_libraries(shadow_data_collect Open3D::Open3D)

install(TARGETS shadow_data_collect
  DESTINATION lib/${PROJECT_NAME})

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
