cmake_minimum_required(VERSION 3.8)
project(tomato_xarm6)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wunused-parameter)
endif()

set(Open3D_DIR "/home/lxianglabxing/open3d_install/lib/cmake/Open3D")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(message_filters REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Open3D REQUIRED)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${Open3D_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

# Source files
set(SOURCES
    src/xarm6_moveit.cpp
    src/csv_parser.cpp
    src/image_subscribe.cpp
    src/unique_point_cloud.cpp
    src/environment_info.cpp
    src/planar_robot.cpp
    src/hitscan.cpp
)

# Create a library from the sources
add_library(${PROJECT_NAME} SHARED ${SOURCES})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  ${PROJECT_NAME}
  moveit_ros_planning_interface
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

target_link_libraries(${PROJECT_NAME} Open3D::Open3D yaml-cpp)

# Export include directories so other packages can find headers
ament_export_include_directories(
  include
)

# Export library targets
ament_export_libraries(${PROJECT_NAME})

# Export dependencies for downstream packages
ament_export_dependencies(
  rclcpp
  moveit_ros_planning_interface
  sensor_msgs
  cv_bridge
  OpenCV
)

# Optionally build executables that use this library
add_executable(tomato_xarm6_node src/tomato_xarm6.cpp)
target_link_libraries(tomato_xarm6_node ${PROJECT_NAME})
ament_target_dependencies(tomato_xarm6_node
  moveit_ros_planning_interface
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

add_executable(tomato_data_gen_node src/tomato_data_gen.cpp)
target_link_libraries(tomato_data_gen_node ${PROJECT_NAME})
ament_target_dependencies(tomato_data_gen_node
  moveit_ros_planning_interface
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

add_executable(tomato_instance_gen_node src/tomato_instance_gen.cpp)
target_link_libraries(tomato_instance_gen_node ${PROJECT_NAME})
ament_target_dependencies(tomato_data_gen_node
  moveit_ros_planning_interface
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)
# Install the library and executables
install(TARGETS ${PROJECT_NAME} tomato_xarm6_node tomato_data_gen_node tomato_instance_gen_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install include files
install(DIRECTORY include/ DESTINATION include/)

# Install launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

ament_package()
