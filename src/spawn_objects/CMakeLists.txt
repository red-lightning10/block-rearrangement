cmake_minimum_required(VERSION 3.8)
project(spawn_objects)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(include)

# Spawn objects server
add_executable(spawn_objects_server
  src/spawn_objects_server.cpp
  src/spawn_objects.cpp
)

ament_target_dependencies(spawn_objects_server
  rclcpp
  interfaces
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  shape_msgs
  geometric_shapes
  std_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  Eigen3
)

# Clear objects client
add_executable(clear_objects
  src/clear_objects_client.cpp
)

ament_target_dependencies(clear_objects
  rclcpp
  interfaces
  moveit_msgs
  moveit_ros_planning_interface
)

install(TARGETS
  spawn_objects_server
  clear_objects
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()


