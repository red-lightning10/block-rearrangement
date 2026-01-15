cmake_minimum_required(VERSION 3.8)
project(grounding)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(interfaces REQUIRED)

add_library(predicates_grounder
  src/object_registry.cpp
  src/predicates_grounder.cpp)

target_include_directories(predicates_grounder PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(predicates_grounder
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs)

add_executable(grounding_node src/grounding_node.cpp)
target_link_libraries(grounding_node predicates_grounder)
ament_target_dependencies(grounding_node
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  interfaces)

install(TARGETS predicates_grounder grounding_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

ament_package()
