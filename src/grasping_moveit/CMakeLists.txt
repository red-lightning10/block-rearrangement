cmake_minimum_required(VERSION 3.8)
project(grasping_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find build tool
find_package(ament_cmake REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  moveit_ros_planning
  moveit_task_constructor_core
  moveit_task_constructor_msgs
  interfaces
  geometry_msgs
  tf2_geometry_msgs
  tf2
  tf2_ros
  control_msgs
  rclcpp_action
  pick_place_generator
)

# Find all dependencies
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Find Eigen3 for quaternion conversions
find_package(Eigen3 REQUIRED)

# Find yaml-cpp for config loading
find_package(yaml-cpp REQUIRED)

# MoveIt utilities library
add_library(moveit_utils 
  src/utils/moveit_utils.cpp
  src/mtc_actions.cpp
)

# Use ament_target_dependencies for linking and dependencies
ament_target_dependencies(moveit_utils
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

if(TARGET moveit_ros_planning_interface::moveit_ros_planning_interface)
  target_link_libraries(moveit_utils PUBLIC moveit_ros_planning_interface::moveit_ros_planning_interface)
endif()
if(TARGET moveit_core::moveit_core)
  target_link_libraries(moveit_utils PUBLIC moveit_core::moveit_core)
endif()
if(TARGET moveit_ros_planning::moveit_ros_planning)
  target_link_libraries(moveit_utils PUBLIC moveit_ros_planning::moveit_ros_planning)
endif()

# Add our custom include directories
target_include_directories(moveit_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

# Move to pose server executable
add_executable(move_to_pose_server src/move_to_pose_server.cpp)
ament_target_dependencies(move_to_pose_server
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
target_link_libraries(move_to_pose_server moveit_utils)
target_include_directories(move_to_pose_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Move to home client executable
add_executable(move_to_home_client src/move_to_home_client.cpp)
ament_target_dependencies(move_to_home_client
  rclcpp
  interfaces
  geometry_msgs
  tf2_geometry_msgs
  tf2
  ament_index_cpp
)
target_link_libraries(move_to_home_client yaml-cpp)
target_include_directories(move_to_home_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# MTC Action server executable
add_executable(mtc_action_server src/mtc_action_server.cpp)
ament_target_dependencies(mtc_action_server
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
  ament_index_cpp
)
target_link_libraries(mtc_action_server moveit_utils yaml-cpp)
target_include_directories(mtc_action_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

# Install executables and library
install(TARGETS
  move_to_pose_server
  move_to_home_client
  mtc_action_server
  moveit_utils
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  include/
  DESTINATION include
)

install(DIRECTORY 
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_package()

