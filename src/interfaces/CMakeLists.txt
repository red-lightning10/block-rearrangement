cmake_minimum_required(VERSION 3.8)
project(interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_c REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

if(ament_cmake_python_FOUND)
  ament_export_dependencies(ament_cmake_python)
endif()

set(msg_files
  "msg/Detection.msg"
  "msg/DetectionArray.msg"
)
set(srv_files
  "srv/ProjectTo3D.srv"
  "srv/GetSegmentation.srv"
  "srv/GetPlacement.srv"
  "srv/MoveToPose.srv"
  "srv/PickUp.srv"
  "srv/ClearObjects.srv"
  "srv/GetAction.srv"
  "srv/GetGrounding.srv"
  "srv/Place.srv"
  "srv/AdjustObjects.srv"
)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES
  geometry_msgs
  std_msgs
  sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(rosidl_default_generators)
ament_export_dependencies(rosidl_typesupport_c)

ament_package()
