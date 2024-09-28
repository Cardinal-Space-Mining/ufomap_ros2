cmake_minimum_required(VERSION 3.8)
project(ufomap_msgs)

# Use C++17 standard
set(CMAKE_CXX_STANDARD 17)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)  # For message/service generation
find_package(geometry_msgs REQUIRED)              # For message dependencies
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)                     # For conversion code
find_package(ufomap REQUIRED)                     # UFOMap dependency

# Define message and service files
set(MSG_FILES
  "msg/AABB.msg"
  "msg/BoundingVolume.msg"
  # "msg/Capsule.msg"
  # "msg/Cone.msg"
  # "msg/Cylinder.msg"
  # "msg/Ellipsoid.msg"
  "msg/Frustum.msg"
  "msg/LineSegment.msg"
  "msg/OBB.msg"
  "msg/Plane.msg"
  "msg/Ray.msg"
  "msg/Sphere.msg"
  # "msg/Triangle.msg"
  "msg/UFOMap.msg"
  "msg/UFOMapMetaData.msg"
  "msg/UFOMapStamped.msg"
)

set(SRV_FILES
  "srv/ClearVolume.srv"
  "srv/GetMap.srv"
  "srv/Reset.srv"
  "srv/SaveMap.srv"
)

# Generate message and service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${SRV_FILES}
  DEPENDENCIES geometry_msgs std_msgs
)

# Install message and service definitions
install(DIRECTORY msg/ srv/
  DESTINATION share/${PROJECT_NAME}
)

# Ensure the generated message headers are included
include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}  # For generated message headers
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

# Export package dependencies to ensure other packages can use the generated messages and services
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(geometry_msgs std_msgs rclcpp ufomap)

# Linting and testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ROS 2 package declaration
ament_package()
