cmake_minimum_required(VERSION 3.8)
project(ufomap_mapping)

set(CMAKE_CXX_STANDARD 17)

# Find ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(ufomap REQUIRED)
find_package(ufomap_msgs REQUIRED)
find_package(ufomap_ros REQUIRED)

# Optional: Find dynamic reconfigure dependencies (if you have dynamic reconfigure support in ROS 2)
find_package(dynamic_reconfigure REQUIRED)

# Generate dynamic reconfigure options (reconfigure is supported in ROS 2)
generate_dynamic_reconfigure_options(cfg/Server.cfg)

###################################
## ament specific configuration ##
###################################
# Declare ament package (replaces catkin_package)
ament_package(
  INCLUDE_DIRS include
  # LIBRARIES ufomap_mapping_server_nodelet
  # DEPENDS other non-ament dependencies
)

###########
## Build ##
###########

# Specify include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${ufomap_INCLUDE_DIRS}
)

# Declare a C++ executable
add_executable(${PROJECT_NAME}_server_node
  src/server_node.cpp
  src/server.cpp
)

# Add target dependencies for dynamic reconfigure and message generation
ament_target_dependencies(${PROJECT_NAME}_server_node
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_sensor_msgs
  geometry_msgs
  ufomap_msgs
  ufomap_ros
)

# Link libraries to the target
target_link_libraries(${PROJECT_NAME}_server_node
  UFO::Map
)

#############
## Install ##
#############

# Install the C++ executable
install(TARGETS ${PROJECT_NAME}_server_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

#############
## Testing ##
#############

# Enable testing if necessary (similar to ROS 1, you can add tests here)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Add gtest for C++ tests
  ament_add_gtest(${PROJECT_NAME}_test test/test_ufomap_mapping.cpp)
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
endif()
