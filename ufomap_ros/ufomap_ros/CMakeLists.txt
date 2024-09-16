cmake_minimum_required(VERSION 3.8.2)
project(ufomap_ros)

set(CMAKE_CXX_STANDARD 17)

# Find ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(ufomap REQUIRED)

# Specify include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${ufomap_INCLUDE_DIRS}
)

# Declare a static library
add_library(${PROJECT_NAME} STATIC
  src/conversions.cpp
)

# Link against necessary libraries
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    ${rclcpp_LIBRARIES}
    ${geometry_msgs_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
    UFO::Map
)

# Install the library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

# ROS 2 package declaration
ament_package()
