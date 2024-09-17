cmake_minimum_required(VERSION 3.10)
project(ufomap_ros)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ufomap REQUIRED)

set(SOURCE_LIST
  src/main.cpp
  src/conversions.cpp)

# Specify the source files for the node
add_executable(${PROJECT_NAME} ${SOURCE_LIST})

target_include_directories(${PROJECT_NAME} PRIVATE include)

# Export dependencies for ROS 2
ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs geometry_msgs sensor_msgs ufomap)

# Link libraries
target_link_libraries(${PROJECT_NAME} UFO::Map)

# Set compile features
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

# Install the node and other resources
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})
  

ament_package()