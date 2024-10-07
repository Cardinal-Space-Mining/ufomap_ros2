cmake_minimum_required(VERSION 3.8)
project(ufomap_mapping)

set (CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

find_package(ufomap_msgs REQUIRED)
find_package(ufomap_conversions REQUIRED)
find_package(ufomap REQUIRED)

add_executable(${PROJECT_NAME}_node 
  src/main.cpp 
  src/ufomap_node.cpp
)

ament_target_dependencies(${PROJECT_NAME}_node 
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  std_srvs
  tf2_ros
  tf2_sensor_msgs
  diagnostic_msgs
  ufomap_msgs
  ufomap_conversions
  ufomap
)

target_include_directories(${PROJECT_NAME}_node
  PRIVATE 
    include
)

target_link_libraries(${PROJECT_NAME}_node
  UFO::Map
)

target_compile_features(${PROJECT_NAME}_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(
  TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

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
