cmake_minimum_required(VERSION 3.8)
project(ufomap_conversions)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)

# Compiler options for GCC and Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ufomap_msgs REQUIRED)
find_package(ufomap REQUIRED)

# Create a library from the source files
add_library(${PROJECT_NAME} SHARED
  src/conversions.cpp
)

# Specify the dependencies for the library
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  geometry_msgs
  sensor_msgs
  ufomap_msgs
  ufomap
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>  # This is for when the library is installed
)

target_link_libraries(${PROJECT_NAME}
  UFO::Map
)

# Install the library
install(TARGETS
  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

# Install the headers
install(DIRECTORY include/
  DESTINATION include
)

# Export the package
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include) 

# Ament package configuration
ament_package()
