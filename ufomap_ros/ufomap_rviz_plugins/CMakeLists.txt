cmake_minimum_required(VERSION 3.9)
project(ufomap_rviz_plugins 
  VERSION 1.0
  LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)

# Find ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(ufomap_msgs REQUIRED)
find_package(ufomap_ros REQUIRED)

find_package(ufomap REQUIRED)

## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)

## Find Qt5 (Qt4 is not used in ROS 2)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

set(QT_LIBRARIES Qt5::Widgets)

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc. Define QT_NO_KEYWORDS here.
# add_definitions(-DQT_NO_KEYWORDS)

## Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${rviz_common_INCLUDE_DIRS}
  ${ufomap_msgs_INCLUDE_DIRS}
  ${ufomap_ros_INCLUDE_DIRS}
)

## Automatically process Qt header files with MOC
QT5_WRAP_CPP(MOC_FILES
  # include/ufomap_rviz_plugins/ufomap_base_display.h
  include/ufomap_rviz_plugins/ufomap_display.h
  OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED -DBOOST_LEXICAL_CAST_INCLUDED 
)

## Declare the plugin library
add_library(${PROJECT_NAME} SHARED
  # src/ufomap_base_display.cpp
  src/ufomap_display.cpp
  ${MOC_FILES} 
)

## Link against required libraries
target_link_libraries(${PROJECT_NAME}
  ${QT_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${rviz_common_LIBRARIES}
  ${ufomap_LIBRARIES}
)

## Additional compile options
target_compile_options(${PROJECT_NAME}
  PRIVATE
    -Wno-register  # TODO: Should not be needed if OGRE fixes it
)

## Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

## Install the library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## Install the plugin description (for RViz)
install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

## ROS 2 package declaration
ament_package()
