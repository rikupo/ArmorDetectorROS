cmake_minimum_required(VERSION 3.0.2)
project(armor_detector)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
  sensor_msgs
)
find_package(OpenCV REQUIRED)
catkin_package(
  LIBRARIES ${PROJECT_NAME}
)


catkin_package(
  LIBRARIES ${PROJECT_NAME}
)

add_library(armor
  src/armor.cpp
)
include_directories(include 
${catkin_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
)
add_executable(armor_detector src/armor_detector)
target_link_libraries(armor_detector
armor
${catkin_LIBRARIES}
${OpenCV_LIBRARIES}
)