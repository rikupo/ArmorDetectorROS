cmake_minimum_required(VERSION 3.0.2)
project(serial_com)

#add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)
find_package(
  Boost REQUIRED COMPONENTS
  system
)
catkin_package(
  LIBRARIES ${PROJECT_NAME}
)
add_executable(serial_com src/serial_com.cpp)

include_directories(include 
${catkin_INCLUDE_DIRS}
)


target_link_libraries(serial_com
${catkin_LIBRARIES}
${Boost_LIBRARIES}
)

