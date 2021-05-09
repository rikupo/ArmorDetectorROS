cmake_minimum_required(VERSION 3.0.2)
project(movie_publisher)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
)
find_package(OpenCV REQUIRED)
catkin_package(
  LIBRARIES ${PROJECT_NAME}
)

include_directories(include 
${catkin_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
)
add_executable(movie_publisher src/movie_publisher)
target_link_libraries(movie_publisher 
${catkin_LIBRARIES}
${OpenCV_LIBRARIES}
)
