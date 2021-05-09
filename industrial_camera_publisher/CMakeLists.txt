cmake_minimum_required(VERSION 3.0.2)
project(industrial_camera_publisher)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)
#find_package(Boost::boost)
catkin_package(
  LIBRARIES ${PROJECT_NAME}
)
add_library(camera
src/MVGE134GCT.cpp
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include/linuxSDK_V2.1.0.20/include
)
add_executable(industrial_camera_publisher
src/industrial_camera_publisher.cpp)
link_directories(
  ${catkin_LIBRARY_DIRS}
#  ${Boost_LIBRARY_DIRS}
  ${PROJECT_SOURCE_DIR}/include/linuxSDK_V2.1.0.20/lib}
 )

target_link_libraries(industrial_camera_publisher
camera
#${BOOST_LIBRARIES}
${OpenCV_LIBS}
${catkin_LIBRARIES}
-lMVSDK)
