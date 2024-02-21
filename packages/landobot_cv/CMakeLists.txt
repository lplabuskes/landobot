cmake_minimum_required(VERSION 2.8.3)
project(landobot_cv)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(camera_reader src/camera_reader.cpp)
target_link_libraries(camera_reader ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
