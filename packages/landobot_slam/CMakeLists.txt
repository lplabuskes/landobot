cmake_minimum_required(VERSION 2.8.3)
project(landobot_slam)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  duckietown_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(encoder_reader src/encoder_reader.cpp)
target_link_libraries(encoder_reader ${catkin_LIBRARIES})
