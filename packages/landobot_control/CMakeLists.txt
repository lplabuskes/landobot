cmake_minimum_required(VERSION 2.8.3)
project(landobot_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  duckietown_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(wheel_control src/wheel_control.cpp)
target_link_libraries(wheel_control ${catkin_LIBRARIES})
