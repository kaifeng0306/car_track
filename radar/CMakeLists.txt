cmake_minimum_required(VERSION 2.8.3)
project(radar)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
  message_generation
)

add_message_files(
  FILES
  radar_data.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp serial std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(radar src/radar.cpp)
target_link_libraries(radar ${catkin_LIBRARIES})

