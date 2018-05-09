cmake_minimum_required(VERSION 2.8.3)
project(lowrance_comms)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package()

include_directories(include
  ${catkin_INCLUDE_DIRS}
  include/interface
)

## Add all source files for the lowrance_comms_interface node
add_executable(${PROJECT_NAME}_interface
    src/interface/command_sender.cpp
    src/interface/data_publishing.cpp
    src/interface/main.cpp
    src/interface/network_configuration.cpp
    src/interface/message_receiver.cpp
    src/interface/radar_configurator.cpp
)
target_link_libraries(${PROJECT_NAME}_interface ${catkin_LIBRARIES})
