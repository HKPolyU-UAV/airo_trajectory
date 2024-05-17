cmake_minimum_required(VERSION 3.0.2)
project(airo_trajectory)
#lala
## Compile as C++17, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  roslaunch
  geometry_msgs
  mavros_msgs
  message_runtime
  tf
  airo_control
  airo_message
)

find_package(Eigen3 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES airo_trajectory
 CATKIN_DEPENDS roscpp message_runtime airo_control airo_message
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(trajectory_example_node
    src/trajectory_example_node.cpp
    src/airo_trajectory_server.cpp)
target_link_libraries(trajectory_example_node
  ${catkin_LIBRARIES}
)
add_dependencies(example_mission_node airo_message_generate_messages_cpp)

add_executable(file_trajectory_node
    src/file_trajectory_node.cpp
    src/airo_trajectory_server.cpp)
target_link_libraries(file_trajectory_node
  ${catkin_LIBRARIES}
)
add_dependencies(file_trajectory_node airo_message_generate_messages_cpp)

add_executable(position_setpoint_node
    src/position_setpoint_node.cpp)
target_link_libraries(position_setpoint_node
  ${catkin_LIBRARIES}
)