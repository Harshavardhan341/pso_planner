cmake_minimum_required(VERSION 3.0.2)
project(pso_planner)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs

)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
add_executable(position_controller_node src/position_controller.cpp)

target_link_libraries(position_controller_node
   ${catkin_LIBRARIES}
)