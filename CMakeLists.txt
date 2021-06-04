cmake_minimum_required(VERSION 3.0.2)
project(test_nodelets)

find_package(catkin REQUIRED COMPONENTS nodelet roscpp std_msgs)

catkin_package(
 LIBRARIES diff_time_nodelet
 INCLUDE_DIRS include
 CATKIN_DEPENDS nodelet roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(diff_time_nodelet src/calc_diff_time_nodelet.cpp)
add_dependencies(diff_time_nodelet ${catkin_EXPORTED_TARGETS})
target_link_libraries(diff_time_nodelet ${catkin_LIBRARIES})
