cmake_minimum_required(VERSION 3.0.2)
project(test_nodelets)

find_package(catkin REQUIRED COMPONENTS nodelet roscpp std_msgs ambs_core)

catkin_package(
 LIBRARIES test_nodelets
 INCLUDE_DIRS include src
 CATKIN_DEPENDS nodelet roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  src
)

add_library(test_nodelets
    src/calc_diff_time_nodelet.cpp
    src/calc_comp_param_nodelet.cpp
    src/calc_comp_float_temporal_nodelet.cpp
    src/calc_diff_int.cpp)
add_dependencies(test_nodelets ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_nodelets ${catkin_LIBRARIES})

