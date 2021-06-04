cmake_minimum_required(VERSION 3.0.2)
project(test_nodelets)

find_package(catkin REQUIRED COMPONENTS nodelet roscpp std_msgs)

catkin_package(
# LIBRARIES
 INCLUDE_DIRS include
 CATKIN_DEPENDS nodelet roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
