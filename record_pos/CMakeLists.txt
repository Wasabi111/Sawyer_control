# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(record_pos)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs intera_core_msgs genmsg)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs intera_core_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(record_position src/Record_pos.cpp)
target_link_libraries(record_position ${catkin_LIBRARIES})

# %EndTag(FULLTEXT)%

