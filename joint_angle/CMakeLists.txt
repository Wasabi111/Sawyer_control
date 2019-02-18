# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(joint_angle)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(joint_control src/Joint_0_pos.cpp)
target_link_libraries(joint_control ${catkin_LIBRARIES})
add_executable(joint_control1 src/Joint_pos.cpp)
target_link_libraries(joint_control1 ${catkin_LIBRARIES})

# %EndTag(FULLTEXT)%

