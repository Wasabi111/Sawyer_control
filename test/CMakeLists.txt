# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(test)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(test_1 src/test.cpp)
target_link_libraries(test_1 ${catkin_LIBRARIES})

# %EndTag(FULLTEXT)%

