cmake_minimum_required(VERSION 2.8.3)
project(simple_test)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs message_generation)

## Add in message and service files
add_message_files(FILES raven_automove.msg ravenstate.msg)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime)

## Build talker and listeneri
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(simple_test src/main.cpp src/functions.cpp)
target_link_libraries(simple_test ${catkin_LIBRARIES})
add_dependencies(simple_test simple_test_generate_messages_cpp)

