cmake_minimum_required(VERSION 2.8.3)
project(AutoCircle_generater)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs message_generation)

## Add in message and service files
add_message_files(FILES raven_automove.msg raven_state.msg)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime)

## Build talker and listeneri
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(talkerAutoCircle src/talker.cpp  src/Raven_Controller.cpp src/Raven_PathPlanner.cpp)
target_link_libraries(talkerAutoCircle ${catkin_LIBRARIES})
add_dependencies(talkerAutoCircle AutoCircle_test_generate_messages_cpp)

add_executable(listenerAutoCircle src/listener.cpp)
target_link_libraries(listenerAutoCircle ${catkin_LIBRARIES})
add_dependencies(listenerAutoCircle AutoCircle_test_generate_messages_cpp)
