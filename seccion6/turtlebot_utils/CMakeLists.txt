cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_utils)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  roscpp
  rospy
  std_msgs
  turtlesim
  rosbag
)

#################
### ACTIONLIB ###
#################
## Generate actions in the 'action' folder
add_action_files(
    FILES
    GoTo.action
)
## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs 
  std_msgs
  turtlesim
)

## The catkin_package macro generates cmake config files for your package
catkin_package()

## Specify additional locations of header files
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#################
### ITERATOR ###
#################

## Declare a C++ executable
#add_executable(${PROJECT_NAME}_iterator_node src/iterator.cpp)
## Rename C++ executable without prefix
#set_target_properties(${PROJECT_NAME}_iterator_node PROPERTIES OUTPUT_NAME iterator_node PREFIX "")
## Add cmake target dependencies of the executable
#add_dependencies(${PROJECT_NAME}_iterator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## Specify libraries to link a library or executable target against
#target_link_libraries(${PROJECT_NAME}_iterator_node
#   ${catkin_LIBRARIES}
#)


