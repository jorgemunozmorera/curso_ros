cmake_minimum_required(VERSION 2.8.3)
project(text_to_speech)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sound_play
  std_msgs
  message_generation
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   SaySomething.srv
)


## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)

## The catkin_package macro generates cmake config files for your package
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES text_to_speech
#  CATKIN_DEPENDS roscpp rospy sound_play std_msgs
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

##################
## SPEAKER NODE ##
##################

## Declare a C++ executable
#add_executable(${PROJECT_NAME}_server_node src/speaker_node.cpp)
## Rename C++ executable without prefix
#set_target_properties(${PROJECT_NAME}_server_node PROPERTIES OUTPUT_NAME server_node PREFIX "")
## Add cmake target dependencies of the executable
#add_dependencies(${PROJECT_NAME}_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_server_node
#   ${catkin_LIBRARIES}
#)

##################
##  TEST NODE   ##
##################

## Declare a C++ executable
#add_executable(${PROJECT_NAME}_test_node src/test_node.cpp)
## Rename C++ executable without prefix
#set_target_properties(${PROJECT_NAME}_test_node PROPERTIES OUTPUT_NAME test_node PREFIX "")
## Add cmake target dependencies of the executable
#add_dependencies(${PROJECT_NAME}_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_test_node
#   ${catkin_LIBRARIES}
#)

