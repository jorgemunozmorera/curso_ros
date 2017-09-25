cmake_minimum_required(VERSION 2.8.3)
project(speech_recognition)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

## The catkin_package macro generates cmake config files for your package
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES speech_recognition
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


## Specify additional locations of header files
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(${PROJECT_NAME}_media_node src/media_command.cpp)
## Rename C++ executable without prefix
set_target_properties(${PROJECT_NAME}_media_node PROPERTIES OUTPUT_NAME media_node PREFIX "")
## Add cmake target dependencies of the executable
add_dependencies(${PROJECT_NAME}_media_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_media_node
   ${catkin_LIBRARIES}
)

