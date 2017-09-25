cmake_minimum_required(VERSION 2.8.3)
project(simple_pub_sub)

## Support for c++11
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

## The catkin_package macro generates cmake config files for the package
catkin_package()


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#################
### PUBLISHER ###
#################

## Declare a C++ executable
#add_executable(${PROJECT_NAME}_publisher_node src/publisher.cpp)
## Rename C++ executable without prefix
#set_target_properties(${PROJECT_NAME}_publisher_node PROPERTIES OUTPUT_NAME publisher_node PREFIX "")
## Add cmake target dependencies of the executable
#add_dependencies(${PROJECT_NAME}_publisher_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## Specify libraries to link a library or executable target against
#target_link_libraries(${PROJECT_NAME}_publisher_node
#   ${catkin_LIBRARIES}
#)

##################
### SUBSCRIBER ###
##################

## Declare a C++ executable
#add_executable(${PROJECT_NAME}_subscriber_node src/subscriber.cpp)
## Rename C++ executable without prefix
#set_target_properties(${PROJECT_NAME}_subscriber_node PROPERTIES OUTPUT_NAME subscriber_node PREFIX "")
## Add cmake target dependencies of the executable
#add_dependencies(${PROJECT_NAME}_subscriber_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## Specify libraries to link a library or executable target against
#target_link_libraries(${PROJECT_NAME}_subscriber_node
#   ${catkin_LIBRARIES}
#)
