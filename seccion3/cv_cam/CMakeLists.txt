cmake_minimum_required(VERSION 2.8.3)
project(cv_cam)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## If the following line is missing, OpenCV 2 will be used instead
find_package(OpenCV 3 REQUIRED)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  std_msgs
  sensor_msgs
)

## The catkin_package macro generates cmake config files for the package
catkin_package()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

##############################
###     OpenCV Driver      ###
##############################

## Declare a C++ executable for the OpenCV driver
#add_executable(${PROJECT_NAME}_driver_node src/camera_driver.cpp)
## Rename C++ executable without prefix
#set_target_properties(${PROJECT_NAME}_driver_node PROPERTIES OUTPUT_NAME camera_node PREFIX "")
## Add cmake target dependencies of the executable
#add_dependencies(${PROJECT_NAME}_driver_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## Specify libraries to link a library or executable target against
#target_link_libraries(${PROJECT_NAME}_driver_node
#  ${OpenCV_LIBRARIES}
#  ${catkin_LIBRARIES}
#)

##############################
###     ROS Driver         ###
##############################

## Declare a C++ executable for the ROS driver
#add_executable(${PROJECT_NAME}_ros_driver_node src/camera_driver_ros.cpp)
## Rename C++ executable without prefix
#set_target_properties(${PROJECT_NAME}_ros_driver_node PROPERTIES OUTPUT_NAME ros_camera_node PREFIX "")
## Add cmake target dependencies of the executable
#add_dependencies(${PROJECT_NAME}_ros_driver_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## Specify libraries to link a library or executable target against
#target_link_libraries(${PROJECT_NAME}_ros_driver_node
#  ${OpenCV_LIBRARIES}
#  ${catkin_LIBRARIES}
#)

##############################
###       ROS ARGS         ###
##############################

## Declare a C++ executable for the ROS driver (args version)
#add_executable(${PROJECT_NAME}_ros_driver_node_args src/camera_driver_ros_args.cpp)
## Rename C++ executable without prefix
#set_target_properties(${PROJECT_NAME}_ros_driver_node_args PROPERTIES OUTPUT_NAME ros_camera_node_args PREFIX "")
## Add cmake target dependencies of the executable
#add_dependencies(${PROJECT_NAME}_ros_driver_node_args ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## Specify libraries to link a library or executable target against
#target_link_libraries(${PROJECT_NAME}_ros_driver_node_args
#  ${OpenCV_LIBRARIES}
#  ${catkin_LIBRARIES}
#)

##############################
###       ROS PARAMS       ###
##############################

## Declare a C++ executable for the ROS driver (params version)
#add_executable(${PROJECT_NAME}_ros_driver_node_params src/camera_driver_ros_params.cpp)
## Rename C++ executable without prefix
#set_target_properties(${PROJECT_NAME}_ros_driver_node_params PROPERTIES OUTPUT_NAME ros_camera_node_params PREFIX "")
## Add cmake target dependencies of the executable
#add_dependencies(${PROJECT_NAME}_ros_driver_node_params ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## Specify libraries to link a library or executable target against
#target_link_libraries(${PROJECT_NAME}_ros_driver_node_params
#  ${OpenCV_LIBRARIES}
#  ${catkin_LIBRARIES}
#)

##############################
###     ROS TRANSPORT      ###
##############################

## Declare a C++ executable for the ROS driver (image_transport version)
#add_executable(${PROJECT_NAME}_img_transport_node src/img_transport_camera_driver.cpp)
## Rename C++ executable without prefix
#set_target_properties(${PROJECT_NAME}_img_transport_node PROPERTIES OUTPUT_NAME img_transport_camera_node PREFIX "")
## Add cmake target dependencies of the executable
#add_dependencies(${PROJECT_NAME}_img_transport_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## Specify libraries to link a library or executable target against
#target_link_libraries(${PROJECT_NAME}_img_transport_node
#  ${OpenCV_LIBRARIES}
#  ${catkin_LIBRARIES}
#)

##############################
###         TRACKER        ###
##############################

## Declare a C++ executable for the ROS driver (image_transport version)
#add_executable(${PROJECT_NAME}_img_tracker_node src/tracker.cpp)
## Rename C++ executable without prefix
#set_target_properties(${PROJECT_NAME}_img_tracker_node PROPERTIES OUTPUT_NAME tracker_node PREFIX "")
## Add cmake target dependencies of the executable
#add_dependencies(${PROJECT_NAME}_img_tracker_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## Specify libraries to link a library or executable target against
#target_link_libraries(${PROJECT_NAME}_img_tracker_node
#  ${OpenCV_LIBRARIES}
#  ${catkin_LIBRARIES}
#)
