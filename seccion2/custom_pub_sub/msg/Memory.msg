# The header contains information that is commonly used in ROS messages.
# It is usually used as the first field in all messages.
Header header
std_msgs/Float64 total #Megabytes
std_msgs/Float64 available #Megabytes
std_msgs/Float64 free #Megabytes
# We can use also primitive data types.
uint32 buffers #Kilobytes
