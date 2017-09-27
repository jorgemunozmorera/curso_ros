#!/usr/bin/env python
# ROS imports
import rospy
from std_msgs.msg import Float64

# Other imports needed to read free memory (RAM)
# This import allows to use regular expresions.
import re

def memory_publisher():
    rospy.init_node('memory_publisher', anonymous=True) # (node name, set unique name)    
    pub = rospy.Publisher('memory', Float64, queue_size=1000) # (topic name, message type, queue size)    
    rate = rospy.Rate(1) # 1 Hz
    msg = Float64(0.0)
    while not rospy.is_shutdown(): # returns true if CTRL+C
        get_free_memory(msg)
        rospy.loginfo("Sending free memory value: %f", msg.data)
        pub.publish(msg)
        rate.sleep() # sleep the necessary to iterate at 1 Hz

def get_free_memory(msg):
    # Open the system file that holds the memory info.
    with open('/proc/meminfo') as fp:
        # The first line contains the total memory value. Not interested.
        fp.readline()
        # The second line contains the free memory value. Interested!.
        line = fp.readline()
        str_list = re.split('[\t| ]+', line) # Split the line to get the number (in position 1)
        msg.data = long(str_list[1]) / 1024.0 # From kilobytes to megabytes.

if __name__ == '__main__':
    try:
        memory_publisher()
    except rospy.ROSInterruptException:
        pass
