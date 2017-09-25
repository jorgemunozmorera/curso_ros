#!/usr/bin/env python
# ROS imports
import rospy
from custom_pub_sub.msg import Memory
# Other imports needed to read free memory (RAM)
import re

def talker():
    pub = rospy.Publisher('memory', Memory, queue_size=1000)
    rospy.init_node('memory_publisher', anonymous=True)
    rate = rospy.Rate(1) # 1 Hz
    counter = 0
    msg = Memory()
    while not rospy.is_shutdown():
        msg.header.seq = counter
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "0"
        get_memory(msg)
        rospy.loginfo("Sending memory message of time: %d.%d", msg.header.stamp.secs, msg.header.stamp.nsecs)
        pub.publish(msg)
        rate.sleep()
        counter += 1

def get_memory(msg):
    # Open the system file that holds the memory info.
    with open('/proc/meminfo') as fp:        
        # The first line contains the total memory value. Interested!.
        line = fp.readline()
        msg.total.data = get_number(line) / 1024.0
        # The second line contains the free memory value. Interested!.
        line = fp.readline()
        msg.free.data = get_number(line) / 1024.0
        # The third line contains the available memory value. Interested!.
        line = fp.readline()
        msg.available.data = get_number(line) / 1024.0
        # The fourth line contains the buffers memory. Interested!.
        line = fp.readline()
        msg.buffers = get_number(line)

def get_number(line):
    str_list = re.split('[\t| ]+', line)
    return long(str_list[1])    

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
