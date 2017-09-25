#!/usr/bin/env python
# ROS imports
import rospy
from custom_pub_sub.msg import Memory

def callback(msg):
    rospy.loginfo("Received memory message of time: %d.%d", msg.header.stamp.secs, msg.header.stamp.nsecs)
    print 'Total memory: ' + str(msg.total.data)
    print 'Free memory: ' + str(msg.free.data)
    print 'Available memory: ' + str(msg.available.data)
    print 'Buffers memory: ' + str(msg.buffers)
    print ''
    
def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('memory_subscriber', anonymous=True)
    rospy.Subscriber("memory", Memory, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
