#!/usr/bin/env python
# ROS imports
import rospy
from std_msgs.msg import Float64

# This function executes any time a Float64 message arrives.
# It executes on its own thread.
def callback(msg):
    rospy.loginfo("Received free memory: %f", msg.data)
    
def memory_subscriber():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('memory_subscriber', anonymous=True)
    rospy.Subscriber("memory", Float64, callback) # The topic name must match the one on the publisher, also with the message type.
    rospy.spin() # spin() simply keeps python from exiting until this node is stopped with CTRL+C

if __name__ == '__main__':
    memory_subscriber()
