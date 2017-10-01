#!/usr/bin/env python
import rospy
import rosbag
from geometry_msgs.msg import Twist

def bag_iterator_publisher():
    rospy.init_node('bag_iterator', anonymous=True)
    bag = rosbag.Bag('/home/jorge/2017-09-20-12-13-11.bag') # Put your bag path here.
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10) # Remember: hard-coding is a bad practice.
    rospy.sleep(2) # See what happens if we comment this line.
    rate = rospy.Rate(0.5) # See what happens if we comment this line.
    for topic, msg, t in bag.read_messages(topics=['/turtle1/cmd_vel']): # read_messages returns a generator of tuples of (str, genpy.Message, genpy.Time)
        rospy.loginfo('Topic: %s', topic)
        rospy.loginfo('Message: \n%s', msg)
        rospy.loginfo('Time: %d.%d', t.secs, t.nsecs)
        rospy.loginfo('########')
        pub.publish(msg)
        rate.sleep() # See what happens if we comment this line.
    bag.close()

if __name__ == '__main__':
    try:
        bag_iterator_publisher()
    except rospy.ROSInterruptException:
        pass
