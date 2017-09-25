#!/usr/bin/env python
import rospy
from pymouse import PyMouse
from std_msgs.msg import String

# Create a dictionary of applications->positions.
screen_point = {
                'explorer': (31, 118),
                'browser': (31, 174),
                'terminator': (31, 230),
                'console': (31, 293),
                'settings': (31, 352),
                'cute': (31, 414),
                'spider': (31, 470),
                'finish': (13, 13),
                }

def callback(data):
    # Get the detected word.
    word = data.data
    rospy.loginfo("I heard: %s", word)
    # Create a PyMouse instance.
    m = PyMouse()
    # If the word is in the dictionary of applications, then simulate a click over its icon.
    if word in screen_point.keys():
        point = screen_point[word]
        m.click(point[0], point[1])
    
    
def commander():
    rospy.init_node('commander', anonymous=True)
    rospy.loginfo("Listening for applications...")
    # Subscribe to the recognizer output.
    rospy.Subscriber("/recognizer/output", String, callback)
    rospy.spin()

if __name__ == '__main__':
    commander()
    
