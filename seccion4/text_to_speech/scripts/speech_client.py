#!/usr/bin/env python
from text_to_speech.srv import SaySomething
import rospy


def say_something_client(phrase, voice):
    
    try:
        say_something_function = rospy.ServiceProxy('say_something', SaySomething) # Returns a function
        response = say_something_function(phrase, voice)
        return response.data # Return the Duration object of the response.
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return rospy.Duration(0, 0) # Duration 0.0 to remark an error ocurred!

if __name__ == "__main__":
    rospy.wait_for_service('say_something') # Waits until service is available
    while True:
        phrase = raw_input("Enter phrase to say (or INTRO to stop): ")
        if len(phrase) == 0:
            break
        voice = raw_input("Enter voice to speak (or INTRO to stop): ")
        if len(voice) == 0:
            break
        duration = say_something_client(phrase, voice)
        print "Play duration: %d.%d"%(duration.secs, duration.nsecs)