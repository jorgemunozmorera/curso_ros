#!/usr/bin/env python
from sound_play.libsoundplay import SoundClient
from text_to_speech.srv import *
import rospy

def get_estimate(phrase): # Compute an estimate of the phrase duration as exercise
    return 0

def handle_request(req):
    print "Request for saying '%s' with voice '%s'..."%(req.phrase, req.voice)
    soundhandle = SoundClient()
    rospy.sleep(1) # This is always needed
    soundhandle.stopAll() # Stop any sounds that we are still playing.
    soundhandle.say(req.phrase, req.voice)
    rospy.sleep(1) # This is always needed
    play_estimate = get_estimate(req.phrase) # Compute an estimate of the play duration
    print "Request finished."
    return SaySomethingResponse(rospy.Duration(2 + play_estimate))

def text_to_speech_server():
    rospy.init_node('text_to_speech_server', anonymous=True)
    rospy.Service('say_something', SaySomething, handle_request) # Advertise the service 'say_something' and assign callback
    print "Ready to speak!."
    rospy.spin()

if __name__ == "__main__":
    text_to_speech_server()