#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import pyaudio
from pocketsphinx import *
from sphinxbase import *

def recognizer():
    # Init node.
    rospy.init_node('recognizer', anonymous=True)
    # Create publisher.
    pub = rospy.Publisher('/recognizer/output', String, queue_size=1000)
    # Create a decoder with certain model
    config = Decoder.default_config()
    # Hard-coded values to our language, not a good practice, use params instead.
    config.set_string('-hmm', '/home/robot/catkin_ws/src/speech_recognition/misc/cmusphinx-es-5.2/model_parameters/voxforge_es_sphinx.cd_ptm_4000')
    config.set_string('-lm', '/home/robot/catkin_ws/src/speech_recognition/misc/cmusphinx-es-5.2/etc/es-20k.lm')
    config.set_string('-dict', '/home/robot/catkin_ws/src/speech_recognition/misc/cmusphinx-es-5.2/etc/voxforge_es_sphinx.dic')
    # Disable output
    config.set_string('-logfn', '/dev/null')
    # Create decoder
    decoder = Decoder(config)    
    p = pyaudio.PyAudio()
    # Open audio stream input
    stream = p.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=1024)
    # Start audio stream
    stream.start_stream()    
    in_speech_bf = False # speech state: True = there is speech in the audio buffer, False = the opposite
    decoder.start_utt()
    rospy.loginfo("Listening...")
    # While the speech state does not change, we will read 1024 bytes from the audio buffer and check if there is speech.
    # When there is speech, then the speech state changes to True, and now we again read 1024 bytes from the audio while
    # the speech state does not change (in other words, while we keep saying words). When we stop talking, then a speech 
    # state change is detected. As the speech state is now False, then we print all the words tracked.
    while not rospy.is_shutdown():
        buf = stream.read(1024)
        if buf:
            # Decode raw audio data
            decoder.process_raw(buf, False, False)
            # Check if speech state is the same as previous iteration, on wich case we read a new buffer
            if decoder.get_in_speech() != in_speech_bf:
                in_speech_bf = decoder.get_in_speech() # get_in_speech returns true if audio buffer contains speech
                if not in_speech_bf: # When no more speech is detected, then print the hypothesis
                    decoder.end_utt()
                    msg = String(decoder.hyp().hypstr)
                    pub.publish(msg)
                    rospy.loginfo("I heard: %s", msg.data)
                    decoder.start_utt()
        else:
            break
    decoder.end_utt()
    # Stop audio stream
    stream.stop_stream()

if __name__ == '__main__':
    recognizer()
