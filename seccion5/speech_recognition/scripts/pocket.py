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
    config.set_string('-hmm', '/home/jorge/catkin_ws/src/speech_recognition/misc/cmusphinx-es-5.2/model_parameters/voxforge_es_sphinx.cd_ptm_4000')
    config.set_string('-lm', '/home/jorge/catkin_ws/src/speech_recognition/misc/cmusphinx-es-5.2/etc/es-20k.lm')
    config.set_string('-dict', '/home/jorge/catkin_ws/src/speech_recognition/misc/cmusphinx-es-5.2/etc/voxforge_es_sphinx.dic')
    # Disable output
    config.set_string('-logfn', '/dev/null')
    # Create decoder
    decoder = Decoder(config)    
    p = pyaudio.PyAudio()
    # Open audio stream input
    stream = p.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=1024)
    # Start audio stream
    stream.start_stream()    
    in_speech_bf = False
    decoder.start_utt()
    rospy.loginfo("Listening...")
    while not rospy.is_shutdown():
        buf = stream.read(1024)
        if buf:
            decoder.process_raw(buf, False, False)
            if decoder.get_in_speech() != in_speech_bf:
                in_speech_bf = decoder.get_in_speech()
                if not in_speech_bf:
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