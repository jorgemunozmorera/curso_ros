#!/usr/bin/env python
import cv2
import rospy
from sensor_msgs.msg import Image # Our sensor_msgs/Image message.
from cv_bridge import CvBridge, CvBridgeError # CvBridge for convertin OpenCV to ROS.

def main():
    rospy.init_node('camera_driver', anonymous=True)
    # Open the default camera. 0 stands for /dev/video0
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360);
    cap.set(cv2.CAP_PROP_FPS, 20);
    image_pub = rospy.Publisher('image', Image, queue_size=20) # As always.
    rate = rospy.Rate(20) # 20hz.
    bridge = CvBridge() # Instantiate CvBridge object.
    while not rospy.is_shutdown():
        ret_val, cv_img = cap.read()
        try:
            # Convert the cv image to ros, given the encoding, and publish.
            # OpenCV uses the bgr8 encoding by default.
            image_pub.publish(bridge.cv2_to_imgmsg(cv_img, "bgr8")) 
        except CvBridgeError as e: # catch any possible CvBridge exception.
            print(e)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
