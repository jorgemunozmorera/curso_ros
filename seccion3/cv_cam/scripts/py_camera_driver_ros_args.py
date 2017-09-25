#!/usr/bin/env python
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys # Needed to access the argv array

def main():
    rospy.init_node('camera_driver', anonymous=True)
    # Open the default camera. 0 stands for /dev/video0
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(sys.argv[1]));
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(sys.argv[2]));
    cap.set(cv2.CAP_PROP_FPS, int(sys.argv[3]));
    image_pub = rospy.Publisher('image', Image, queue_size=20)
    rate = rospy.Rate(int(sys.argv[3]))
    bridge = CvBridge()
    while not rospy.is_shutdown():
        ret_val, cv_img = cap.read()
        try:
            image_pub.publish(bridge.cv2_to_imgmsg(cv_img, "bgr8"))
        except CvBridgeError as e:
            print(e)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
