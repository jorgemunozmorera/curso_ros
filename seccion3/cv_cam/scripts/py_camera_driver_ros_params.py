#!/usr/bin/env python
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def get_param(name, default_value):
    if not rospy.has_param(name):
        rospy.logwarn("Param '%s' not found. Using default value.", name)    
    value = rospy.get_param(name, default_value) # We can give a default value if param does not exists
    return value        

def main():
    rospy.init_node('camera_driver', anonymous=True)    
    # Recover private parameters from the launch file
    width = get_param('~width', 640) # We must put a '~' char before the param name
    height = get_param('~height', 360)
    fps = get_param('~fps', 20)
    # Open the default camera. 0 stands for /dev/video0
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width);
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height);
    cap.set(cv2.CAP_PROP_FPS, fps);
    image_pub = rospy.Publisher('image', Image, queue_size=20)
    rate = rospy.Rate(fps)
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

