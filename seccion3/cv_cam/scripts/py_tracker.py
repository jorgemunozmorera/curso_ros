#!/usr/bin/env python
import cv2
import sys
import rospy
 
def main():
    # Show help
    if len(sys.argv) < 2:
        print "Usage: tracker <algorithm name>"
        sys.exit()    
    # Init ROS
    rospy.init_node('tracker', anonymous=True)
    # Create a tracker object
    # Possible algorithms: BOOSTING, KCF, TLD or MEDIANFLOW (OpenCV 3.1)
    # Don't use KCF in opencv 3.1, it has a bug.    
    tracker = cv2.Tracker_create(sys.argv[1]) 
    cap = cv2.VideoCapture(0) 
    if not cap.isOpened():
        print "Unable to open default camera device"
        sys.exit() 
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320);
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240);
    cap.set(cv2.CAP_PROP_FPS, 10);
    # Read first frame.
    ok, frame = cap.read()
    if not ok:
        print 'Cannot read camera file'
        sys.exit()     
    bbox = cv2.selectROI(frame, False)
    # This is needed in Python because the window remains opened
    cv2.destroyAllWindows()
    # Initialize tracker with first frame and bounding box
    ok = tracker.init(frame, bbox)
    # Perform the tracking process
    print "Start the tracking process, press CTRL + C to quit."
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # Get frame from the camera
        ok, frame = cap.read()
        # Stop the program if no more images
        if not ok:
            break         
        # Update the tracking result
        ok, bbox = tracker.update(frame) 
        # Draw the tracked object
        if ok:
            p1 = (int(bbox[0]), int(bbox[1]))
            p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
            cv2.rectangle(frame, p1, p2, (255,0,0), 2, 1) 
        # Show image with the tracked object
        cv2.imshow("Tracking", frame) 
        # Still need waitkey 
        cv2.waitKey(1)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
