#!/usr/bin/env python
import cv2

def main():
    # Open the default camera. 0 stands for /dev/video0
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360);
    cap.set(cv2.CAP_PROP_FPS, 20);
    while cv2.waitKey(50) != 27: # Waits 50 milliseconds, if user presses 'ESC' then exit.
        ret_val, img = cap.read()
        cv2.imshow('Webcam', img)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
