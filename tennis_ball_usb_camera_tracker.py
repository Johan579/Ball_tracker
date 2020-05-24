#!/usr/bin/env python

import sys
import rospy
from cv2 import cv2     # When using VS Code got error by only using import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np 
from sensor_msgs.msg import Image
from tennis_ball_tracking import HSV, selectContours, Contours   # Importing functions from first script to avoid repetition

def main():
    rospy.Subscriber('/usb_cam/image_raw', Image, callback)
    rospy.spin()

def callback(img):
    try:
        image = bridge.imgmsg_to_cv2(img, "bgr8")
    except CvBridgeError as e:
        print(e)

    binary_image = HSV(image, (60, 100, 20), (90, 255, 255))      # Filtered image
    contours = Contours(binary_image)     # Found contours on image
    tracking_image = selectContours(image, contours, 3000)       # Draw on selected contours

    cv2.imshow("Tracking Image", tracking_image)    # Showing output

    if cv2.waitKey(1) & 0xFF == ord('q'):
        rospy.signal_shutdown("User requested")
        


if __name__ == "__main__":
    try:
        rospy.init_node('usb_camera_tracker', disable_signals=True)
        bridge = CvBridge()
        main()
    except KeyboardInterrupt:
        pass