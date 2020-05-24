#!/usr/bin/env python

import sys
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv2 import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from tennis_ball_tracking import HSV, Contours, selectContours   # Importing functions from developed python script


def main():
        rospy.init_node('tennis_ball_subscriber', disable_signals=True)   # Initialize node
        rospy.Subscriber('/tennis_ball_image', Image, callback)     # Subscribe to topic and initialize callback function
        rospy.Subscriber('/kill_script', Bool, stopDisplay)
        rospy.spin()

def callback(img):
    try:
        image = bridge.imgmsg_to_cv2(img, "bgr8")   # CvBridge to convert message into image
    except CvBridgeError as e:
        print(e)

    binary_image = HSV(image, (30, 80, 20), (60, 255, 255))      # Filtered image
    contours = Contours(binary_image)     # Found contours on image
    tracking_image = selectContours(image, contours, 3000)       # Draw on selected contours

    cv2.imshow("Tracking Image", tracking_image)    # Showing output
    cv2.waitKey(1)

def stopDisplay(msg):
    if msg.data == False:   # If message that video has terminated is received, kill windows and shutdown node
        cv2.destroyAllWindows()     # Destroy all windows
        rospy.signal_shutdown("Video terminated")   # Shutdown node

if __name__ == "__main__":
    try:
        bridge = CvBridge()
        main()
    except KeyboardInterrupt:
        pass