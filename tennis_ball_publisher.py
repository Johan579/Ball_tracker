#!/usr/bin/env python

import sys
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv2 import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError


def main():
    img = cv2.VideoCapture('/home/johan/catkin_ws/src/ball_tracking/src/tennis-ball-video.mp4')  # Capturing video from path

    while True:
        ret, frame = img.read()     # Reading video frame by frame
        if ret == True:     # If reading video execute
            try:
                img_publish = bridge.cv2_to_imgmsg(frame, "bgr8")   # Converting frame into image message
                image_pub.publish(img_publish)    # Publishing image message
                rate.sleep()
            except CvBridgeError as e:
                print(e)

        else:   # When stop reading break
            break
    
    bool_pub.publish(ret)
    img.release()

if __name__ == "__main__":
    try:
        rospy.init_node('tennis_ball_publisher', disable_signals=True)    # Initialize node
        image_pub = rospy.Publisher('/tennis_ball_image', Image, queue_size=1)   # Create image publisher
        bool_pub = rospy.Publisher('/kill_script', Bool, queue_size=1)   # Publisher to stop displaying and kill subscriber script
        bridge = CvBridge()
        rate = rospy.Rate(20)
        main()
    except rospy.ROSInterruptException:
        pass