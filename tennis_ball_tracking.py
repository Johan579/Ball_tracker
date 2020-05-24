#!/usr/bin/env python

from cv2 import cv2     # When using VS Code got error by only using import cv2
import numpy as np 

def HSV(img, lowerbound, upperbound):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)      # Convert to HSV
    mask = cv2.inRange(hsv, lowerbound, upperbound)     # Apply mask to HSV image
    return mask

def Contours(img):
    contours, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)      # Find contours of binary image
    return contours

def selectContours(img, contours, range):
    for c in contours:
        area = cv2.contourArea(c)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        if area > range:     # If area of found contour is greater than 1000 execute
            cx, cy = contourCenter(c)       # Center of center
            cv2.circle(img, (cx, cy), int(radius), (0, 0, 255), thickness=5)    # Draw a circle around found contour
    return img

def contourCenter(contour):
    M = cv2.moments(contour)        #Get moments from found contour
    cx = -1
    cy = -1
    if M['m00'] != 0:  
        cx = int(M['m10']/M['m00'])     # X moment coordinate
        cy = int(M['m01']/M['m00'])     # Y moment coordinate
    return cx, cy

def main():
            
    img = cv2.VideoCapture('../src/tennis-ball-video.mp4')     # Capture video from path
    
    while(True):
        ret, frame = img.read()     # Capture every frame of video

        if ret == True:     # If reading video is true execute code
            binary_image = HSV(frame, (30, 80, 20), (60, 255, 255))      # Filtered image
            contours = Contours(binary_image)     # Found contours on image
            tracking_image = selectContours(frame, contours, 3000)       # Draw on selected contours

            cv2.imshow("Tracking Image", tracking_image)    # Showing output

            if cv2.waitKey(10) & 0xFF == ord('q'):
                break
        else:       # Video didn't load or finished
            print('Video didnt load or finished')
            break

    img.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass