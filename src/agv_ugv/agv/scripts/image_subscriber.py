#!/usr/bin/env python3

############## Task1.1 - ArUco Detection ##############
### YOU CAN EDIT THIS FILE FOR DEBUGGING PURPOSEs, SO THAT YOU CAN TEST YOUR ArUco_library.py AGAINST THE VIDEO Undetected ArUco markers.avi###
### BUT MAKE SURE THAT YOU UNDO ALL THE CHANGES YOU HAVE MADE FOR DEBUGGING PURPOSES BEFORE TESTING AGAINST THE TEST IMAGES ###

import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ArUcodetection import *

bridge = CvBridge()

def callback(data):
    try:
        img = bridge.imgmsg_to_cv2(data, "bgr8")
        cv2.imshow("Input image", img)
        Detected_ArUco_markers = detect_ArUco(img)
        angle = Calculate_orientation_in_degree(Detected_ArUco_markers)
        cv_image = mark_ArUco(cv_image, Detected_ArUco_markers, angle)
        #cv2.imshow("ArUco detection", cv_image)
        cv2.waitKey(1)

    except Exception as e:
        print(e)

def main():
    rospy.init_node('ArUco_detection', anonymous=True)
    rospy.Subscriber("/camera/rgb/image_raw", Image, callback)
    rospy.spin()

if __name__ == '__main__':
    main()

