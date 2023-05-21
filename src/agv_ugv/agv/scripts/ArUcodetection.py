#!/usr/bin/env python3
############## Task1.1 - ArUco Detection ##############

import numpy as np
import cv2
# import cv2.aruco as aruco
from cv2 import aruco, imread
import sys
import math
import time

from numpy.lib.function_base import angle


def detect_ArUco(img):
    ## function to detect ArUco markers in the image using ArUco library
    ## argument: img is the test image
    ## return: dictionary named Detected_ArUco_markers of the format {ArUco_id_no : corners}, where ArUco_id_no indicates ArUco id and corners indicates the four corner position of the aruco(numpy array)
    ## 		   for instance, if there is an ArUco(0) in some orientation then, ArUco_list can be like
    ## 				{0: array([[315, 163],
    #							[319, 263],
    #							[219, 267],
    #							[215,167]], dtype=float32)}

	Detected_ArUco_markers = {}
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	aruco_dict = aruco.getPredefinedDictionary( aruco.DICT_5X5_250 )
	parameters = aruco.DetectorParameters_create()
	corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
	# print(ids)
	# print(corners)
	# print(type(corners))
	for x , y in zip(ids,corners):
		for z,a in zip(x,y):
			Detected_ArUco_markers[z] = a

			
			
	# print(Detected_ArUco_markers)

	return Detected_ArUco_markers

def gradient(pt1,pt2):
	return (pt2[1]-pt1[1]/pt2[0]-pt1[0])
def Calculate_orientation_in_degree(Detected_ArUco_markers):
    ## function to calculate orientation of ArUco with respective to the scale mentioned in problem statement
    ## argument: Detected_ArUco_markers  is the dictionary returned by the function detect_ArUco(img)
    ## return : Dictionary named ArUco_marker_angles in which keys are ArUco ids and the values are angles (angles have to be calculated as mentioned in the problem statement)
    ##			for instance, if there are two ArUco markers with id 1 and 2 with angles 120 and 164 respectively, the
    ##			function should return: {1: 120 , 2: 164}

	ArUco_marker_angles = {}
	for x in Detected_ArUco_markers:
		points = Detected_ArUco_markers[x]
		
		top_left = points[0]
		top_right = points[1]
		bottom_right= points[2]
		bottom_left = points[3]
		

		mid_top = [(top_left[0]+top_right[0])/2 , (top_left[1]+top_right[1])/2 ]
		mid_bottom= [(bottom_left[0]+bottom_right[0])/2 , (bottom_left[1]+bottom_right[1])/2]
		mid_marker_1 = [(mid_top[0]+mid_bottom[0])/2,(mid_top[1]+mid_bottom[1])/2]
		
		# print(mid_marker_1)
		angle = (math.atan2(((top_right[0])-(top_left[0])),((top_right[1]-top_left[1]))))
		# print((round(math.degrees(angle)))%360)
		if angle<0:
			angle = (round(math.degrees(angle)))+360
		else:
			angle = round(math.degrees(angle))
	



		# ArUco_marker_angles[x] = (round(math.degrees(angle)))%360
		print(angle)
		ArUco_marker_angles[x] = angle
		

		



	## enter your code here ##

	return ArUco_marker_angles  ## returning the angles of the ArUco markers in degrees as a dictionary


def mark_ArUco(img, Detected_ArUco_markers, ArUco_marker_angles):
	
	## function to mark ArUco in the test image as per the instructions given in problem statement
	## arguments: img is the test image
	##			  Detected_ArUco_markers is the dictionary returned by function detect_ArUco(img)
	##			  ArUco_marker_angles is the return value of Calculate_orientation_in_degree(Detected_ArUco_markers)
	## return: image namely img after marking the aruco as per the instruction given in problem statement

	## enter your code here ##
	for x in Detected_ArUco_markers:
		points = Detected_ArUco_markers[x]
		top_left = [ int(x) for x in points[0]]
		
		top_right = [ int(x) for x in points[1]]
		bottom_right= [ int(x) for x in points[2]]
		bottom_left = [ int(x) for x in points[3]]
		mid_top = [int((top_left[0]+top_right[0])/2) , int((top_left[1]+top_right[1])/2) ]
		mid_bottom= [int((bottom_left[0]+bottom_right[0])/2) , int((bottom_left[1]+bottom_right[1])/2)]
		centre = (int((mid_bottom[0]+mid_top[0])/2),int((mid_bottom[1]+mid_top[1])/2))
		angle = ArUco_marker_angles[x]
		cv2.circle(img,centre,4,(0,0,255),4)
		cv2.circle(img,(top_left),4,(125,125,125),4)
		cv2.circle(img,top_right,4,(0,255,0),4)
		cv2.circle(img,bottom_right,4,(180,105,255),4)
		cv2.circle(img,bottom_left,4,(255,255,255),4)
		cv2.line(img,centre,mid_top,(255,0,0),4)

		# cv2.line(img,centre,(centre[0]+100,centre[1]),(255,255,0),4)
		# cv2.line(img,(0,0),(50,0),(255,255,0),4)
		cv2.line(img,top_left,top_right,(0,0,0),4)
		cv2.putText(img,"SS_1876",(0,30),cv2.FONT_HERSHEY_SIMPLEX,
		0.7, (0,0,255), 2)

		cv2.line(img,top_left,top_right,(0,0,0),4)
		cv2.putText(img,str(x),(top_right[0] + 50, top_right[1]+50),cv2.FONT_HERSHEY_SIMPLEX,
		1, (0,0,255), 2)
		cv2.putText(img,str(angle),(top_left[0] + 20 , top_left[1]+50),cv2.FONT_HERSHEY_SIMPLEX,
		1, (0,225,0), 2)





	# cv2.line(img, topLeft, topRight, (0, 255, 0), 2)
	# cv2.line(img, topRight, bottomRight, (0, 255, 0), 2)
	# cv2.line(img, bottomRight, bottomLeft, (0, 255, 0), 2)
	# cv2.line(img, bottomLeft, topLeft, (0, 255, 0), 2)
	# # compute and draw the center (x, y)-coordinates of the
	# # ArUco marker
	# cX = int((topLeft[0] + bottomRight[0]) / 2.0)
	# cY = int((topLeft[1] + bottomRight[1]) / 2.0)
	# cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)
	# # draw the ArUco marker ID on the frame
	# cv2.putText(frame, str(markerID),
	# 	(topLeft[0], topLeft[1] - 15),
	# 	cv2.FONT_HERSHEY_SIMPLEX,
	# 	0.5, (0, 255, 0), 2)



	return img
