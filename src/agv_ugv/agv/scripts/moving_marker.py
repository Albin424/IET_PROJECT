import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
import math
from agv.msg import Marker

class ImageProcessor:
    def __init__(self):
        rospy.init_node('aruco_detection')
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        self.marker_pub = rospy.Publisher("marker_info", Marker, queue_size=10)
        self.bridge = CvBridge()

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return
        
        '''marker = Marker()
        marker.id = 1
        marker.position.x = 0.0
        marker.position.y = 0.0
        marker.position.z = 0.0

        print("Marker ID:", marker.id)
        print("Marker Position:", marker.position) '''


        detected_markers = self.detect_markers(cv_image)
        cv_image = self.draw_marker_corners(cv_image, detected_markers)
        cv_image = self.draw_marker_orientation(cv_image, detected_markers)

        cv2.imshow("ArUco Detection", cv_image)
        cv2.waitKey(1)


        for marker_id, corners in detected_markers.items():
            marker = Marker()
            marker.id = marker_id
            marker.x = (corners[0][0] + corners[2][0]) / 2.0  # Calculate the x position based on marker corners
            marker.y = (corners[0][1] + corners[2][1]) / 2.0  # Calculate the y position based on marker corners
            marker.z =  self.z_distance # Set the z position of the marker
            marker.roll = 0.0  # Set the roll angle of the marker
            marker.pitch = 0.0  # Set the pitch angle of the marker
            marker.yaw = 0.0  # Set the yaw angle of the marker
            marker.angle = self.angle # Set the angle of the marker

            self.marker_pub.publish(marker)


    def detect_markers(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_50)
        parameters = cv2.aruco.DetectorParameters_create()
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        detected_markers = {}
        if ids is not None:
            for idx, marker_id in enumerate(ids):
                detected_markers[marker_id[0]] = corners[idx][0]

        return detected_markers

    def draw_marker_corners(self, img, detected_markers):
        if detected_markers:
            for marker_id, corners in detected_markers.items():
                top_left = tuple(corners[0].astype(int).ravel())
                top_right = tuple(corners[1].astype(int).ravel())
                bottom_right = tuple(corners[2].astype(int).ravel())
                bottom_left = tuple(corners[3].astype(int).ravel())
                centre = tuple(((top_left[0] + bottom_right[0]) // 2, (top_left[1] + bottom_right[1]) // 2))
                mid_top = tuple(((top_left[0] + top_right[0]) // 2, (top_left[1] + top_right[1]) // 2))

                cv2.circle(img, centre, 4, (0, 0, 255), 4)
                cv2.circle(img, top_left, 4, (125, 125, 125), 4)
                cv2.circle(img, top_right, 4, (0, 255, 0), 4)
                cv2.circle(img, bottom_right, 4, (180, 105, 255), 4)
                cv2.circle(img, bottom_left, 4, (255, 255, 255), 4)
                cv2.line(img, centre, mid_top, (255, 0, 0), 4)
                cv2.line(img, top_left, top_right, (255, 255, 255), 4)

                #Caluclatiing the Z axis distance 

                focal_length = 1200  # Adjust this value based on your camera and setup
                marker_size = 0.0033  # Adjust this value based on the actual marker size in meters
                self.z_distance = focal_length * marker_size / (bottom_right[1] - top_right[1])

                # Print the Z-axis distance
                print("Marker ID:", marker_id)
                print("Z-axis Distance:", self.z_distance)
                #print("bottom_right:" ,bottom_right[1])
                #print("top_right:" ,top_right[1])


                # Add text showing the marker ID
                text_pos = tuple(corners[0].astype(int).ravel())
                cv2.putText(img, str(marker_id), text_pos, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        return img
    


    def draw_marker_orientation(self, img, detected_markers):
        if detected_markers:
            for marker_id, corners in detected_markers.items():
                top_left = tuple(corners[0].astype(int).ravel())
                top_right = tuple(corners[1].astype(int).ravel())

                # Calculate the orientation angle
                self.angle = round(math.degrees(math.atan2(top_right[1] - top_left[1], top_right[0] - top_left[0])))

                # Draw a line from top_left to top_right with color white
                cv2.line(img, top_left, top_right, (255, 255, 255), 4)

                # Add text showing the marker ID
                text_pos = tuple(corners[0].astype(int).ravel())
                cv2.putText(img, str(marker_id), text_pos, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

                # Add text showing the angle
                angle_pos = (top_left[0] + 20, top_left[1] + 50)
                cv2.putText(img, str(self.angle), angle_pos, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        return img




if __name__ == '__main__':
    image_processor = ImageProcessor()
    rospy.spin()
