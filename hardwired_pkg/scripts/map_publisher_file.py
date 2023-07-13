#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
# from nav_msgs.msg import OccupancyGrid
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


rospy.init_node('map_publisher',anonymous=True)
map_pub = rospy.Publisher('map',OccupancyGrid,queue_size=1)
occupancy_matrix = OccupancyGrid()
# Capture the video feed
cap = cv2.VideoCapture(0)
print("video start")
# Create trackbars for the lower and upper bounds of the threshold
cv2.namedWindow('Threshold Controls')
cv2.createTrackbar('Lower Bound', 'Threshold Controls', 248, 255, lambda x: x)
cv2.createTrackbar('Upper Bound', 'Threshold Controls', 255, 255, lambda x: x)
# Initialize the ROS node and image publisher
# rospy.init_node('thresholded_image_publisher')
# image_pub = rospy.Publisher('/thresholded_image', Image, queue_size=1)
bridge = CvBridge()
while not rospy.is_shutdown():
    # Read a frame from the video feed
    ret, frame = cap.read()
    # Convert the frame to greyscale
    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Apply Gaussian blur to the greyscale image to reduce noise
    blur = cv2.GaussianBlur(grey, (5, 5), 0)
    # Get the lower and upper bounds of the threshold from the trackbars
    lower_bound = cv2.getTrackbarPos('Lower Bound', 'Threshold Controls')
    upper_bound = cv2.getTrackbarPos('Upper Bound', 'Threshold Controls')
    # Apply the threshold to the blurred image
    ret, threshold = cv2.threshold(
        blur, lower_bound, upper_bound, cv2.THRESH_BINARY)
    # Convert the thresholded image to a ROS image message
    x = list(threshold/255)
    # print(x)
    # x = [int(i) for i in x]
    # try:
    #     image_msg = bridge.cv2_to_imgmsg(x, "mono8")
    # except CvBridgeError as e:
    #     print(e)
    # occupancy_matrix.data = image_msg
    occupancy_matrix.data = []
    # numpy_arr = np.array(x)
    for i in x:
        for j in i:
            occupancy_matrix.data.append(int(j))
    # Publish the image message
    print(occupancy_matrix.data)
    map_pub.publish(occupancy_matrix)
    # Display the thresholded image
    cv2.imshow('Thresholded Video', threshold)
    # Break the loop if the user presses 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
# Release the video capture and destroy the windows
cap.release()
cv2.destroyAllWindows()

rospy.spin()
