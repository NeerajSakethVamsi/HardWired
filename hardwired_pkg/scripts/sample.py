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
from geometry_msgs.msg import Pose2D
import math

rospy.init_node('map_publisher',anonymous=True)
map_pub = rospy.Publisher('map',OccupancyGrid,queue_size=1)
aruco_pub = rospy.Publisher('aruco_marker', Pose2D, queue_size=10)
aruco_msg = Pose2D()
aruco_list = {}
occupancy_matrix = OccupancyGrid()

def angle_calculate(pt1,pt2, trigger = 0):  # function which returns angle between two points in the range of 0-359
    angle_list_1 = list(range(359,0,-1))
    #angle_list_1 = angle_list_1[90:] + angle_list_1[:90]
    angle_list_2 = list(range(359,0,-1))
    angle_list_2 = angle_list_2[-90:] + angle_list_2[:-90]
    x=pt2[0]-pt1[0] # unpacking tuple
    y=pt2[1]-pt1[1]
    angle=int(math.degrees(math.atan2(y,x))) #takes 2 points nad give angle with respect to horizontal axis in range(-180,180)
    if trigger == 0:
        angle = angle_list_2[angle]
    else:
        angle = angle_list_1[angle]
    return int(angle)

def calculate_Robot_State(img,aruco_list):  #gives the state of the bot (centre(x), centre(y), angle)
    robot_state = {}
    key_list = aruco_list.keys()
    font = cv2.FONT_HERSHEY_SIMPLEX

    robot_state_list =[]
    for key in key_list:
        dict_entry = aruco_list[key]
        pt1 , pt2 = tuple(dict_entry[0]) , tuple(dict_entry[1])
        centre = dict_entry[0] + dict_entry[1] + dict_entry[2] + dict_entry[3]
        centre[:] = [int(x / 4) for x in centre]
        centre = tuple(centre)
        #print centre
        angle = angle_calculate(pt1, pt2)
        cv2.putText(img, str(angle), (int(centre[0] - 80), int(centre[1])), font, 1, (0,0,255), 2, 
cv2.LINE_AA)
        robot_state[key] = (int(centre[0]), int(centre[1]), angle)#HOWEVER IF YOU ARE SCALING IMAGE AND ALL...THEN BETTER INVERT X AND Y...COZ THEN ONLY THE RATIO BECOMES SAME
    #print (robot_state)
        robot_state_list = [centre[0],centre[1],angle]
        print(robot_state_list)

    return robot_state_list

def main():
    # Capture the video feed
    cap = cv2.VideoCapture(2)
    print("video start")
    # Set up the Aruco detector
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)
    detector = cv2.aruco.DetectorParameters_create()
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
        
        
        # Detect Aruco markers
        corners, ids, _ = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=detector)
        if ids is not None:
            # Draw the markers on the frame
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        if len(corners):    #returns no of arucos
        #print (len(corners))
        #print (len(ids))
            print (type(corners))
            print (corners[0][0])
            for k in range(len(corners)):
                temp_1 = corners[k]
                temp_1 = temp_1[0]
                temp_2 = ids[k]
                temp_2 = temp_2[0]
                aruco_list[temp_2] = temp_1
                
        robot_pose = calculate_Robot_State(frame,aruco_list)
        if len(robot_pose) != 0:
            print(robot_pose)
            aruco_msg.x = robot_pose[0]
            aruco_msg.y = robot_pose[1]
            aruco_msg.theta = robot_pose[2]
        
        # aruco_pub(aruco_msg)
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
    # Release the video capture and destroy the windows
    cap.release()
    cv2.destroyAllWindows()

    rospy.spin()

if __name__ == '__main__':
    main()