#!/usr/bin/env python3

import math
import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Pose2D

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
    rospy.init_node('aruco_marker_detector')
    aruco_pub = rospy.Publisher('aruco_marker', Pose2D, queue_size=10)
    aruco_msg = Pose2D()
    aruco_list = {}
    # Set up the Aruco detector
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)
    detector = cv2.aruco.DetectorParameters_create()

    # Set up the camera capture
    cap = cv2.VideoCapture(0)

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        if not ret:
            print("Error capturing frame")
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
        
        # if ids is not None:
        #     for i in range(len(ids)):
        #         # Draw marker
        #         cv2.aruco.drawDetectedMarkers(frame, corners)

        #         # Calculate marker pose
        #         rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, np.eye(3), np.zeros((3, 1)))

        #         # Draw coordinate frame and display position and orientation
        #         for j in range(len(ids)):
        #             cv2.aruco.drawAxis(frame, np.eye(3), np.zeros((3, 1)), rvec[j], tvec[j], 0.1)
        #             cv2.putText(frame, f'Position: {tvec[j]}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        #             cv2.putText(frame, f'Orientation: {rvec[j]}', (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        
        # Display the resulting frame
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Clean up
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()