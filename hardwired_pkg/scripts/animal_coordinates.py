#!/usr/bin/env python3

import time
import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
animal_coordinates = PoseArray()
giraffe = Pose() 
barre = Pose()

def node():
    global animal_coordinates, giraffe, barre
    rospy.init_node('file1',anonymous=False)
    pointsarray_pub = rospy.Publisher('animals',PoseArray,queue_size=10)
    
    while not rospy.is_shutdown():
        animal_coordinates.header.stamp = rospy.Time.now()
        animal_coordinates.header.frame_id = "animal_coordinates"

        barre.position.x=5    
        barre.position.y=5

        giraffe.position.x = 10
        giraffe.position.y = 20
        
        animal_coordinates.poses.append(barre)
        animal_coordinates.poses.append(giraffe)

        pointsarray_pub.publish(animal_coordinates)
        rospy.loginfo(animal_coordinates)
  
if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass