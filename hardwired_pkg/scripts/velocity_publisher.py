#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseWithCovarianceStamped # for initialpose
from geometry_msgs.msg import PoseStamped # for movebase_simple_goal
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

# global variables
# coordinates_of_animals = PoseArray()
current_position = Pose2D()
# intermediate_goals = Path()
vel = Twist()
# callback functions

def path_from_astar(msg:Path):
    global intermediate_goals
    intermediate_goals = msg
    # intermediate_goals.poses[i].pose.position.x
def arucomarker_position(msg:Pose2D):
    global current_position
    current_position = msg

def animals(msg:PoseArray):
    global coordinates_of_animals
    coordinates_of_animals = msg
    
def main():
    rospy.init_node('file_3',anonymous=True)
    rospy.Subscriber('aruco_marker',Pose2D,callback=arucomarker_position)
    # rospy.Subscriber('animals', PoseArray, animals)
    # rospy.Subscriber('nav_path', Path, callback= path_from_astar)
    # rospy.Subscriber('mask', Path, callback= map_from_astar)
    
    # initialpose_pub = rospy.Publisher('initialpose',PoseWithCovarianceStamped,queue_size=10)
    # move_base_goal_pub = rospy.Publisher('move_base_simple/goal',PoseStamped, queue_size=10)
    vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    
    # print(coordinates_of_animals)
    
    # for i in range(4):
        
    #     # publishing final pose
    #     finalpose = PoseStamped()
    #     finalpose.pose.position.x = coordinates_of_animals.poses[i].position.x
    #     finalpose.pose.position.y = coordinates_of_animals.poses[i].position.y
    #     finalpose.pose.position.z = 0.
    #     # finalpose.pose.orientation.x =
    #     # finalpose.pose.orientation.y = 
    #     # finalpose.pose.orientation.z = 
    #     # finalpose.pose.orientation.w = 
    #     move_base_goal_pub(finalpose)
        
    #     for j in len(intermediate_goals):
                        
    #         x = intermediate_goals.poses[i].pose.orientation.x
    #         y = intermediate_goals.poses[i].pose.orientation.y
    #         z = intermediate_goals.poses[i].pose.orientation.z
    #         w = intermediate_goals.poses[i].pose.orientation.w
    #         rpy_orientation = euler_from_quaternion(x,y,z,w)
            
    #         small_goal_x = intermediate_goals.poses[i].pose.position.x
    #         small_goal_y = intermediate_goals.poses[i].pose.position.y
    #         small_goal_orientation = rpy_orientation[2]
            
    #         error_x = small_goal_x - current_position.x
    #         error_y = small_goal_y - current_position.y
    #         error_theta = small_goal_orientation - current_position.theta
            
            
    goal = Pose2D()
    goal.x = current_position.x    
    goal.y = current_position.y
    goal.theta = 90
    

    while True :
                # # publishing initial pose
                # initialpose = PoseWithCovarianceStamped()
                # initialpose.pose.pose.position.x = current_position.x 
                # initialpose.pose.pose.position.y = current_position.y
                # initialpose.pose.pose.position.z = 0.
                # initialpose.pose.pose.orientation.x = 
                # initialpose.pose.pose.orientation.y = 
                # initialpose.pose.pose.orientation.z = 
                # initialpose.pose.pose.orientation.w = 
                
                # initialpose.pose.covariance = [0. for i in range(36)]
                # initialpose_pub(initialpose)
                
                error_x = goal.x - current_position.x
                error_y = goal.y - current_position.y
                error_theta = goal.theta - current_position.theta
                
                kp = 1
                
                if error_theta>0 and error_theta<180:
                    a =0*error_theta 
                    b=1*error_theta
                elif error_theta<360 and error_theta>180:
                    a=1*error_theta
                    b=0*error_theta
                else:
                    a=0*error_theta
                    b=0*error_theta
                
                v_x = kp * error_x + a + b
                v_y = kp * error_y  +a + b
                
                vel.linear.x = v_x
                vel.linear.y = v_y
                vel.angular.x = a
                vel.angular.y = b
                
                
                vel_pub.publish(vel)
            
    rospy.spin()
  
if __name__ == '__main__':
    main()