#!/usr/bin/env python
import rospy
import time
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math
import numpy as np

pose = ''

def Waypoints(t):
    x  = t
    y  = 2*(math.sin(x))*(math.sin(x/2))
    return [x,y]

def control_loop():
    rospy.init_node('ebot_controller')
    
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    
    rate = rospy.Rate(10) 

    velocity_msg = Twist()
    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    pub.publish(velocity_msg)

    x_target = np.arange(0.1,2* math.pi,0.5)
    y_target = [ j[1] for j in [ Waypoints(i) for i in x_target] ]
    count = 0
    count_f = 0
    obstacle_avoidance_on = False
    object_detected = False
    final_path = False
    final_path_points_x = [11,12.5]
    final_path_points_y = [2,0]
    while not rospy.is_shutdown():
    	
    	#
    	# Your algorithm to complete the obstacle course
    	#
        rate.sleep()
        global regions, pose
        

        # Traversing in path 2*(math.sin(x))*(math.sin(x/2))
        if(count < len(x_target)):
            x_vel = 0.4
            theta_target = math.atan( (y_target[count]-pose[1])/(x_target[count]-pose[0]) )
            theta_vel = 2.5 * (theta_target - pose[2])

            if( (-(pose[0] - x_target[count]) < 0.05)  ):
                count+=1
        
        if(count >= len(x_target)): # When bot reaches first goal
            obstacle_avoidance_on = True
            theta_vel = 0

        #After bot reaches first goal, Obstacle detection algorithm starts
        if(obstacle_avoidance_on == True):  

            # Checking for object in front
            if( (regions['fright']<=2 ) and (regions['fleft']<=2 ) and (regions['front']<=2 ) and (object_detected == False) ):
                x_vel = 0
                object_detected =True

            # If object detected , rotating bot 90 degrees to left and move away from obstacle
            if(object_detected == True):
                if(regions['front'] < 3):
                    theta_vel = 2
                else:
                    if(regions['bright'] < 2):
                        x_vel = 0.4
                        theta_vel = 0
                    else:
                        obstacle_avoidance_on = False
                        final_path = True
        
        # Start final journey towards x = 12.5
        if(final_path == True):
            if(count_f < len(final_path_points_x)):
                x_vel = 0.4
                theta_target = math.atan( (final_path_points_y[count_f]-pose[1])/(final_path_points_x[count_f]-pose[0]) )
                theta_vel = 2.5 * (theta_target - pose[2])

                if( (-(pose[0] - final_path_points_x[count_f]) < 0.05)  ):
                    count_f+=1
        
            if(count_f >= len(final_path_points_x)): # When bot reaches first goal
                x_vel = 0
                theta_vel = 0

    	velocity_msg.linear.x = x_vel
        velocity_msg.angular.z = theta_vel

    	pub.publish(velocity_msg)
    	print("Controller message pushed at {}".format(rospy.get_time()))


def odom_callback(data):
    global pose
    x  = data.pose.pose.orientation.x
    y  = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x,y,z,w])[2]]


def laser_callback(msg):
    global regions
    regions = {
        'bright':  min(min(msg.ranges[0:143]),10)	,
        'fright':  min(min(msg.ranges[144:287]),10)	,
        'front':   min(min(msg.ranges[288:431]),10)	,
        'fleft':   min(min(msg.ranges[432:575]),10)	,
        'bleft':   min(min(msg.ranges[576:713]),10)	,
    }
if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass


