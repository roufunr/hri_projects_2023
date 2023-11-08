#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def laser_callback(laser_data):
    
    min_distance = min(laser_data.ranges)
    
    if min_distance < 0.5: 
        twist_msg.linear.x = 0.0
    else:
        twist_msg.linear.x = 0.5
    
    if min_distance < 1.0:  
        left_distances = sum(laser_data.ranges[:len(laser_data.ranges)//2])
        right_distances = sum(laser_data.ranges[len(laser_data.ranges)//2:])
        
        if left_distances < right_distances:
            twist_msg.angular.z = 0.5  # Turn right
        else:
            twist_msg.angular.z = -0.5  # Turn left
    else:
        twist_msg.angular.z = 0.0  # No turn

    cmd_vel_pub.publish(twist_msg)


rospy.init_node('obstacle_avoidance_node')
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
twist_msg = Twist()

# run node
rospy.spin()
