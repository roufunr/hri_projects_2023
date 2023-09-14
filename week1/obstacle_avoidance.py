#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def laser_callback(laser_data):
    # Check if there are any obstacles within a certain distance
    min_distance = min(laser_data.ranges)
    
    if min_distance < 0.5:  # Adjust the threshold distance as needed
        # Goal 3: Stop the robot if an obstacle is close
        twist_msg.linear.x = 0.0
    else:
        # Goal 3: Move forward if no obstacles are too close
        twist_msg.linear.x = 0.5  # You can adjust the linear velocity
    
    # Goal 4: If there's an obstacle in front, turn to the side with fewer obstacles
    if min_distance < 1.0:  # Adjust the threshold distance as needed
        # Calculate the average distances on the left and right sides
        left_distances = sum(laser_data.ranges[:len(laser_data.ranges)//2])
        right_distances = sum(laser_data.ranges[len(laser_data.ranges)//2:])
        
        if left_distances < right_distances:
            twist_msg.angular.z = 0.5  # Turn right
        else:
            twist_msg.angular.z = -0.5  # Turn left
    else:
        twist_msg.angular.z = 0.0  # No need to turn

    # Publish the cmd_vel message
    cmd_vel_pub.publish(twist_msg)


# Initialize ROS node
rospy.init_node('obstacle_avoidance_node')
laser_sub = rospy.Subscriber('/base_scan', LaserScan, laser_callback)
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
twist_msg = Twist()


# Run the node
rospy.spin()
