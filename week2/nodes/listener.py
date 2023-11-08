#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
import math
from sensor_msgs.msg import LaserScan

def jump(): 
    msg = Twist()
    msg.angular.z = 0
    msg.linear.x = 1.0
    cmd_vel_publisher.publish(msg)

def laser_callback(data):
    global laser_data 
    laser_data = data

def avoid_obstacle(msg):
    if (msg.linear.x <= 0.01 and abs(msg.angular.z) <= 0.01) : # arrive to people 
        msg.linear.x = 0
        msg.angular.z = 0
    else: # walk around
        min_distance = min(laser_data.ranges)
        if min_distance < 0.5: 
            msg.linear.x = 0.0
        elif msg.linear.x + 0.5 < min_distance:
            msg.linear.x += 0.5
        
        if min_distance < 1.0:  
            left_distances = sum(laser_data.ranges[:len(laser_data.ranges)//2])
            right_distances = sum(laser_data.ranges[len(laser_data.ranges)//2:])
            
            if left_distances < right_distances:
                msg.angular.z = 0.5  # Turn right
            else:
                msg.angular.z = -0.5  # Turn left
      
    cmd_vel_publisher.publish(msg)

def robot_follow_person():
    jump()
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rate = rospy.Rate(10)  # 100 Hz
    while not rospy.is_shutdown():
        try:
            try:
                trans = tf_buffer.lookup_transform('robot', 'person', rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()
                continue
            msg = Twist()
            # rospy.loginfo("\n" + str(trans.transform.translation))
            msg.angular.z = 1.0 * math.atan2(trans.transform.translation.y, trans.transform.translation.x)
            msg.linear.x = 0.2 * math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)
            # rospy.loginfo("\n" + str(msg.angular.z * 180) + "\n" + str(msg.linear.x ))
            avoid_obstacle(msg)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("TF lookup exception")
            rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('robot_follow_person')
        cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        laser_sub = rospy.Subscriber('/base_scan', LaserScan, laser_callback)
        robot_follow_person()
    except rospy.ROSInterruptException:
        pass
