#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
import math

def robot_follow_leg():
    rospy.init_node('robot_follow_leg')
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        try:
            # Look up the transform between the "robot" and "leg" frames
            trans = tf_buffer.lookup_transform('robot', 'leg', rospy.Time(0))
            
            msg = Twist()

            msg.angular.z =  4 * math.atan2(trans.transform.translation.y, trans.transform.translation.x)
            msg.linear.x =   0.5 * math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)

            # Publish the Twist message to control the robot
            cmd_vel_publisher.publish(msg)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            # Handle TF lookup exceptions
            rospy.logwarn("TF lookup exception")

        rate.sleep()

if __name__ == '__main__':
    try:
        robot_follow_leg()
    except rospy.ROSInterruptException:
        pass
