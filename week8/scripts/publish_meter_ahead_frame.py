#!/usr/bin/env python3

import rospy
import tf2_ros
import tf_conversions
from geometry_msgs.msg import TransformStamped

def broadcast_frame(tf_broadcaster):
    transform = TransformStamped()
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = "RFinger22_link"  
    transform.child_frame_id = "pointed_frame"
    transform.transform.translation.x = 1
    transform.transform.rotation.w = 1
    tf_broadcaster.sendTransform(transform)     

if __name__ == '__main__':
    rospy.init_node('new_frame_publisher')
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        broadcast_frame(tf_broadcaster)
        rate.sleep()
