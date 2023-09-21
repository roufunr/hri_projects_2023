#!/usr/bin/env python

import rospy
import tf2_ros
import tf_conversions
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker

def leg_detector_callback(data):
    
    transform = TransformStamped()
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = "odom"
    transform.child_frame_id = "leg"
    transform.transform.translation.x = data.pose.position.x
    transform.transform.translation.y = data.pose.position.x
    transform.transform.translation.z = 0.0  # Assuming legs are on the ground

    
    transform.transform.rotation.x = data.pose.orientation.x
    transform.transform.rotation.y = data.pose.orientation.y
    transform.transform.rotation.z = data.pose.orientation.z
    transform.transform.rotation.w = 1.0

    
    tf_broadcaster.sendTransform(transform)
    #rospy.loginfo(transform)

if __name__ == '__main__':
    rospy.init_node('leg_tf_broadcaster')
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    leg_detector_topic = "/visualization_marker"  # Replace with your leg detector topic
    rospy.Subscriber(leg_detector_topic, Marker, leg_detector_callback)

    rospy.spin()


