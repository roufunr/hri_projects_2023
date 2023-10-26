#!/usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

def robot_movement_callback(data): 
    transform = TransformStamped()
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = "odom"
    transform.child_frame_id = "robot"
    transform.transform.translation.x = data.pose.pose.position.x
    transform.transform.translation.y = data.pose.pose.position.y
    transform.transform.translation.z = 0.0  
    transform.transform.rotation.x = data.pose.pose.orientation.x
    transform.transform.rotation.y = data.pose.pose.orientation.y
    transform.transform.rotation.z = data.pose.pose.orientation.z
    transform.transform.rotation.w = data.pose.pose.orientation.w
    tf_broadcaster.sendTransform(transform)
  
        
if __name__ == '__main__':
    rospy.init_node('robot_tf_broadcaster')
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    robot_topic = "/robot_0/base_pose_ground_truth"  
    rospy.Subscriber(robot_topic, Odometry, robot_movement_callback)
    rospy.spin()


