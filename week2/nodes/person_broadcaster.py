#!/usr/bin/env python

import rospy
import tf2_ros
import tf_conversions
from geometry_msgs.msg import TransformStamped
from people_msgs.msg import PositionMeasurementArray

def person_detector_callback(data):
    
    for each in data.people:
        if each.header.frame_id == "odom": 
            transform = TransformStamped()
            transform.header.stamp = rospy.Time.now()
            transform.header.frame_id = "odom"  # Change this to match the frame of your world/map
            transform.child_frame_id = "person"
            transform.transform.translation.x = each.pos.x
            transform.transform.translation.y = each.pos.y
            transform.transform.translation.z = 0.0  # Assuming legs are on the ground
            q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
            transform.transform.rotation.x = q[0]
            transform.transform.rotation.y = q[1]
            transform.transform.rotation.z = q[2]
            transform.transform.rotation.w = q[3]
            tf_broadcaster.sendTransform(transform)
            # rospy.loginfo(transform)

if __name__ == '__main__':
    rospy.init_node('person_tf_broadcaster')
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    person_detector_topic = "/people_tracker_measurements"  # Replace with your leg detector topic
    rospy.Subscriber(person_detector_topic, PositionMeasurementArray, person_detector_callback)

    rospy.spin()
