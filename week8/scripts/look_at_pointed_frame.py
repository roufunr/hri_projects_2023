#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
import math
from sensor_msgs.msg import JointState

TOTAL_STEPS = 10

def get_ith_state(i, states):
    ith_state = {}
    for key in states["init"]:
        difference = states["final"][key] - states["init"][key]
        ith_state[key] = states["init"][key] + ((difference) * (i/TOTAL_STEPS))
    return ith_state
    
def get_a_joint_state(state):
    joint_states = JointState()
    joint_states.header.stamp = rospy.Time.now()
    # joint_states.header.frame_id = "Torso"
    for key in state:
        joint_states.name.append(key)
        joint_states.position.append(state[key])
    return joint_states

def looker(rate, del_x, del_y, del_z, pub):
    distance = math.sqrt(del_x**2 + del_y**2 + del_z**2)
    yaw = math.atan2(del_y, del_x)
    pitch = (-1) * math.asin(del_z / distance)

    states = {
        "init": {"HeadYaw": 0,"HeadPitch": 0},
        "final": {"HeadYaw": yaw,"HeadPitch": pitch},
    }

    for i in range(TOTAL_STEPS):
        state = get_ith_state(i, states)
        #rospy.loginfo(state)
        joint_states = get_a_joint_state(state)
        #rospy.loginfo(joint_states)
        pub.publish(joint_states)
        rate.sleep()
def look_at_the_point(pub):
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        try: 
            head_frame = tf_buffer.lookup_transform("base_link", "Head", rospy.Time())
            pointed_frame = tf_buffer.lookup_transform("base_link", "pointed_frame", rospy.Time())
            

            # Calculate the differences in translation
            del_x = pointed_frame.transform.translation.x - head_frame.transform.translation.x
            del_y = pointed_frame.transform.translation.y - head_frame.transform.translation.y
            del_z = pointed_frame.transform.translation.z - head_frame.transform.translation.z
            looker(rate = rate, del_x = del_x, del_y = del_y, del_z = del_z, pub = pub)
        
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue
        

if __name__ == '__main__':
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.init_node('look_at_the_point')
    look_at_the_point(pub)
    
