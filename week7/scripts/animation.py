#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
import time
import numpy as np

def send_joint_state(joint_names, positions):
    joint_state_msg = JointState()
    joint_state_msg.header.stamp = rospy.Time.now()
    joint_state_msg.name = joint_names
    joint_state_msg.position = positions

    pub.publish(joint_state_msg)

def interpolate_states(initial_state, final_state, num_steps):
    interpolated_states = []
    for i in range(num_steps):
        alpha = float(i) / float(num_steps - 1)
        interpolated_state = [
            initial + alpha * (final - initial) for initial, final in zip(initial_state, final_state)
        ]
        interpolated_states.append(interpolated_state)
    
    return interpolated_states

def animate_keyframes(initial_state, final_state, num_steps):
    # Define joint names
    joint_names = ["HeadYaw", "HeadPitch", "LHipYawPitch", "LHipRoll", "LHipPitch", "LKneePitch",
                   "LAnklePitch", "LAnkleRoll", "RHipYawPitch", "RHipRoll", "RHipPitch", "RKneePitch",
                   "RAnklePitch", "RAnkleRoll", "LShoulderPitch", "LShoulderRoll", "LElbowYaw",
                   "LElbowRoll", "LWristYaw", "LHand", "RShoulderPitch", "RShoulderRoll", "RElbowYaw",
                   "RElbowRoll", "RWristYaw", "RHand", "RFinger23", "RFinger13", "RFinger12", "LFinger21",
                   "LFinger13", "LFinger11", "RFinger22", "LFinger22", "RFinger21", "LFinger12", "RFinger11",
                   "LFinger23", "LThumb1", "RThumb1", "RThumb2", "LThumb2"]

    # Interpolate between initial and final states
    interpolated_states = interpolate_states(initial_state, final_state, num_steps)

    # Create a ROS node
    rospy.init_node('joint_state_publisher', anonymous=True)

    # Create a publisher for the JointState messages
    global pub
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

    # Wait for the publisher to be ready
    rospy.sleep(1)

    # Publish each interpolated state with a delay
    for interpolated_state in interpolated_states:
        send_joint_state(joint_names, interpolated_state)
        rospy.sleep(2)  # Adjust the delay as needed

    rospy.loginfo("Keyframe animation completed.")

if __name__ == '__main__':
    try:
        # Define the number of steps for interpolation
        num_steps = 10  # You can adjust this value

        # Initial and final states
        initial_state = [0.0, -9.093359999989836e-05, 0.004986279199999943, -3.8051500000024774e-05, -0.00018283900000004571,
                         -0.00016400378000000493, -0.00016097489999999937, -1.490230000000814e-05, 0.004986279199999943,
                         -7.893800000002837e-05, -0.00018283900000004571, -0.00016400378000000493, -4.8639999999711137e-05,
                         -0.0001017730000000272, 0.0, -0.00014643740000003236, 0.0, -0.7897633000000001, 0.0, 0.0, 0.0,
                         -1.7623500000008008e-05, 0.0, 0.7897633000000001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        final_state = [2.009334478, 0.18173050320000017, -0.20737822159999997, -0.007174410999999992, -1.308654625,
                       1.53531296578, 0.6106354982999997, 0.3858303147999999, -0.20737822159999997, 0.15387924400000008,
                       -0.6780512109999999, 0.044595017589999994, -0.3275387475999999, -0.5176734037999999, -0.7908860639999999,
                       0.5991680303, -0.2523660699999999, -0.6582672628600001, -0.07514344400000006, 0.2254, 0.6920253060000001,
                       -0.2090312100999998, -1.14503283, 1.4559998234200002, -0.16159488199999994, 0.5829, 0.5828411270999999,
                       0.5828411270999999, 0.5828411270999999, 0.22537723459999998, 0.22537723459999998, 0.22537723459999998,
                       0.5828411270999999, 0.22537723459999998, 0.5828411270999999, 0.22537723459999998, 0.5828411270999999,
                       0.22537723459999998, 0.22537723459999998, 0.5828411270999999, 0.5828411270999999, 0.22537723459999998]

        animate_keyframes(initial_state, final_state, num_steps)
    except rospy.ROSInterruptException:
        pass
