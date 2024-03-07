#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
import pandas as pd

rq_data = []
sn_data = []

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

def load_data():
    global rq_data
    global sn_data
    df = pd.read_csv('/home/rouf20/Documents/2022RoboCup/2022-07-12_FieldA_B-Human_Bembelbots_7vs7_DerPinguin.csv')
    req_data = []
    sens_data = []
    print(df.shape[0])
    for i in range(int(df.shape[0])):
        requests = df.iloc[i][1:12].tolist()
        requests.pop(5)
        req_data.append(requests)
        sensors = df.iloc[i][12:].tolist()
        sensors.pop(5)
        sens_data.append(sensors)
    rq_data = req_data
    sn_data = sens_data


def animate_keyframes(initial_state, final_state, num_steps):
    joint_names = ['LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll']
    interpolated_states = interpolate_states(initial_state, final_state, num_steps)
    global pub
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    rospy.sleep(1)
    for interpolated_state in interpolated_states:
        send_joint_state(joint_names, interpolated_state)
        rospy.sleep(0.1)
    rospy.loginfo("Keyframe animation completed.")

if __name__ == '__main__':
    try:
        rospy.init_node('animator', anonymous=True)
        load_data()
        num_steps = 10  # You can adjust this value
        initial_state = [-3.8051500000024774e-05, -0.00018283900000004571, -0.00016400378000000493, -0.00016097489999999937, -1.490230000000814e-05, -7.893800000002837e-05, -0.00018283900000004571, -0.00016400378000000493, -4.8639999999711137e-05, -0.0001017730000000272]
        final_state = [-0.0017391672590747, -0.3634498715400696, 0.8295252919197083, -0.4792930781841278, -0.0006708744913339, 0.0017391672590747, -0.3631020486354828, 0.831960141658783, -0.4785974025726318, -0.0006710228626616]
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            animate_keyframes(initial_state, final_state, num_steps)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
