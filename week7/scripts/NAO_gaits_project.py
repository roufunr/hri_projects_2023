#!/usr/bin/env python3
import rospy
import pandas as pd
from sensor_msgs.msg import JointState
import time

def publish_joint_states(csv_file_path):
    # Initialize the ROS node
    rospy.init_node('nao_joint_state_publisher', anonymous=False)
    # Create a publisher for the 'joint_states' topic
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    # Read the CSV file
    df = pd.read_csv(csv_file_path)
    # Define the rate (in Hz) at which to publish the messages
    rate = rospy.Rate(1/0.02) # 12 ms
    # Infinite loop to continuously animate the robot
    while not rospy.is_shutdown():
        # Iterate over each row in the DataFrame
        for index, row in df.iterrows():
            # Check if ROS is shutting down
            if rospy.is_shutdown():
                break
            # Create a JointState message
            joint_state = JointState()
            joint_state.header.stamp = rospy.Time.now()
            # Assuming the joint names are known and correspond to the columns
            joint_names = ['lHipRoll', 'lHipPitch', 'lKneePitch', 'lAnklePitch', 'lAnkleRoll', 
                            'rHipRoll', 'rHipPitch', 'rKneePitch', 'rAnklePitch', 'rAnkleRoll']
            # Fill in the joint names and positions
            joint_state.name = ['LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll',  
                                'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll', 
                                'LHipYawPitch', 'RHipYawPitch']
            joint_state.position = [row[f'Request.angles.{name}'] for name in joint_names]
            joint_state.position.append(row[f'Request.angles.hipYawPitch'])
            joint_state.position.append(row[f'Request.angles.hipYawPitch'])
            rospy.loginfo(joint_state.position)
            # Publish the joint state
            pub.publish(joint_state)
            # Sleep to maintain the loop rate
            rate.sleep()

if __name__ == '__main__':
    try:
        csv_file_path = '/home/rouf20/Documents/2022GORE/2022-04-15_FieldB_RoboEireann_B-Human_Sarah.csv'
        publish_joint_states(csv_file_path)
    except rospy.ROSInterruptException:
        pass