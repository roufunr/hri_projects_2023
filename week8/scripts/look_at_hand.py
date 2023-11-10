#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
import copy

TOTAL_STEPS = 20

states = {
    "init": {"HeadYaw": 0,"HeadPitch": 0},
    "final": {"HeadYaw": -0.26,"HeadPitch": 0.36},
}

def interpolate_poses(fraction):
    interpolated_pose = {}
    for joint in states["init"]:
        interpolated_pose[joint] = states["init"][joint] + fraction * (states["final"][joint] - states["init"][joint])
    return interpolated_pose

def get_ith_state(i):
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

def looker():
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.init_node('looker', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        for i in range(TOTAL_STEPS):
            state = interpolate_poses(i/TOTAL_STEPS)
            #rospy.loginfo(state)
            joint_states = get_a_joint_state(state)
            #rospy.loginfo(joint_states)
            pub.publish(joint_states)
            rate.sleep()
if __name__ == '__main__':
    try:
        looker()
    except rospy.ROSInterruptException:
        pass
