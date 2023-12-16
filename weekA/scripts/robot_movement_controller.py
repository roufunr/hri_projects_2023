#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Bool

TOTAL_STEPS = 20

states = {
    "init": {"HeadYaw": 0,"HeadPitch": 0},
    "final": {"HeadYaw": -0.26,"HeadPitch": 0.36},
}



class RobotController:
    def __init__(self):
        rospy.init_node('robot_movement', anonymous=True)
        self.stt_sub_node = "/speech_recognition/final_result"
        self.joint_pub_node = "joint_states"
        
        self.speech_sub = rospy.Subscriber(self.stt_sub_node, String, self.speech_callback)
        self.joint_pub = rospy.Publisher(self.joint_pub_node, JointState, queue_size=10)

    def interpolate_poses(self, fraction):
        interpolated_pose = {}
        for joint in states["init"]:
            interpolated_pose[joint] = states["init"][joint] + fraction * (states["final"][joint] - states["init"][joint])
        return interpolated_pose
    
    def interpolate_pose_reverse(self, fraction):
        interpolated_pose = {}
        for joint in states["init"]:
            interpolated_pose[joint] = states["final"][joint] + fraction * (states["init"][joint] - states["final"][joint])
        return interpolated_pose

    def get_ith_state(self, i):
        ith_state = {}
        for key in states["init"]:
            difference = states["final"][key] - states["init"][key]
            ith_state[key] = states["init"][key] + ((difference) * (i/TOTAL_STEPS))
        return ith_state
        
    def get_a_joint_state(self, state):
        joint_states = JointState()
        joint_states.header.stamp = rospy.Time.now()
        # joint_states.header.frame_id = "Torso"
        for key in state:
            joint_states.name.append(key)
            joint_states.position.append(state[key])
        return joint_states

    def looker(self):
        rate = rospy.Rate(10)
        for i in range(TOTAL_STEPS):
            state = self.interpolate_poses(i/TOTAL_STEPS)
            joint_states = self.get_a_joint_state(state)
            self.joint_pub.publish(joint_states)
            rate.sleep()

    def get_ready(self):
        rate = rospy.Rate(10)
        for i in range(TOTAL_STEPS):
            state = self.interpolate_pose_reverse(i/TOTAL_STEPS)
            joint_states = self.get_a_joint_state(state)
            self.joint_pub.publish(joint_states)
            rate.sleep() 

    def speech_callback(self, data):
        rospy.loginfo("Robot heard %s", data.data)
        if "look at your hand" in data.data:
            self.looker()
        if "ready" in data.data:
            self.get_ready()
        

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
       robot_controller = RobotController()
       robot_controller.run()
    except rospy.ROSInterruptException:
        pass


