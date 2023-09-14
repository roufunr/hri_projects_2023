#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

from time import time

def talker():
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(0.5) # frequency
    while not rospy.is_shutdown():
        twist_msg = Twist()
        twist_msg.linear = Vector3(x = 5, y = 0, z = 0)
        twist_msg.angular = Vector3(x = 0.0, y = 0.0, z = 0.1)
        rospy.loginfo(twist_msg)
        pub.publish(twist_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
