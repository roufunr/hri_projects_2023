#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from time import time

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(0.25) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world " + str(time())
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
