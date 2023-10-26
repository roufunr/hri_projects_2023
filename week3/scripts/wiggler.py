#!/usr/bin/python
# license removed for brevity

import rospy
import random
from geometry_msgs.msg import Twist

def wiggler():
    pub1 = rospy.Publisher('/robot_1/cmd_vel', Twist, queue_size=10)
    pub2 = rospy.Publisher('/robot_2/cmd_vel', Twist, queue_size=10)
    pub3 = rospy.Publisher('/robot_3/cmd_vel', Twist, queue_size=10)
    rospy.init_node('wiggler', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    random.seed()
    while not rospy.is_shutdown():
        # generate random x and y values
        xvel = random.gauss(0, 0.03)
        yvel = random.gauss(0, 0.02)
        turn = 0 #random.gauss(0, 0.2)
        
        cmd_vel = Twist()
        cmd_vel.linear.x = xvel
        cmd_vel.linear.y = yvel
        cmd_vel.angular.z = turn

        pub1.publish(cmd_vel)
        pub2.publish(cmd_vel)
        pub3.publish(cmd_vel)
        rate.sleep()

if __name__ == '__main__':
    try:
        wiggler()
    except rospy.ROSInterruptException:
        pass