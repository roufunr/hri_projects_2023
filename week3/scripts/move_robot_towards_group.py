#!/usr/bin/python3

import rospy
import copy
import math
from people_msgs.msg import PositionMeasurementArray
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

def base_scan_callback(data):
    print("Base Scan Data Here")

def detected_groups_callback(data):
    print("Detected Group Data Here")

def publish_robot_moving_data(publisher):
    twist_data = Twist()
    # insert implementation here
    publisher.publish(twist_data)

def main():
    rospy.init_node('robot_mover', anonymous=True)
    robot_moving_data_publisher = rospy.Publisher("/robot_0/cmd_vel", Twist, queue_size = 20)
    rospy.Subscriber("/robot_0/base_scan", LaserScan, base_scan_callback)
    rospy.Subscriber("/robot_0/detected_groups", PositionMeasurementArray, detected_groups_callback)
    publish_robot_moving_data(robot_moving_data_publisher)

if __name__=="__main__":
    main()