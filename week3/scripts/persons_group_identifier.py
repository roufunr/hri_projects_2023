#!/usr/bin/python3

import rospy
import copy
import math
from people_msgs.msg import PositionMeasurementArray


current_position_data = None
two_persons_distance_threashold = 3


def people_tracker_measurements_callback(data):
    global current_position_data
    current_position_data = copy.deepcopy(data)


def get_distance(person1, person2):
    x1 = person1.pos.x
    y1 = person1.pos.y
    x2 = person2.pos.x
    y2 = person2.pos.y
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def get_all_groups(data):
    groups = []
    persons = data.people
    total_persons = len(persons)
    for i in range(1, total_persons):
        group = [persons[i]]
        for j in range(i):
            distance_from_j_to_i = get_distance(persons[j], persons[i])
            if distance_from_j_to_i <= two_persons_distance_threashold:
                group.append(persons[j])
        groups.append(group)
    return groups

def publish_persons_groups(publisher):
    while not rospy.is_shutdown():
        current_data = copy.deepcopy(current_position_data)
        if current_data == None:
            continue
        elif current_data != None and len(current_data.people) == 0:
            continue
        else:
            groups = get_all_groups(current_data)


    #publisher.publish()

def main():
    rospy.init_node('persons_group_detector', anonymous=True)
    groups_publisher = rospy.Publisher("/robot_0/detected_groups", PositionMeasurementArray, queue_size=10)
    persons_position_subscriber = rospy.Subscriber("/people_tracker_measurements", PositionMeasurementArray, people_tracker_measurements_callback)
    publish_persons_groups(groups_publisher)

if __name__=="__main__":
    main()