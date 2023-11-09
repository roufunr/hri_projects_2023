#!/usr/bin/python3

import rospy
import copy
import math
from people_msgs.msg import PositionMeasurementArray

current_position_data = None                            # current position of all detected persons data
two_persons_distance_threashold = 3                     # constant and preddefined 
slope_tolerance = 0.1                                   # constant and preddefined (approx 5 degree)
radius_tolerance = 5                                    # percentage of tolerance

def people_tracker_measurements_callback(data):
    global current_position_data
    current_position_data = copy.deepcopy(data)

def calculate_slope(x1, y1, x2, y2):
    return (y2-y1) / (x2 - x1)

def get_slope(person1, person2):
    x1 = person1.pos.x
    y1 = person1.pos.y
    x2 = person2.pos.x
    y2 = person2.pos.y
    return calculate_slope(x1, y1, x2, y2)

def calculate_midpoint(x1, y1, x2, y2):
    return [(x1 + x2) / 2, (y1 + y2) / 2]

def get_midpoint(person1, person2):
    x1 = person1.pos.x
    y1 = person1.pos.y
    x2 = person2.pos.x
    y2 = person2.pos.y
    return calculate_midpoint(x1, y1, x2, y2)

def calculate_distance(x1,y1,x2,y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def get_distance(person1, person2):
    x1 = person1.pos.x
    y1 = person1.pos.y
    x2 = person2.pos.x
    y2 = person2.pos.y
    return calculate_distance(x1, y1, x2, y2)

def get_all_persons_coordinate(group, data):
    coordinates = []
    for person_idx in group:
        x = data.people[person_idx].pos.x
        y = data.people[person_idx].pos.y
        coordinates.append([x, y])
    return coordinates

def get_max_distant_points(data, group):
    points = get_all_persons_coordinate(group, data)
    total_points = len(points)
    persons = data.people
    max_dist = -1
    max_dist_point_idx = []
    for i in range(total_points):
        for j in range(i + 1, total_points):
            distance_from_i_to_j = get_distance(persons[i], persons[j])
            if distance_from_i_to_j > max_dist:
                max_dist = distance_from_i_to_j
                max_dist_point_idx = []
                max_dist_point_idx.append(i)
                max_dist_point_idx.append(j)
    return max_dist_point_idx

def is_in_line(data, group):
    if len(group) <= 1:
        return False
    slopes = []
    persons = data.people
    for i in range(1, len(group)):
        slope = get_slope(persons[group[i-1]], persons[group[i]])
        slopes.append(slope)
    for i in range(1, len(slopes)):
        delta_slope = abs(slopes[i-1], slopes[i])
        if delta_slope > slope_tolerance:
            return False
    return True

def is_in_circle(data, group):
    if len(group) <= 2:
        return True
    max_distant_point_idx = get_max_distant_points(data, group) # return two points of diameter
    center = get_midpoint(data.people[max_distant_point_idx[0]], data.people[max_distant_point_idx[1]])
    radius = get_distance(data.people[max_distant_point_idx[0]], data.people[max_distant_point_idx[1]]) / 2
    points = get_all_persons_coordinate(data, group)
    total_persons_in_group = len(points)
    for i in range(total_persons_in_group):
        x1 = center[0]
        y1 = center[1]
        x2 = points[i][0]
        y2 = points[i][1]
        dist_from_center = calculate_distance(x1, y1, x2, y2)
        percentage_of_difference = (abs(dist_from_center - radius) / radius) * 100
        if percentage_of_difference > radius_tolerance:
            return False
    return True

def get_all_groups(data):
    groups = []
    persons = data.people
    total_persons = len(persons)
    for i in range(total_persons):
        group = [i]
        for j in range(i + 1, total_persons):
            distance_from_i_to_j = get_distance(persons[i], persons[j])
            if distance_from_i_to_j <= two_persons_distance_threashold:
                group.append(j)
        if len(group) >= 2:
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
            rospy.loginfo(rospy.get_caller_id() + " :::: total detected groups: %d", len(groups))
            for group_idx in range(len(groups)):
                group = groups[group_idx]
                group_data = copy.deepcopy(current_data)
                group_data.people = []
                if is_in_line(current_data, group):
                    for person_idx in group:
                        person_data = copy.deepcopy(current_data.people[person_idx])
                        person_data.name = "line_" + str(group_idx) + "_person_" + person_data.name.replace("Person", "")
                        group_data.people.append(person_data)
                elif is_in_circle(current_data, group):
                    for person_idx in group:
                        person_data = copy.deepcopy(current_data.people[person_idx])
                        person_data.name = "circle_" + str(group_idx) + "_person_" + person_data.name.replace("Person", "")
                        group_data.people.append(person_data)
                else: 
                    for person_idx in group:
                        person_data = copy.deepcopy(current_data.people[person_idx])
                        person_data.name = "other_" + str(group_idx) + "_person_" + person_data.name.replace("Person", "")
                        group_data.people.append(person_data)
                publisher.publish(group_data)
            rospy.sleep(0.25)

def main():
    rospy.init_node('persons_group_detector', anonymous=True)
    groups_publisher = rospy.Publisher("/robot_0/detected_groups", PositionMeasurementArray, queue_size=10)
    persons_position_subscriber = rospy.Subscriber("/people_tracker_measurements", PositionMeasurementArray, people_tracker_measurements_callback)
    publish_persons_groups(groups_publisher)

if __name__=="__main__":
    main()