#!/usr/bin/python3

import rospy
import copy
import math
from people_msgs.msg import PositionMeasurementArray

measured_people = None
current_msg = None
epsilon = .5
group_dist = 4.0

def person_cb(data):
	global measured_people, current_msg
	current_msg = copy.deepcopy(data)
	measured_people = copy.deepcopy(data.people)


def identify_groups(people):
	groups = []
	taken_indices = []
	for index, person in enumerate(people):
		if index in taken_indices:
			continue
		n_group = [ person ]
		taken_indices.append(index)
		for j in range(index + 1, len(people)): 
			if j in taken_indices:
				continue
			elif math.dist( [ person.pos.x, person.pos.y ], [ people[j].pos.x, people[j].pos.y ] ) < group_dist:
				n_group.append(people[j])
				taken_indices.append(j)
		groups.append(n_group)
	return groups 

def in_a_line(grp):
	xy_poses = []
	current_cp = copy.deepcopy(grp)
	for person in current_cp:
		xy_poses.append([person.pos.x, person.pos.y])
	inline = True
	slope = None
	prev_point = xy_poses[0]
	for index, pose in enumerate(xy_poses):
		if index == 0:
			continue
		n_slope = ( pose[1] - prev_point[1] ) / ( pose[0] - prev_point[0] )
		# print(f'n_slope: {n_slope}, slope: {slope}')
		if slope is not None and abs(n_slope - slope) >= epsilon:
			inline = False
			break
		slope = n_slope
	return inline

def in_a_circle(grp ):
	# measure distance between people in the group
	distances = []
	current_cp = copy.deepcopy(grp)
	for index, person in enumerate(current_cp):

		for j in range(index + 1, len(current_cp)):
			distances.append( [ math.dist( [person.pos.x, person.pos.y], [current_cp[j].pos.x, current_cp[j].pos.y] ), index, j ] )
	# pick the largest as the diameter
	largest_dist_person_1 = -1
	largest_dist_person_2 = -1
	max_dist = -1
	max_ind = -1
	for index, length_pair in enumerate(distances):
		if length_pair[0] > max_dist:
			max_dist = length_pair[0]

			largest_dist_person_1 = length_pair[1]
			largest_dist_person_2 = length_pair[2]

			max_ind = index
	# take the midpoint as the center of the circle, and calculate the radius
	person_1_point = [ current_cp[largest_dist_person_1].pos.x, current_cp[largest_dist_person_1].pos.y ]
	person_2_point = [ current_cp[largest_dist_person_2].pos.x, current_cp[largest_dist_person_2].pos.y ]
	midpoint = [ ( person_1_point[0] + person_2_point[0] ) / 2 , ( person_1_point[1] + person_2_point[1] ) / 2 ]
	radius = max_dist / 2
	# measure the radius to each other person
	distance_from_center_by_person = []
	for person in current_cp:
		distance_from_center_by_person.append(math.dist(midpoint, [ person.pos.x, person.pos.y ] ) )
	# we can check to see if there are outliers and then reapply the algorithm perhaps? outliers might essentially be checked for if the leg detector simply doesn't see them
	# if the largest distance between the people is too far then they aren't in a circle
	if max_dist > 4.0:
		return False
	# return true if that radius is within a certain tolerance
	for i in distance_from_center_by_person:
		if abs(i - radius) >= epsilon:
			return False
	return True

def main(pub):
	global measured_people, current_msgs
	while measured_people == None or current_msg == None:
		pass
	while not rospy.is_shutdown():
		n_msg = copy.deepcopy(current_msg)
		n_msg.people = []
		groups = identify_groups(measured_people)
		for index, group in enumerate( groups ):
			if len(group) > 1 and in_a_line(group):
				for person in group:
					person.name = 'line_' + str(index) + '_' + person.name[:-1] + '_' + person.name[-1]
					n_msg.people.append(person)
			elif len(group) > 1 and in_a_circle(group):
				for person in group:
					person.name = 'circle_' + str(index) + '_' + person.name[:-1] + '_' + person.name[-1]
					n_msg.people.append(person)
			else:
				for person in group:
					person.name = 'other_' + str(index) + '_' + person.name[:-1] + '_' + person.name[-1]
					n_msg.people.append(person)
		pub.publish(n_msg)
		rospy.sleep(.3)

if __name__ == '__main__':
	rospy.init_node('persons_group_detector', anonymous=True)
	person_pub = rospy.Publisher('/robot_0/detected_groups', PositionMeasurementArray, queue_size=10)
	person_sub = rospy.Subscriber('/people_tracker_measurements', PositionMeasurementArray, person_cb)
	main(person_pub)