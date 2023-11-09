#!/usr/bin/python3

import rospy
import copy
import math
import sys
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from sensor_msgs.msg import LaserScan
from people_msgs.msg import PositionMeasurementArray



# globals

def is_same_angle(ang1, ang2, eps):
    temp1 = ang1
    temp2 = ang2

    if ang1 <= 0:
        temp1 = math.pi * 2 + ang1

    if ang2 <= 0:
        temp2 = math.pi * 2 + ang2

    distance1 = temp1 - temp2
    distance2 = temp2 - temp1

    if distance1 <= 0:
        distance1 += math.pi * 2

    if distance2 <= 0:
        distance2 += math.pi * 2

    if distance1 >= distance2:
        return distance2 <= eps
    else:
        return distance1 <= eps


def get_smallest_dist_and_direction(ang1, ang2):

    temp1 = ang1
    temp2 = ang2

    if ang1 <= 0:
        temp1 = math.pi * 2 + ang1

    if ang2 <= 0:
        temp2 = math.pi * 2 + ang2

    distance1 = temp1 - temp2
    distance2 = temp2 - temp1

    if distance1 <= 0:
        distance1 += math.pi * 2

    if distance2 <= 0:
        distance2 += math.pi * 2

    if distance1 >= distance2:
        return distance2, 'left'
    else:
        return distance1, 'right'

class RobotMover:

    def __init__(self):
        rospy.init_node('join_group_mover', anonymous=True)

        self.move_pub = rospy.Publisher('/robot_0/cmd_vel', Twist, queue_size=10)

        self.person_sub = rospy.Subscriber('/robot_0/detected_groups', PositionMeasurementArray, self.group_detection_cb)
        self.laser_sub = rospy.Subscriber('/robot_0/base_scan', LaserScan, self.laser_cb)
        self.position_sub = rospy.Subscriber('/robot_0/base_pose_ground_truth', Odometry, self.set_pos_cb)

        self.pos = None
        self.laser_data = None
        self.groups = []

        # wait until cbs get called the first time
        while self.pos == None or self.laser_data == None or self.groups == []:
            pass

    def group_detection_cb (self, data):
        number_of_groups = -1

        for i in data.people:
            parsed_name = i.name.split('_')

            if int(parsed_name[1]) > number_of_groups:
                number_of_groups = int(parsed_name[1])

        groups = []

        for i in range(number_of_groups + 1):
            groups.append([])

        for i in data.people:
            parsed_name = i.name.split('_')

            groups[int(parsed_name[1])].append(copy.deepcopy(i))

        self.groups = groups

    def laser_cb(self, data):
        self.laser_data = data

    def set_pos_cb(self, data):
        self.pos = data.pose.pose

    def find_closest_obstacle(self):    
        min_range = 10000000 

        for i in self.laser_data.ranges:
            if i < min_range:
                min_range = i

        return min_range

    def find_longest_sequence(self, seq_num):
        longest_num = -1
        longest_seq_index = -1
        in_seq = False

        seq_min = -1
        seq_len = -1

        seqs = []

        for i in range(len(self.laser_data.ranges)):
            if self.laser_data.ranges[i] == seq_num:
                if not in_seq:
                    seq_min = i
                    seq_len = 0

                    in_seq = True
                
                seq_len += 1

            else:
                if in_seq:
                    seqs.append([seq_min, i])

                    if seq_len > longest_num:
                        longest_num = seq_len
                        longest_seq_index = len(seqs) - 1

                in_seq = False
                seq_min = -1
                seq_len = -1
                
        if in_seq:
            seqs.append([seq_min, len(self.laser_data.ranges) - 1])

            if seq_len > longest_num:
                longest_num = seq_len
                longest_seq_index = len(seqs) - 1

        # print(f'biggest index is {longest_seq_index}')
        # for i in seqs:
        #     print(f'seq {i[0]} -> {i[1]} of len {i[1] - i[0]}')

        return seqs[longest_seq_index]

    def find_farthest_obstacle_angle(self):
        current_max = -1

        for i in range(len(self.laser_data.ranges)):
            if self.laser_data.ranges[i] > current_max:
                current_max = self.laser_data.ranges[i]

        longest_seq = self.find_longest_sequence(current_max)

        return self.laser_data.angle_min + ( math.floor( ( longest_seq[1] - longest_seq[0] ) / 2 ) + longest_seq[0] ) * self.laser_data.angle_increment 


    def transform_sensor_angle_to_world_angle(self, angle):
        (init_r, init_p, init_y) = euler_from_quaternion([ self.pos.orientation.x, self.pos.orientation.y, self.pos.orientation.z, self.pos.orientation.w ])

        return init_y + angle

    def go_forward(self, dist):

        twistmsg = Twist()

        print("going forward!")

        try:
            twistmsg.linear.x = 1.0

            initial_pos = copy.deepcopy(self.pos.position)

            initial_pos = [ initial_pos.x, initial_pos.y, initial_pos.z ]

            # until we've moved the specified square side length
            while math.dist(initial_pos, [ self.pos.position.x, self.pos.position.y, self.pos.position.z ]) <= dist:

                self.move_pub.publish(twistmsg)


        except rospy.ROSInterruptException:
            rospy.logfatal("Movement interrupted!")

    def go_forward_until_obstacle_or_goal(self, min_dist, goal):

        twistmsg = Twist()

        print("going forward!")

        try:
            twistmsg.linear.x = 1.0

            while self.find_closest_obstacle() >= min_dist and math.dist([ self.pos.position.x, self.pos.position.y ], goal) >= min_dist :
                self.move_pub.publish(twistmsg)

            twistmsg.linear.x = 0.0
            self.move_pub.publish(twistmsg)
        
        except rospy.ROSInterruptException:
            rospy.logwarn("Movement Interrupted!")

        if self.find_closest_obstacle() < min_dist and not math.dist([ self.pos.position.x, self.pos.position.y ], goal) < min_dist:
            return True

        return False


    def turn_right(self, rad, forward_speed):
        twistmsg = Twist()

        print("turning right!")

        try:
            (init_r, init_p, init_y) = euler_from_quaternion([ self.pos.orientation.x, self.pos.orientation.y, self.pos.orientation.z, self.pos.orientation.w ])

            end_y = init_y - rad

            cur_y = init_y

            if end_y >= math.pi:
                end_y += math.pi * 2

            twistmsg.angular.z = -1 * math.pi / 8
            twistmsg.linear.x = forward_speed


            # was lazy so found smallest angle approach
            while not is_same_angle(cur_y, end_y, .08):

                (r, p, y) = euler_from_quaternion([ self.pos.orientation.x, self.pos.orientation.y, self.pos.orientation.z, self.pos.orientation.w ] )

                cur_y = y

                self.move_pub.publish(twistmsg)


        except rospy.ROSInterruptException:
            rospy.logfatal("Turning Interrupted!")

    def turn_left(self, rad, forward_speed):
        twistmsg = Twist()

        print("turning left!")

        try:
            (init_r, init_p, init_y) = euler_from_quaternion([ self.pos.orientation.x, self.pos.orientation.y, self.pos.orientation.z, self.pos.orientation.w ])

            end_y = init_y + rad

            cur_y = init_y

            if end_y >= math.pi:
                end_y -= math.pi * 2
            
            twistmsg.angular.z = math.pi / 8
            twistmsg.linear.x = forward_speed

            # was lazy so found smallest angle approach
            while not is_same_angle(cur_y, end_y, .08):

                (r, p, y) = euler_from_quaternion([ self.pos.orientation.x, self.pos.orientation.y, self.pos.orientation.z, self.pos.orientation.w ] )

                cur_y = y

                self.move_pub.publish(twistmsg)


        except rospy.ROSInterruptException:
            rospy.loginfo("Turning Interrupted!")

    def turn_toward(self, angle):
        (init_r, init_p, init_y) = euler_from_quaternion([ self.pos.orientation.x, self.pos.orientation.y, self.pos.orientation.z, self.pos.orientation.w ])

        dist, direction = get_smallest_dist_and_direction(init_y, angle)

        if 'right' == direction:
            self.turn_right(dist, 0.0)
        else:
            self.turn_left(dist, 0.0)


    def avoid_obstacle(self):
       angle = self.find_farthest_obstacle_angle()

       world_angle = self.transform_sensor_angle_to_world_angle(angle)

       self.turn_toward(world_angle)

       self.go_forward(.6)

    def move_to_destination(self, dest):

        # until we are at the destination
        while math.dist( [self.pos.position.x, self.pos.position.y], dest ) > .6:

            angle_toward = math.atan2(dest[1] - self.pos.position.y, dest[0] - self.pos.position.x)

            print("Turning toward goal!")

            self.turn_toward(angle_toward)

            print("Moving toward goal!")

            hit_obstacle = self.go_forward_until_obstacle_or_goal(.4, dest)

            if hit_obstacle:
                print("Too close to an obstacle, avoiding!")
                self.avoid_obstacle()

    def get_group_midpoint(self, group):

        avg_x_y = [ 0, 0 ]

        n = len(group)

        for i in group:
            avg_x_y[0] += i.pos.x
            avg_x_y[1] += i.pos.y

        x = avg_x_y[0] / n
        y = avg_x_y[1] / n

        return [x , y]

    def find_nearest_group(self):

        min_dist = 100000
        min_index = -1

        groups_cp = copy.deepcopy(self.groups)

        for index, val in enumerate(groups_cp):
            midpoint = self.get_group_midpoint(val)

            distance = math.dist([self.pos.position.x, self.pos.position.y], midpoint)

            if distance < min_dist:
                min_dist = distance
                min_index = index

        return copy.deepcopy(groups_cp[min_index])

    def get_group_line_slope(self, group):

        avg_slope = 0

        for i in range(len(group) - 1):
            position_1 = [group[i].pos.x, group[i].pos.y]
            position_2 = [group[i + 1].pos.x, group[i + 1].pos.y]

            if position_2[0] > position_1[0]:
                tmp = position_1
                position_1 = position_2
                position_2 = tmp

            print('2 points')
            print(position_1)
            print(position_2)

            avg_slope += (position_2[1] - position_1[1]) / (position_2[0] - position_1[0])

            print('slope: ')
            print(f'({position_2[1]} - {position_1[1]}) / ({position_2[0]} - {position_1[0]}) == {(position_2[1] - position_1[1]) / (position_2[0] - position_1[0])}')

        print(f'avg_slope: {avg_slope / ( len(group) - 1 )}')
        return avg_slope / ( len(group) - 1 )

    def get_min_group_dist(self, group, point):

        min_dist = 100000
        min_index = -1

        for index, val in enumerate(group):
            dist = math.dist(point, [val.pos.x, val.pos.y])

            if dist < min_dist:
                min_dist = dist
                min_index = index

        return min_dist

    def get_farleft(self, group):

        farleft = [10000, 10000]

        for i in group:

            if i.pos.x < farleft[0]:
                farleft[0] = i.pos.x
                farleft[1] = i.pos.y

        return farleft


    def get_join_line_pos(self, group):
        
        print('hahdh')
        slope = self.get_group_line_slope(group)

        # we are assuming that the end of the line is always west
        leftpoint = self.get_farleft(group)

        delta_x = -.5

        n_x = leftpoint[0] + delta_x
        n_y = delta_x * slope + leftpoint[1]

        print(f'{n_x} * {slope} + {leftpoint[1]} == {n_y}')

        return [ n_x, n_y ]

    def get_join_circle_pos(self, group):
        return self.get_group_midpoint(group)

    def get_join_other_pos(self, group):
        midpoint = self.get_group_midpoint(group)

        # joint half a meter west of the person
        midpoint[0] -= .5

        return midpoint

    def join_nearest_group(self):
        
        # first get destination
        nearest = self.find_nearest_group()

        dest = [0, 0]

        if 'circle' in nearest[0].name:
            dest = self.get_join_circle_pos(nearest)
        elif 'line' in nearest[0].name:
            print('heehaw')
            dest = self.get_join_line_pos(nearest)
        else:
            dest = self.get_join_other_pos(nearest)

        # then move to destination
        self.move_to_destination(dest)

def main():
    robot_mover = RobotMover()
    robot_mover.join_nearest_group()


if __name__ == '__main__':
    main()