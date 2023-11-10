#!/usr/bin/python3

import rospy
import copy
import math
from people_msgs.msg import PositionMeasurementArray
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose
from tf.transformations import quaternion_from_euler, euler_from_quaternion

# base_scan_ground_truth
# std_msgs/Header header
#   uint32 seq
#   time stamp
#   string frame_id
# string child_frame_id
# geometry_msgs/PoseWithCovariance pose
#   geometry_msgs/Pose pose
#     geometry_msgs/Point position
#       float64 x
#       float64 y
#       float64 z
#     geometry_msgs/Quaternion orientation
#       float64 x
#       float64 y
#       float64 z
#       float64 w
#   float64[36] covariance
# geometry_msgs/TwistWithCovariance twist
#   geometry_msgs/Twist twist
#     geometry_msgs/Vector3 linear
#       float64 x
#       float64 y
#       float64 z
#     geometry_msgs/Vector3 angular
#       float64 x
#       float64 y
#       float64 z
#   float64[36] covariance

# base_scan
# std_msgs/Header header
#   uint32 seq
#   time stamp
#   string frame_id
# float32 angle_min
# float32 angle_max
# float32 angle_increment
# float32 time_increment
# float32 scan_time
# float32 range_min
# float32 range_max
# float32[] ranges
# float32[] intensities


robot_position = None
group_data = None
laser_data = None

def base_scan_callback(data):
    global laser_data
    laser_data = copy.deepcopy(data)

def detected_groups_callback(data):
    global group_data
    group_data = copy.deepcopy(data)

def base_pose_callback(data):
    global robot_position
    robot_position = copy.deepcopy(data)

def calculate_distance_to_obstacle():
    # Calculate the distance to the nearest obstacle from laser data
    if laser_data is not None:
        min_distance = min(laser_data.ranges)
        return min_distance
    return float('inf')  # Return infinity if no laser data is available

def calculate_angle_to_obstacle():
    # Calculate the angle to the nearest obstacle from laser data
    if laser_data is not None:
        min_distance_index = min(range(len(laser_data.ranges)), key=lambda i: laser_data.ranges[i])
        angle_to_obstacle = laser_data.angle_min + min_distance_index * laser_data.angle_increment
        return angle_to_obstacle
    return 0.0  # Return 0 if no laser data is available

def is_valid_person_name(person_name):
    # Check if the person name is in the valid format (e.g., 'line_1_person_2', 'circle_3_person_1')
    parts = person_name.split('_')
    return len(parts) == 4 and parts[0] in ('line', 'circle') and parts[3].startswith('person')

def get_group_type(person_name):
    # Extract the group type from the person name
    return person_name.split('_')[0]

# def handle_obstacle(laser_data):
    
#     min_distance = min(laser_data.ranges)
    
#     if min_distance < 0.5: 
#         twist_msg.linear.x = 0.0
#     else:
#         twist_msg.linear.x = 0.5
    
#     if min_distance < 1.0:  
#         left_distances = sum(laser_data.ranges[:len(laser_data.ranges)//2])
#         right_distances = sum(laser_data.ranges[len(laser_data.ranges)//2:])
        
#         if left_distances < right_distances:
#             twist_msg.angular.z = 0.0873  # Turn right
#         else:
#             twist_msg.angular.z = -0.0873  # Turn left
#     else:
#         twist_msg.angular.z = 0.0  # No turn

#     cmd_vel_pub.publish(twist_msg)

def find_distance_to_front_obstacle():
    if laser_data is not None:
        # Assuming laser_data.angle_min is the starting angle and laser_data.angle_increment is the angle between measurements
        # Find the index corresponding to the front direction (e.g., 0 degrees)
        front_index = int((0.0 - laser_data.angle_min) / laser_data.angle_increment)

        # Get the distance measurement at the front
        distance_to_front = laser_data.ranges[front_index]

        return distance_to_front
    return float('inf')  # Return infinity if no laser data is available

def move_towards_group():
    # Move the robot towards the detected group with obstacle avoidance

    twist_data = Twist()
    # Check if group data is available
    if group_data is not None and len(group_data.people) > 0:
        valid_people = group_data.people
        rospy.loginfo("before" + str(valid_people))
        if len(valid_people) > 0:
            # Choose the first valid person for simplicity
            group_type = get_group_type(valid_people[0].name)
            distance_to_obstacle = find_distance_to_front_obstacle()

            rospy.loginfo("before" + str(group_type))
            

            if distance_to_obstacle < 0.3:
                # Rotate the robot away from the obstacle
                twist_data.linear.x = 0.0
                twist_data.angular.z = 0.5
                return twist_data
            else:
                # Move the robot based on the group type
                if group_type == 'line':
                    # Move towards the end of the line (choose the last person)
                    target_person = valid_people[0]
                    rospy.loginfo("before" + str(twist_data))
                    twist_data = move_towards_target(target_person.pos)
                    rospy.loginfo("after" + str(twist_data))
                elif group_type == 'circle':
                    # Move towards the center of the circle (choose the first person)
                    twist_data = move_towards_target(valid_people[0].pos)

    return twist_data

def calculate_distance_to_target(target_pos):
    # Calculate the distance to the target position
    robot_x = robot_position.pose.pose.position.x
    robot_y = robot_position.pose.pose.position.y
    distance = math.sqrt((target_pos.x - robot_x)**2 + (target_pos.y - robot_y)**2)
    return distance

def calculate_angle_to_target(target_pos):
    # Calculate the angle to the target position
    robot_x = robot_position.pose.pose.position.x
    robot_y = robot_position.pose.pose.position.y
    angle = math.atan2(target_pos.y - robot_y, target_pos.x - robot_x)
    return angle

def calculate_required_rotation(target_pos):
    # Calculate the required rotation to face the target position
    _, _, robot_yaw = euler_from_quaternion([
        robot_position.pose.pose.orientation.x,
        robot_position.pose.pose.orientation.y,
        robot_position.pose.pose.orientation.z,
        robot_position.pose.pose.orientation.w
    ])
    target_angle = calculate_angle_to_target(target_pos)
    required_rotation = target_angle - robot_yaw
    return required_rotation

def move_towards_target(target_pos):
    # Move the robot towards the target position

    twist_data = Twist()

    # Calculate required rotation to face the target
    required_rotation = calculate_required_rotation(target_pos)

    # Adjust angular velocity to perform the required rotation
    twist_data.angular.z = required_rotation

    # If the required rotation is small, move linearly towards the target
    if abs(required_rotation) < 0.1:
        twist_data.linear.x = min(1, calculate_distance_to_target(target_pos))

    return twist_data

def publish_robot_moving_data(publisher):
    twist_data = move_towards_group()
    # rospy.loginfo(twist_data)
    publisher.publish(twist_data)

def main():
    rospy.init_node('robot_mover', anonymous=True)
    robot_moving_data_publisher = rospy.Publisher("/robot_0/cmd_vel", Twist, queue_size=20)
    rospy.Subscriber("/robot_0/base_scan", LaserScan, base_scan_callback)
    rospy.Subscriber("/robot_0/detected_groups", PositionMeasurementArray, detected_groups_callback)
    rospy.Subscriber("/robot_0/base_pose_ground_truth", Odometry, base_pose_callback)

    rate = rospy.Rate(3)  # 10 Hz
    while not rospy.is_shutdown():
        publish_robot_moving_data(robot_moving_data_publisher)
        rate.sleep()

if __name__ == "__main__":
    main()