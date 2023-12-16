Assignment 2: Look at hand / look at where the hand is pointing

Task 1: The two nodes above
Task 1.1: Start by making a node to look at the hand. You can do this by figuring out which angle a frane on the robot (you could use one of the camera frames or the Head frame has to the hand). Once you have the angles, then you can change HeadYaw and HeadPitch to point at those angles.
Task 1.2: Next let's make a node that publishes a new frame, 1 meter from one of the hand frames (make sure it looks right in rviz). Instead of looking at the hand, write another node to look at the place the hand is pointing to (your new frame).

Task 1.1
    step 1: Open a terminal and run "roslaunch nao_description robot_state_publisher.launch"
    step 2: Open another terminal and run "rviz" and open a rviz file and select RobotModel
    step 3: Open another terminal and run "rosrun joint_state_publisher_gui joint_state_publisher_gui" and close the terminal
    step 4: Open another terminal and run "rosrun week8 look_at_hand.py"

Task 1.2
    step 1: Open a terminal and run "roslaunch nao_description robot_state_publisher.launch"
    step 2: Open another terminal and run "rviz" and open a rviz file and select RobotModel
    step 3: Open another terminal and run "rosrun joint_state_publisher_gui joint_state_publisher_gui"
    step 4: Open another terminal and run "rosrun week8 publish_meter_ahead_frame.py"
    step 5: Open another terminal and run "rosrun week8 look_at_pointed_frame.py"

Task 2: The keyframe animator we made last week (week7)
    step 1: Open a terminal and run "roslaunch nao_description robot_state_publisher.launch"
    step 2: Open another terminal and run "rviz" and open a rviz file and select RobotModel
    step 3: Open another terminal and run "rosrun joint_state_publisher_gui joint_state_publisher_gui" and close the terminal
    step 4: Open another terminal and run "rosrun week7 animation.py"