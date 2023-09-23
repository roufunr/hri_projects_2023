#!/bin/bash

# Function to run a command in a new terminal window
run_in_terminal() {
    gnome-terminal -- bash -c "$1"
}

# Run each pair of lines in separate terminals
run_in_terminal "source ~/hri2023/devel/setup.bash && roslaunch week2 person_world.launch"
sleep 5  # Delay for 2 seconds
run_in_terminal "source ~/hri2023/devel/setup.bash && rosrun rviz rviz -d ~/hri2023/src/hri_projects_2023/week2/person.rviz"
sleep 5  # Delay for 2 seconds
run_in_terminal "source ~/hri2023/devel/setup.bash && roslaunch week2 leg_detector.launch"
sleep 3  # Delay for 2 seconds
run_in_terminal "source ~/hri2023/devel/setup.bash && rosrun week2 robot_broadcaster.py"
sleep 2  # Delay for 2 seconds
run_in_terminal "source ~/hri2023/devel/setup.bash && rosrun week2 person_broadcaster.py"
sleep 2  # Delay for 2 seconds
run_in_terminal "source ~/hri2023/devel/setup.bash && rosrun week2 tf_listener.py"
sleep 2  # Delay for 2 seconds
