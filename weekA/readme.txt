Assignment Speech Generation
Task 1:
Step 1: Open a terminal and run "roslaunch ros_vosk ros_vosk.launch"
Step 2: Open another terminal and run "rosrun weekA speech_repeater.py"
Step 3: Say something, you'll hear the same thing from your computer speaker

Task 2: 
Step 1: Open a terminal and run "roslaunch ros_vosk ros_vosk.launch"
Step 2: Open another terminal and run "roslaunch nao_description robot_state_publisher.launch"
Step 3: Open another terminal and run "rviz"
Step 4: open .rviz file
Step 5: Open another terminal and run "rosrun joint_state_publisher_gui joint_state_publisher_gui"
Step 6: close step 5
step 7: Open another terminal and run "rosrun weekA robot_movement_controller.py"
step 8: say "look at your hand", he'll look at his right hand.
step 9: get "get ready", he'll go to initial state


Task 3: 
Step 1: Open a terminal and run "roslaunch ros_vosk ros_vosk.launch"
Step 2: Open another terminal and run "rosrun weekA yes_no_questions.py" and it'll ask you three yes/no question
Step 3: based on your answer, finally it'll show you a prompt!

