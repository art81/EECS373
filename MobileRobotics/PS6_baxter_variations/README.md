#In order to control the arms on the baxter robot use the following commands:

roslaunch worlds glennan_world.launch
rosrun baxter_tools enable_robot.py -e
rosrun baxter_tools tuck_arms.py -u (Example node that will tuck the arms of the bot)

Can also use this control arm joints from command line:

rostopic pub /robot/right_joint_position_controller/joints/right_s0_controller/command std_msgs/Float64 VALUE
