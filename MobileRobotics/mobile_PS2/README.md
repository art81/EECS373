# my_stdr_control

This package includes a program (lidar_alarm.cpp) that returns true if the robot in the stdr
is unsafe to move forward and false if it is clear in front of it. Using this with another
program in the package (reactive_commander.cpp) will make iut so the robot can explore the stdr workspace and will never collide with any walls.

## Example usage
roslaunch stdr_launchers server_with_map_and_gui_plus_robot.launch
rosrun lidar_alarm lidar_alarm
rosrun stdr_control reactive_commander
