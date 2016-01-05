#! /bin/bash
# /etc/init.d/ControlMotorsWithJoystickTwistStartupFile

#Path to ROS and Workspace
. /opt/ros/indigo/setup.sh	
. ~/catkin_ws/devel/setup.sh

sleep 20 #wait 15 seconds for opening the ports 5 7 en 8

# Path to ROS workspace and starting the ros launch file
export ROS_PACKAGE_PATH=/opt/ros/indigo/stacks:$ROS_PACKAGE_PATH
roslaunch  turtle5k_drive_system control_motors_with_joystick_twist.launch