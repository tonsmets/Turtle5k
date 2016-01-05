Avans Turtle5k Branch
============
This directory needs to be placed in a catkin (ros) directory. If the standard ROS tutorials are followed it is needs to be placed in this directory : Home/catkin_ws/src/

///////////5KdriveWithVC/////////////
The directory 5KdriveWithVC is the code for the motordrivers (microcontrollers). It is only usefull for the omniwheel drivers, because the data feedback is different from the ball handling. This code can be opened in atmel studio.

///////////src/////////////
The directory "src" is for the source code of the nodes in ROS.
caclulate_velocity_node.cpp => convert twist message on topic motorspeed_set in 3 RPM speeds for the motordrivers. The RPMs are sended in a float32multiarray on topic mcWheelVelocityMps.

joystick_to_rpm_multiarray_node.cpp => converts standard joytick messages in 3 RPM speeds for the motordrivers. The RPMs are sended in a float32multiarray on topic mcWheelVelocityMps.

joystick_to_twist_node.cpp => converts standard joystick messages in a twist message on topic motorspeed_set. 

motor_driver_node => reads on topic mcWheelVelocityMps. The data on this topic is sended in a float32Multiarray. This data will be sended via RS422 to the microcontrollers.

rpm_sender_multi_array_node.cpp => is a test program that sends a constand RPM value on topic mcWheelVelocityMps.

rotateWheelsTest.cpp => this is a test program for the G++ compiler. With this code the omniwheels can be turnd on. This program is not running on ROS.

///////////launch files/////////////
The launch files are placed in the root of the directory. The name of this file covers the funtion of the file.

///////////linuxScript/////////////
see the readme in this folder.
