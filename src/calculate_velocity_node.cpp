/*****************************************************************************************************************************************
Function: 
This function will convert a vector message x,y and angular z into RPM values for 3 omniwheels.

The following coordinate system will be used for the robot(Turtle5k)

shooting system is on the x-axe

		^x
		|
		|
	   zX----->y

Pre:
Twist message on topic motorspeed_set

Post:
Float32Multiarray on topic mcWheelVelocityMps. THe 3 rpm values for the motor will be written in array[4], array[6] and array[7]

Writer 		: Niek Francke
date 		: 11-12-2015
********************************************************************************************************************************************/

#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <string>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
#include "ros/ros.h"
#include <math.h>

//-----settings
#define PUBLISH_TOPIC_NAME 						"mcWheelVelocityMps"
#define SUBSCRIBE_TOPIC_NAME					"motorspeed_set"
#define SUBSCRIBE_TOPIC_BUFFER_SIZE				1
#define PUBLISH_TOPIC_BUFFER_SIZE				1
#define ANGLE_1									0 	//degrees
#define ANGLE_2									120 //degrees
#define ANGLE_3									240 //degrees
#define THETA									30	//degrees
#define RADIUS_OMNI_WHEEL						0.1016/2 //meter
#define RADIUS_DRIVING_SYSTEM					0.22 //meter
#define MOTOR_TO_WHEEL_TRANSMISSION_RATIO 		12
#define DEBUG_SPEED								100

using namespace std;

/*****************************************************************************************************************************************
Start defining class PublischAndSubscribe
********************************************************************************************************************************************/
class PublishAndSubscribe
{
public:
	
	///////////////////////////////////////////////////////////////////////////////////////////
	//Function: create class
	//pre: 	-
	//post: -
	///////////////////////////////////////////////////////////////////////////////////////////
	PublishAndSubscribe(ros::NodeHandle nh)
	{
		sub = nh.subscribe(SUBSCRIBE_TOPIC_NAME,SUBSCRIBE_TOPIC_BUFFER_SIZE, &PublishAndSubscribe::twistMessageReceived, this);
		pub = nh.advertise<std_msgs::Float32MultiArray>(PUBLISH_TOPIC_NAME,PUBLISH_TOPIC_BUFFER_SIZE);

		iTwistMessageReceivedCounter = 0;

		ROS_INFO("calculate_velocity_node is initialized");
	}
	
	/////////////////////////////////////////////////////////////////////////////
	//function: this function responds on a Twist message.
	//pre: linear values in m/s en angular values in rad/s.
	//post: there will be an Float32MultiArray send on a topic. In this array the RPM's per motor will be defined.
	/////////////////////////////////////////////////////////////////////////////
	void twistMessageReceived(const geometry_msgs::Twist::ConstPtr& msg)
	{
		iTwistMessageReceivedCounter ++;

		ROS_INFO_ONCE("Received a twist message for the first time.");

		//linear values out of twist messages.
		float fX = msg->linear.x;
		float fY = msg->linear.y;
		float fOmega = msg->angular.z;

		//convert degree values in to radian value's
		float fAngle1 = ((float)ANGLE_1/180)*M_PI;
		float fAngle2 = ((float)ANGLE_2/180)*M_PI;
		float fAngle3 = ((float)ANGLE_3/180)*M_PI;
		float fTheta  = ((float)THETA/180)*M_PI;

		float fRadiusOmniwheel = RADIUS_OMNI_WHEEL;	//this is needed, because the define won't work in a formule. (i don't know why)
		float fMotorToWheelTransmissionRatio = MOTOR_TO_WHEEL_TRANSMISSION_RATIO;	

		//send data
		if(iTwistMessageReceivedCounter % DEBUG_SPEED == 0 ){
			ROS_DEBUG("x = %f", fX);
			ROS_DEBUG("y = %f", fY);
			ROS_DEBUG("omega = %f", fOmega);
			ROS_DEBUG("angle1 is %f  pi radians and %i degrees" ,(fAngle1/M_PI), ANGLE_1);
			ROS_DEBUG("angle2 is %f pi radians and %i degrees" , (fAngle2/M_PI), ANGLE_2);
			ROS_DEBUG("angle3 is %f pi radians and %i degrees" , (fAngle3/M_PI), ANGLE_3);
			ROS_DEBUG("theta is %f pi radians and %i degrees" , (fTheta/M_PI), THETA);
			ROS_DEBUG("radius omni wheel: %f", fRadiusOmniwheel);
			ROS_DEBUG("radius omni wheel: %f", fMotorToWheelTransmissionRatio);
		}

		float fSpeedWheel[10];

		//convert the x, y and angular z values in the 3 omniwheel speed values.
		//Vwheel = Vlinear + Vangular
		//Vlinear = -sin(theta + a1)*y + cos(thetha + a1)*x
		//Vangular = Radius * omega
		fSpeedWheel[7] = (-sin(fTheta + fAngle1)*fY + (cos(fTheta + fAngle1))*fX + RADIUS_DRIVING_SYSTEM*(-fOmega))/fRadiusOmniwheel; 
		fSpeedWheel[8] = (-sin(fTheta + fAngle2)*fY + (cos(fTheta + fAngle2))*fX + RADIUS_DRIVING_SYSTEM*(-fOmega))/fRadiusOmniwheel;
		fSpeedWheel[5] = (-sin(fTheta + fAngle3)*fY + (cos(fTheta + fAngle3))*fX + RADIUS_DRIVING_SYSTEM*(-fOmega))/fRadiusOmniwheel;

		if(iTwistMessageReceivedCounter % DEBUG_SPEED == 0 ){
			//Debug messages
			ROS_DEBUG("speed wheel 1 rad/s = %f", fSpeedWheel[7]);
			ROS_DEBUG("speed wheel 2 rad/s = %f", fSpeedWheel[5]);
			ROS_DEBUG("speed wheel 3 rad/s = %f", fSpeedWheel[8]);
		}

		//convert rad/s to radian and convert motor rpm to wheel RPM
		//RPM = rad * 60/2pi
		for(int i = 0 ; i < 10 ; i++){
			fSpeedWheel[i] = fSpeedWheel[i] * (60 / ( 2 * M_PI));
			fSpeedWheel[i] = fSpeedWheel[i] * fMotorToWheelTransmissionRatio;
		}

		if(iTwistMessageReceivedCounter % DEBUG_SPEED == 0 ){
			//Debug messages
			ROS_DEBUG("speed wheel 1 RPM = %f", fSpeedWheel[7]);
			ROS_DEBUG("speed wheel 2 RPM = %f", fSpeedWheel[5]);
			ROS_DEBUG("speed wheel 3 RPM = %f", fSpeedWheel[8]);
		}

		//Define output message (Float32MultiArray)
		std_msgs::Float32MultiArray msg_out;

		//Clear data. If you don't don't use this function the array won't start again on number 0 with filling new data.
		msg_out.data.clear();

		//define data. Set RPM in output message
      	msg_out.data.push_back(0);
	    msg_out.data.push_back(0);
	    msg_out.data.push_back(0);
	    msg_out.data.push_back(0);
	    msg_out.data.push_back(fSpeedWheel[5]);
	    msg_out.data.push_back(0);
	    msg_out.data.push_back(fSpeedWheel[7]);
	    msg_out.data.push_back(fSpeedWheel[8]);
	    msg_out.data.push_back(0);
	    msg_out.data.push_back(0);

	    //Send message
	    pub.publish(msg_out);
	}

private:
	ros::Subscriber sub;	//define ros subscriber
	ros::Publisher pub;
	int iTwistMessageReceivedCounter;
};
/*****************************************************************************************************************************************
end of defining class Subscribe
********************************************************************************************************************************************/

/*****************************************************************************************************************************************
Start of main
********************************************************************************************************************************************/
int main(int argc, char **argv  )
{
	//initialize ROS
	ros::init(argc, argv, "calculate_velocity_node");
	
	//create nodehandle
	ros::NodeHandle nh;	

	//create class
	PublishAndSubscribe PandSobject(nh);

	//wait until a Float32MulitArray is received and run the callback function
	ros::spin();

	return 0;
}

/*****************************************************************************************************************************************
End of main
********************************************************************************************************************************************/
