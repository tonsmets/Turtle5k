#include "ros/ros.h"
#include "std_msgs/String.h"

#include "../hardware/headers/MovementTestStub.h"

ros::Publisher pVelocityPub;
ros::Publisher pPIDRegulationPub;
ros::Subscriber pWheelControlCmdSub;
ros::Subscriber pEncoderDataSub;

MovementTestStub movement;

void wheelcontrolcommandsCallback(const std_msgs::Movement::ConstPtr& aMessage)
{
	ROS_INFO("[WHEELCONTROL] Got %f %f, From: COMMANDS", aMessage->speed, aMessage->angle);

	double newSpeed = atof(aMessage->data.c_str());
	movement.setSpeed(newSpeed);
	ROS_INFO("setSpeed: [%f]", newSpeed);
}

void encoderDataCallback(const std_msgs::String::ConstPtr& aMessage)
{
	ROS_INFO("[WHEELCONTROL] Got %s, From: WHEELDRIVER", aMessage->data.c_str());
	std_msgs::String pMessage = getVelocity();
	pVelocityPub.publish(pMessage);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "t5k-wheelcontrol");
	ros::NodeHandle pHandle;
	
	pVelocityPub = pHandle.advertise<std_msgs::String>("/t5k/velocity", 1000);
	pPIDRegulationPub = pHandle.advertise<std_msgs::String>("/t5k/pidregulation", 1000);
	pWheelControlCmdSub = pHandle.subscribe("/t5k/wheelcontrolcommands", 1000, wheelcontrolcommandsCallback);
	pEncoderDataSub = pHandle.subscribe("/t5k/encoderdata", 1000, encoderDataCallback);
	
	ros::spin();
	
	return 0;
}
