#include "ros/ros.h"
#include "std_msgs/String.h"
ros::Publisher pVelocityPub;
ros::Publisher pPIDRegulationPub;
ros::Subscriber pWheelControlCmdSub;
ros::Subscriber pEncoderDataSub;

std_msgs::String getVelocity()
{
	//TODO: create command from input
	return std_msgs::String();
}

std_msgs::String getPIDRegulation()
{
	//TODO: create PID-Reg message
	return std_msgs::String();
}

void wheelcontrolcommandsCallback(const std_msgs::String::ConstPtr& aMessage)
{
	ROS_INFO("[WHEELCONTROL] Got %s, From: COMMANDS", aMessage->data.c_str());
	std_msgs::String pMessage = getPIDRegulation();
	pPIDRegulationPub.publish(pMessage);
}

void encoderDataCallback(const std_msgs::String::ConstPtr& aMessage)
{
	ROS_INFO("[WHEELCONTROL] Got %s, From: WHEELDRIVER", aMessage->data.c_str());
	std_msgs::String pMessage = getVelocity();
	pVelocityPub.publish(pMessage);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "t5k_wheelcontrol");
	ros::NodeHandle pHandle;
	
	pVelocityPub = pHandle.advertise<std_msgs::String>("/t5k/velocity", 1000);
	pPIDRegulationPub = pHandle.advertise<std_msgs::String>("/t5k/pidregulation", 1000);
	pWheelControlCmdSub = pHandle.subscribe("/t5k/wheelcontrolcommands", 1000, wheelcontrolcommandsCallback);
	pEncoderDataSub = pHandle.subscribe("/t5k/encoderdata", 1000, encoderDataCallback);
	
	ros::spin();
	
	return 0;
}
