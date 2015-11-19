#include "ros/ros.h"
#include "std_msgs/String.h"
ros::Publisher pShootingInfoPub;
ros::Subscriber pShootingCmdSub;

std_msgs::String getShootingParams()
{
	//TODO: create location from input
	return std_msgs::String();
}

void frameCallback(const std_msgs::String::ConstPtr& aMessage)
{
	ROS_INFO("[SHOOTING] Got %s, From: BALLCONTROL", aMessage->data.c_str());
	std_msgs::String pMessage = getShootingParams();
	pShootingInfoPub.publish(pMessage);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "t5k_shooting");
	ros::NodeHandle pHandle;
	
	pShootingInfoPub = pHandle.advertise<std_msgs::String>("/t5k/shootinginfo", 1000);
	pShootingCmdSub = pHandle.subscribe("/t5k/shootingcommands", 1000, frameCallback);
	
	ros::spin();
	
	return 0;
}
