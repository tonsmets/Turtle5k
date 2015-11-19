#include "ros/ros.h"
#include "std_msgs/String.h"
ros::Publisher pBallHandlingInfoPub;
ros::Subscriber pBallHandlingCmdSub;

std_msgs::String getHandlingParams()
{
	//TODO: create location from input
	return std_msgs::String();
}

void frameCallback(const std_msgs::String::ConstPtr& aMessage)
{
	ROS_INFO("[BALLHANDLING] Got %s, From: BALLCONTROL", aMessage->data.c_str());
	std_msgs::String pMessage = getHandlingParams();
	pBallHandlingInfoPub.publish(pMessage);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "t5k_ballhandling");
	ros::NodeHandle pHandle;
	
	pBallHandlingInfoPub = pHandle.advertise<std_msgs::String>("/t5k/ballhandlinginfo", 1000);
	pBallHandlingCmdSub = pHandle.subscribe("/t5k/ballhandlingcommands", 1000, frameCallback);
	
	ros::spin();
	
	return 0;
}
