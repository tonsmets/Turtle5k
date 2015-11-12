#include "ros/ros.h"
#include "std_msgs/String.h"
ros::Publisher pTacticsPub;
ros::Subscriber pPathSub;
ros::Subscriber pWorldSub;
ros::Subscriber pBallControlInfoSub;

std_msgs::String getTactic()
{
	//TODO: generate appropriate tactic
	return std_msgs::String();
}

void pathCallback(const std_msgs::String::ConstPtr& aMessage)
{
	ROS_INFO("[TACTICS] Got %s, From: PATHPLANNING", aMessage->data.c_str());
	std_msgs::String pMessage = getTactic();
	pTacticsPub.publish(pMessage);
}

void worldCallback(const std_msgs::String::ConstPtr& aMessage)
{
	ROS_INFO("[TACTICS] Got %s, From: WORLD", aMessage->data.c_str());
	std_msgs::String pMessage = getTactic();
	pTacticsPub.publish(pMessage);
}

void ballcontrolCallback(const std_msgs::String::ConstPtr& aMessage)
{
	ROS_INFO("[TACTICS] Got %s, From: BALLCONTROL", aMessage->data.c_str());
	std_msgs::String pMessage = getTactic();
	pTacticsPub.publish(pMessage);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "t5k-tactics");
	ros::NodeHandle pHandle;
	
	pTacticsPub = pHandle.advertise<std_msgs::String>("/t5k/tactics", 1000);
	pPathSub = pHandle.subscribe("/t5k/paths", 1000, pathCallback);
	pWorldSub = pHandle.subscribe("/t5k/world", 1000, worldCallback);
	pBallControlInfoSub = pHandle.subscribe("t5k/ballcontrolinfo", 1000, ballcontrolCallback);
	
	ros::spin();
	
	return 0;
}
