#include "ros/ros.h"
#include "std_msgs/String.h"
ros::Publisher pGoalLocationPub;
ros::Subscriber pFrameSub;

std_msgs::String getGoalLocation()
{
	//TODO: create location from input
	return std_msgs::String();
}

void frameCallback(const std_msgs::String::ConstPtr& aMessage)
{
	ROS_INFO("[GOALDETECTOR] Got %s, From: CAMERA", aMessage->data.c_str());
	std_msgs::String pMessage = getGoalLocation();
	pGoalLocationPub.publish(pMessage);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "t5k-tactics");
	ros::NodeHandle pHandle;
	
	pGoalLocationPub = pHandle.advertise<std_msgs::String>("/t5k/ballLocation", 1000);
	pFrameSub = pHandle.subscribe("/t5k/frame", 1000, frameCallback);
	
	ros::spin();
	
	return 0;
}
