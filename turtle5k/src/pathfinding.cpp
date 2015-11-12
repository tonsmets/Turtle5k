#include "ros/ros.h"
#include "std_msgs/String.h"

ros::Publisher pPathPub;
ros::Subscriber pWorldSub;

std_msgs::String calcPath()
{
	//TODO: Calculate Path
	return std_msgs::String();
}

void worldCallback(const std_msgs::String::ConstPtr& aMessage)
{
	ROS_INFO("[PATHFINDING] Got %s, From: WORLD", aMessage->data.c_str());
	calcPath();
	std_msgs::String pMessage = calcPath();
	pPathPub.publish(pMessage);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "t5k-pathfinding");
	ros::NodeHandle pHandle;
	
	pWorldSub = pHandle.subscribe("/t5k/world", 1000, worldCallback);
	pPathPub = pHandle.advertise<std_msgs::String>("/t5k/paths", 1000);
	
	ros::spin();
	
	return 0;
}
