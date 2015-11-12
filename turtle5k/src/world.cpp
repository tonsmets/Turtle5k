#include "ros/ros.h"
#include "std_msgs/String.h"
ros::Publisher pWorldPub;
ros::Subscriber pLocationSub;
ros::Subscriber pBallLocationSub;
ros::Subscriber pGoalLocationSub;

void locationCallback(const std_msgs::String::ConstPtr& aMessage)
{
	ROS_INFO("[WORLD] Got %s, From: LOCATION", aMessage->data.c_str());
	std_msgs::String pMessage;
	pMessage.data = "{location: [0, 0]}";
	pWorldPub.publish(pMessage);
}

void ballLocationCallback(const std_msgs::String::ConstPtr& aMessage)
{
	ROS_INFO("[WORLD] Got %s, From: BALLLOCATION", aMessage->data.c_str());
	std_msgs::String pMessage;
	pMessage.data = "{ballLocation: [0, 0]}";
	pWorldPub.publish(pMessage);
}

void goalLocationCallback(const std_msgs::String::ConstPtr& aMessage)
{
	ROS_INFO("[WORLD] Got %s, From: GOALLOCATION", aMessage->data.c_str());
	std_msgs::String pMessage;
	pMessage.data = "{goalLocation: [0, 0]}";
	pWorldPub.publish(pMessage);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "t5k-tactics");
	ros::NodeHandle pHandle;
	
	pWorldPub = pHandle.advertise<std_msgs::String>("/t5k/world", 1000);
	pLocationSub = pHandle.subscribe("/t5k/location", 1000, locationCallback);
	pBallLocationSub = pHandle.subscribe("/t5k/balllocation", 1000, ballLocationCallback);
	pGoalLocationSub = pHandle.subscribe("/t5k/goallocation", 1000, goalLocationCallback);
	ros::spin();
	return 0;
}
