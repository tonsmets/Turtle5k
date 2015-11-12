#include "ros/ros.h"
#include "std_msgs/String.h"
ros::Publisher pLocationPub;
ros::Subscriber pEncoderDataSub;
ros::Subscriber pCompassDataSub;
ros::Subscriber pFrameSub;

std_msgs::String getLocation()
{
	//TODO: create location from input
	return std_msgs::String();
}

void frameCallback(const std_msgs::String::ConstPtr& aMessage)
{
	ROS_INFO("[POSITIONING] Got %s, From: CAMERA", aMessage->data.c_str());
	std_msgs::String pMessage = getLocation();
	pLocationPub.publish(pMessage);
}

void encoderDataCallback(const std_msgs::String::ConstPtr& aMessage)
{
	ROS_INFO("[POSITIONING] Got %s, From: WHEELDRIVER", aMessage->data.c_str());
	std_msgs::String pMessage = getLocation();
	pLocationPub.publish(pMessage);
}

void compassDataCallback(const std_msgs::String::ConstPtr& aMessage)
{
	ROS_INFO("[POSITIONING] Got %s, From: COMPASS", aMessage->data.c_str());
	std_msgs::String pMessage = getLocation();
	pLocationPub.publish(pMessage);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "t5k-tactics");
	ros::NodeHandle pHandle;
	
	pLocationPub = pHandle.advertise<std_msgs::String>("/t5k/location", 1000);
	pFrameSub = pHandle.subscribe("/t5k/frame", 1000, frameCallback);
	pCompassDataSub = pHandle.subscribe("/t5k/compassdata", 1000, compassDataCallback);
	pEncoderDataSub = pHandle.subscribe("/t5k/encoderdata", 1000, encoderDataCallback);
	
	ros::spin();
	
	return 0;
}
