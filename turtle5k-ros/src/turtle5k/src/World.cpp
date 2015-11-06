#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"

void localisationCallback(const std_msgs::String::ConstPtr& aMessage)
{
	ROS_INFO("[WORLD] Got %s, From: LOCALISATION", aMessage->data.c_str());
}

void ball_detectionCallback(const std_msgs::String::ConstPtr& aMessage)
{
	ROS_INFO("[WORLD] Got %s, From: BALL_DETECTION", aMessage->data.c_str());
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "turtle5k_world");
	ros::NodeHandle pNodeHandle;
	ros::Publisher pWorldPublisher = pNodeHandle.advertise<std_msgs::String>("turtle5k_wheelctrl", 1000);
	ros::Subscriber pLocalisationSub = pNodeHandle.subscribe("turtle5k_localisation", 1000, localisationCallback);
	ros::Subscriber pBallDetectionSub = pNodeHandle.subscribe("turtle5k_ball_detection", 1000, ball_detectionCallback);
	ros::Rate pLoopRate(30);
	while(ros::ok()) {
		std_msgs::String pMessage;
		std::stringstream ss;
		ss<<"World.Message[Params]";
		pMessage.data = ss.str();
		ROS_INFO("%s", pMessage.data.c_str());
		ros::spinOnce();
		pWorldPublisher.publish(pMessage);
		pLoopRate.sleep();
	}
	return 0;	
}
