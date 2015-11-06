#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

ros::Publisher pOut;

void shooterCallback(const std_msgs::String::ConstPtr& aMessage) {
	ROS_INFO("[STRATEGY] Got: %s, From: SHOOTER", aMessage->data.c_str());
}

void ballhandlingCallback(const std_msgs::String::ConstPtr& aMessage) {
	ROS_INFO("[STRATEGY] Got: %s, From: BALLHANDLING", aMessage->data.c_str());
}

void wheelControlCallback(const std_msgs::String::ConstPtr& aMessage) {
	ROS_INFO("[STRATEGY] Got: %s, From: WHEELCTRL", aMessage->data.c_str());
}

void pathfindingCallback(const std_msgs::String::ConstPtr& aMessage) {
	ROS_INFO("[STRATEGY] Got: %s, From: PATHFINDING", aMessage->data.c_str());
}

void worldCallback(const std_msgs::String::ConstPtr& aMessage) {
	ROS_INFO("[STRATEGY] Got: %s, From: WORLD", aMessage->data.c_str());
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "strategy");
	ros::NodeHandle pNodeHandle;
	ROS_INFO("Starting Strategy Node");
	ros::Subscriber pShooter = pNodeHandle.subscribe("turtle5k_shooter", 1000, shooterCallback);
	ros::Subscriber pBallHandler = pNodeHandle.subscribe("chatter", 1000, ballhandlingCallback);
	ros::Subscriber pWheelCtrl = pNodeHandle.subscribe("turtle5k_wheelctrl", 1000, wheelControlCallback);
	ros::Subscriber pPathfinding = pNodeHandle.subscribe("turtle5k_pathfinding", 1000, pathfindingCallback);
	ros::Subscriber pWorld = pNodeHandle.subscribe("turtle5k_world", 1000, worldCallback);
	ros::spin();
	
	return 0; 
}
