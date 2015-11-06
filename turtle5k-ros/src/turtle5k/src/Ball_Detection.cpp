#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char** argv) {
	ros::init(argc, argv, "turtle5k_ball_detection");
	ros::NodeHandle pNodeHandle;
	ros::Publisher pBallDetectionPublisher = pNodeHandle.advertise<std_msgs::String>("turtle5k_ball_detection", 1000);
	ros::Rate pLoopRate(30);
	while(ros::ok()) {
		std_msgs::String pMessage;
		std::stringstream ss;
		ss<<"Ball_Detection.Message[Params]";
		pMessage.data = ss.str();
		ROS_INFO("%s", pMessage.data.c_str());
		pBallDetectionPublisher.publish(pMessage);
		ros::spinOnce();
		pLoopRate.sleep();
	}
	return 0;	
}
