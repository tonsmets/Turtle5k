#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char** argv) {
	ros::init(argc, argv, "turtle5k_localisation");
	ros::NodeHandle pNodeHandle;
	ros::Publisher pLocalisationPublisher = pNodeHandle.advertise<std_msgs::String>("turtle5k_localisation", 1000);
	ros::Rate pLoopRate(30);
	while(ros::ok()) {
		std_msgs::String pMessage;
		std::stringstream ss;
		ss<<"Localisation.Message[Params]";
		pMessage.data = ss.str();
		ROS_INFO("%s", pMessage.data.c_str());
		pLocalisationPublisher.publish(pMessage);
		ros::spinOnce();
		pLoopRate.sleep();
	}
	return 0;	
}
