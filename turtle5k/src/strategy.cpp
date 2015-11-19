#include "ros/ros.h"
#include "std_msgs/String.h"


int main(int argc, char** argv) {
	ros::init(argc, argv, "t5k_strategy");
	ros::NodeHandle pHandle;
	
	ros::Publisher pStrategyPub = pHandle.advertise<std_msgs::String>("/t5k/strategy", 1000);
	ros::Rate pRate(10);
	
	while(ros::ok()) {
		std_msgs::String pMessage;
		pMessage.data = "t5k_strategy";
		pStrategyPub.publish(pMessage);
		ros::spinOnce();
		pRate.sleep();
	}
	
	return 0;
}
