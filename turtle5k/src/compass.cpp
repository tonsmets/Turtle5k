#include "ros/ros.h"
#include "std_msgs/String.h"

ros::Publisher pCompassDataPub;


int main(int argc, char** argv) {
	ros::init(argc, argv, "t5k_compass");
	ros::NodeHandle pHandle;
	ros::Rate pRate(24);
	
	pCompassDataPub = pHandle.advertise<std_msgs::String>("/t5k/compassdata", 1000);
	
	while(ros::ok()) {
		std_msgs::String pMessage;
		pMessage.data = "compassdata : {0, 0, 0, 0}";
		pCompassDataPub.publish(pMessage);
		ros::spinOnce();
		pRate.sleep();
	}
	
	return 0;
}
