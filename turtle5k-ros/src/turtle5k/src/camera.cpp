#include "ros/ros.h"
#include "std_msgs/String.h"

ros::Publisher pFramePub;


int main(int argc, char** argv) {
	ros::init(argc, argv, "t5k_camera");
	ros::NodeHandle pHandle;
	ros::Rate pRate(24);
	
	pFramePub = pHandle.advertise<std_msgs::String>("/t5k/frame", 1000);
	while(ros::ok()) {
		std_msgs::String pMessage;
		pMessage.data = "frame:{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}";
		pFramePub.publish(pMessage);
		ros::spinOnce();
		pRate.sleep();
	}
	
	return 0;
}
