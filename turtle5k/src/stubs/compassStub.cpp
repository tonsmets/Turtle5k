#include "ros/ros.h"
#include "std_msgs/String.h"

#include <string>

#include "../hardware/headers/CompassTestStub.h"

ros::Publisher pCompassDataPub;

CompassTestStub compassTestStub;

int main(int argc, char** argv) {
	ros::init(argc, argv, "t5k-tactics");
	ros::NodeHandle pHandle;
	ros::Rate pRate(24);
	
	pCompassDataPub = pHandle.advertise<std_msgs::String>("/t5k/compassdata", 1000);
	
	while(ros::ok()) {
		std_msgs::String pMessage;

		double degress = compassTestStub.getDegrees();

		pMessage.data = std::to_string(degress);

		ROS_INFO_STREAM("Data: " << pMessage.data << std::endl);

		pCompassDataPub.publish(pMessage);
		ros::spinOnce();
		pRate.sleep();
	}
	
	return 0;
}

