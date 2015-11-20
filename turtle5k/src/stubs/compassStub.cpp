#include "ros/ros.h"
#include "std_msgs/String.h"

#include <string>

#include "turtle5k/Compass.h"
#include "../hardware/headers/CompassTestStub.h"

ros::Publisher pCompassDataPub;

CompassTestStub compassTestStub;

int main(int argc, char** argv) {
	ros::init(argc, argv, "t5k-tactics");
	ros::NodeHandle pHandle;
	ros::Rate pRate(24);
	
	pCompassDataPub = pHandle.advertise<turtle5k::Compass>("/t5k/compassdata", 1000);
	
	while(ros::ok()) {
		turtle5k::Compass pMessage;

		double degress = compassTestStub.getDegrees();

		pMessage.pRadiansFromNorth = degress;

		ROS_INFO_STREAM("Data: " << pMessage.pRadiansFromNorth << std::endl);

		pCompassDataPub.publish(pMessage);
		ros::spinOnce();
		pRate.sleep();
	}
	
	return 0;
}

