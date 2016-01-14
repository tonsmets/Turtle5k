#include "ros/ros.h"
#include "turtle5k/ShootMessage.h"

ros::Subscriber pShootSub;

void shootCallback(const turtle5k::ShootMessage &aMessage)
{
	//Shoot the ball with params from aMessage
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "t5k-tactics");
	ros::NodeHandle pHandle;

	pShootSub = pHandle.subscribe("/t5k/shooting_shoot", 1000, shootCallback);

	while(ros::ok()) {
		
		ros::spinOnce();
	}

	return 0;
}

