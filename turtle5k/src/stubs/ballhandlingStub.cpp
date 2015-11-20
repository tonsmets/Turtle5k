#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>
#include <ros/console.h>

#include "turtle5k/BallHandling.h"
#include "../hardware/headers/BallHandlingTestStub.h"

ros::Publisher pBallHandlingInfoPub;
ros::Subscriber pBallHandlingCmdSub;

BallHandlingTestStub ballHandler;

void frameCallback(const turtle5k::BallHandling &aMessage)
{
	ROS_INFO("[BALLHANDLING] Got %s, From: BALLCONTROL", std::to_string(aMessage.pRevolutionsPerSecond).c_str());
	
	ballHandler.setRotationSpeed(aMessage.pRevolutionsPerSecond);
	ROS_INFO("setRotationSpeed: [%f]", aMessage.pRevolutionsPerSecond);
}

int main(int argc, char** argv) {

	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) ) {
	   ros::console::notifyLoggerLevelsChanged();
	}

	ros::init(argc, argv, "t5k_ballhandling");
	ros::NodeHandle pHandle;

	pBallHandlingInfoPub = pHandle.advertise<turtle5k::BallHandling>("/t5k/ballhandlinginfo", 1000);
	pBallHandlingCmdSub = pHandle.subscribe("/t5k/ballhandlingcommands", 1000, frameCallback);
	
	ros::Rate loop_rate(10);

	ballHandler.setRotationSpeed(10);

	while (ros::ok()) {
		turtle5k::BallHandling msg;
		msg.pRevolutionsPerSecond = ballHandler.getRotationSpeed();
		ROS_INFO("Process: %s", std::to_string(msg.pRevolutionsPerSecond).c_str());
		ROS_INFO_STREAM("Data: " << msg.pRevolutionsPerSecond << std::endl);

		pBallHandlingInfoPub.publish(msg);

		ros::spinOnce();
		loop_rate.sleep();
	}
		
	return 0;
}
