#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>
#include <ros/console.h>

#include "../hardware/headers/BallHandlingTestStub.h"

ros::Publisher pBallHandlingInfoPub;
ros::Subscriber pBallHandlingCmdSub;

BallHandlingTestStub ballHandler;

void frameCallback(const std_msgs::String::ConstPtr& aMessage)
{
	ROS_INFO("[BALLHANDLING] Got %s, From: BALLCONTROL", aMessage->data.c_str());
	
	double newSpeed = atof(aMessage->data.c_str());
	ballHandler.setRotationSpeed(newSpeed);
	ROS_INFO("setRotationSpeed: [%f]", newSpeed);
}

int main(int argc, char** argv) {

	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) ) {
	   ros::console::notifyLoggerLevelsChanged();
	}

	ROS_INFO_STREAM("Hello, world!");
	ROS_INFO("Hello %s", "World");
	ROS_DEBUG("Hello %s", "World");
	ROS_DEBUG_STREAM("Hello " << "World");

	std::cout << "test" << std::endl;
	ROS_INFO_STREAM("Start2" << std::endl);

	ros::init(argc, argv, "t5ktactics");
	ros::NodeHandle pHandle;
	
	ROS_INFO_STREAM("Start" << std::endl);

	pBallHandlingInfoPub = pHandle.advertise<std_msgs::String>("/t5k/ballhandlinginfo", 1000);
	pBallHandlingCmdSub = pHandle.subscribe("/t5k/ballhandlingcommands", 1000, frameCallback);
	
	ros::Rate loop_rate(10);

	while (ros::ok()) {
		std_msgs::String msg;
		msg.data = std::to_string(ballHandler.getRotationSpeed());
		ROS_INFO("Process: %s", msg.data.c_str());

		ROS_INFO_STREAM("Data: " << msg.data << std::endl);

		pBallHandlingInfoPub.publish(msg);

		ros::spinOnce();
		loop_rate.sleep();
	}
		
	return 0;
}
