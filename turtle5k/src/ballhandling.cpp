#include "ros/ros.h"
#include "std_msgs/String.h"
#include "turtle5k/BallHandlingMessage.h"

ros::Publisher pBallHandlingInfoPub;
ros::Subscriber pBallHandlingCmdSub;

turtle5k::BallHandlingMessage getHandlingParams()
{
	turtle5k::BallHandlingMessage pBMsg;
	pBMsg.pRevolutionsPerSecond = 60;
	return pBMsg;
}

void frameCallback(const std_msgs::String::ConstPtr& aMessage)
{
	ROS_INFO("[BALLHANDLING] Got %s, From: BALLCONTROL", aMessage->data.c_str());
	pBallHandlingInfoPub.publish(getHandlingParams());
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "t5k_ballhandling");
	ros::NodeHandle pHandle;
	ROS_INFO("Starting Ballhandling");
	pBallHandlingInfoPub = pHandle.advertise<std_msgs::String>("/t5k/ballhandlinginfo", 1000);
	pBallHandlingCmdSub = pHandle.subscribe("/t5k/ballhandlingcommands", 1000, frameCallback);
	while(ros::ok()) {
		pBallHandlingInfoPub.publish(getHandlingParams());
		ROS_INFO("RPS: %f", getHandlingParams().pRevolutionsPerSecond);
		ros::spinOnce();
	}
	ros::spin();
	
	return 0;
}
