#include "ros/ros.h"
#include "std_msgs/String.h"
ros::Publisher pWheelCmdPub;
ros::Publisher pBallCmdPub;
ros::Subscriber pTacticsSub;

std_msgs::String getCommand()
{
	//TODO: create command from input
	return std_msgs::String();
}

void tacticsCallback(const std_msgs::String::ConstPtr& aMessage)
{
	ROS_INFO("[COMMANDS] Got %s, From: TACTICS", aMessage->data.c_str());
	std_msgs::String pMessage = getCommand();
	pBallCmdPub.publish(pMessage);
	pWheelCmdPub.publish(pMessage);
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "t5k-tactics");
	ros::NodeHandle pHandle;
	
	pWheelCmdPub = pHandle.advertise<std_msgs::String>("/t5k/wheelcontrolcommands", 1000);
	pBallCmdPub = pHandle.advertise<std_msgs::String>("/t5k/ballcontrolcommands", 1000);
	pTacticsSub = pHandle.subscribe("/t5k/tactics", 1000, tacticsCallback);
	
	ros::spin();
	
	return 0;
}
