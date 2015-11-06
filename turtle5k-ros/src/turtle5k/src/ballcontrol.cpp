#include "ros/ros.h"
#include "std_msgs/String.h"
ros::Publisher pBallHandlingCmdPub;
ros::Publisher pShootingCmdPub;
ros::Publisher pBallControlInfoPub;
ros::Subscriber pBallControlCmdSub;
ros::Subscriber pCurrVelocitySub;
ros::Subscriber pShootingInfoSub;
ros::Subscriber pBallHandlingInfoSub;

std_msgs::String getFeedback()
{
	//TODO: create command from input
	return std_msgs::String();
}

void ballControlCommandsCallback(const std_msgs::String::ConstPtr& aMessage)
{
	ROS_INFO("[BALLCONTROL] Got %s, From: COMMANDS", aMessage->data.c_str());
	std_msgs::String pMessage = getFeedback();
	pBallHandlingCmdPub.publish(pMessage);
	pShootingCmdPub.publish(pMessage);
	pBallControlInfoPub.publish(pMessage);
}

void velocityCallback(const std_msgs::String::ConstPtr& aMessage)
{
	ROS_INFO("[BALLCONTROL] Got %s, From: WHEELCONTROL", aMessage->data.c_str());
	std_msgs::String pMessage = getFeedback();
	pBallHandlingCmdPub.publish(pMessage);
	pShootingCmdPub.publish(pMessage);
	pBallControlInfoPub.publish(pMessage);
}

void shootingInfoCallback(const std_msgs::String::ConstPtr& aMessage)
{
	ROS_INFO("[BALLCONTROL] Got %s, From: SHOOTING", aMessage->data.c_str());
	std_msgs::String pMessage = getFeedback();
	pBallHandlingCmdPub.publish(pMessage);
	pShootingCmdPub.publish(pMessage);
	pBallControlInfoPub.publish(pMessage);
}

void ballHandlingInfoCallback(const std_msgs::String::ConstPtr& aMessage)
{
	ROS_INFO("[BALLCONTROL] Got %s, From: BALLCONTROL", aMessage->data.c_str());
	std_msgs::String pMessage = getFeedback();
	pBallHandlingCmdPub.publish(pMessage);
	pShootingCmdPub.publish(pMessage);
	pBallControlInfoPub.publish(pMessage);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "t5k-tactics");
	ros::NodeHandle pHandle;
	
	pBallHandlingCmdPub = pHandle.advertise<std_msgs::String>("/t5k/ballhandlingcommands", 1000);
	pShootingCmdPub = pHandle.advertise<std_msgs::String>("/t5k/shootingcommands", 1000);
	pBallControlInfoPub = pHandle.advertise<std_msgs::String>("/t5k/ballcontrolinfo", 1000);
	pBallControlCmdSub = pHandle.subscribe("/t5k/ballcontrolcommands", 1000, ballControlCommandsCallback);
	pCurrVelocitySub = pHandle.subscribe("/t5k/velocity", 1000, velocityCallback);
	pShootingInfoSub = pHandle.subscribe("/t5k/shootinginfo", 1000, shootingInfoCallback);
	pBallHandlingInfoSub = pHandle.subscribe("/t5k/ballhandlinginfo", 1000, ballHandlingInfoCallback);
	
	ros::spin();
	
	return 0;
}
