#include <string>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "../hardware/headers/ShootingLeverTestStub.h"

ros::Publisher pShootingInfoPub;
ros::Subscriber pShootingCmdSub;

ShootingLeverTestStub shootingLeverTestStub;

void frameCallback(const std_msgs::String::ConstPtr& aMessage)
{
	ROS_INFO("[SHOOTING] Got %s, For: Shooting", aMessage->data.c_str());

	int newAngle = atoi(aMessage->data.c_str());
	shootingLeverTestStub.setAngle(newAngle);
	ROS_INFO("setShootingAngle: [%i]", newAngle);
}

void frameCallbackShoot(const std_msgs::String::ConstPtr& aMessage)
{
	ROS_INFO("[SHOOTING] Got %s, For: Shoot", aMessage->data.c_str());

	int meters = atoi(aMessage->data.c_str());
	shootingLeverTestStub.shoot(meters);
	ROS_INFO("shoot: [%i]", meters);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "t5k-tactics");

	ros::NodeHandle pHandle;
	ros::Rate pRate(24);

	pShootingInfoPub = pHandle.advertise<std_msgs::String>("/t5k/shootinginfo", 1000);
	pShootingCmdSub = pHandle.subscribe("/t5k/shootingcommands", 1000, frameCallback);
	//pShootingCmdSub = pHandle.subscribe("/t5k/shootingshoot", 1000, frameCallbackShoot);


	while(ros::ok()) {

		double angle = shootingLeverTestStub.getAngle();

		std_msgs::String pMessage;
		pMessage.data = std::to_string(angle);

		//ROS_INFO_STREAM("Shooting angle: " << pMessage.data << std::endl);

		pShootingInfoPub.publish(pMessage);
		ros::spinOnce();
		pRate.sleep();
	}

	return 0;
}

