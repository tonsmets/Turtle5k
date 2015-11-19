#include <string>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

#include "turtle5k/ShootingLever.h"
#include "../hardware/headers/ShootingLeverTestStub.h"

ros::Publisher pShootingInfoPub;
ros::Subscriber pShootingCmdSub;
ros::Subscriber pShootCmdSub;

ShootingLeverTestStub shootingLeverTestStub;

void frameCallback(const turtle5k::ShootingLever &aMessage)
{
	ROS_INFO("[SHOOTING] Got %i, For: Shooting", (int)aMessage.pShootingAngle);

	shootingLeverTestStub.setAngle(aMessage.pShootingAngle);
	ROS_INFO("setShootingAngle: [%i]", (int)aMessage.pShootingAngle);
}

void frameCallbackShoot(const std_msgs::Int32 &aMessage)
{
	ROS_INFO("[SHOOTING] Got %s, For: Shoot", std::to_string(aMessage.data).c_str());

	shootingLeverTestStub.shoot((int)aMessage.data);
	ROS_INFO("shoot: [%i]", (int)aMessage.data);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "t5k-tactics");

	ros::NodeHandle pHandle;
	ros::Rate pRate(24);

	pShootingInfoPub = pHandle.advertise<turtle5k::ShootingLever>("/t5k/shootinginfo", 1000);
	pShootingCmdSub = pHandle.subscribe("/t5k/shootingcommands", 1000, frameCallback);
	pShootCmdSub = pHandle.subscribe("/t5k/shootingshoot", 1000, frameCallbackShoot);

	while(ros::ok()) {

		double angle = shootingLeverTestStub.getAngle();

		turtle5k::ShootingLever pMessage;
		pMessage.pShootingAngle = angle;

		ROS_INFO_STREAM("Shooting angle: " << pMessage.pShootingAngle << std::endl);

		pShootingInfoPub.publish(pMessage);
		ros::spinOnce();
		pRate.sleep();
	}

	return 0;
}

