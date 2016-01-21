#include "ros/ros.h"
#include "ObjectTypes.h"
#include "geometry_msgs/Twist.h"
#include "turtle5k/WorldMessage.h"
#include "turtle5k/ShootMessage.h"
#include "turtle5k/BallHandlingMessage.h"
#include "turtle5k/BallHandlingOnOffMessage.h"
#include <angles/angles.h>

ros::Publisher pTacticsPub;
ros::Publisher	pTwistPub;
ros::Publisher pShootPub;
ros::Publisher pBallHandlingPub;
geometry_msgs::Twist pTwistMessage;
ros::Subscriber pPathSub;
ros::Subscriber pWorldSub;
ros::Subscriber pBallHandlingSub;

bool pBallGrabbed = false;

geometry_msgs::Twist GetTwistMessage(turtle5k::WorldMessage aMessage)
{
	geometry_msgs::Twist pReturn;
	memset(&pReturn, 0, sizeof(geometry_msgs::Twist));
	if(aMessage.objectType == OBJECT_BALL) {
		pReturn.linear.x = sqrt((pow(aMessage.objectPosition.x, 2) + pow(aMessage.objectPosition.y, 2))) / 30;
		if(pReturn.linear.x > 2.3)
		{
			pReturn.linear.x = 2.3;
		}
		pReturn.angular.z = (angles::normalize_angle((aMessage.angleBetween / (180/M_PI))) * - 3);
	}
	else if(aMessage.objectType == OBJECT_GOAL) {
		//pReturn.angular.z = (angles::normalize_angle((aMessage.angleBetween / (180/M_PI))) * - 1.5);
	}
	else if(aMessage.objectType == OBJECT_LINE) {
		
	}
	else {
		pReturn.angular.z = 0;
		pReturn.angular.x = 0;
	}
	return pReturn;
}

void ballhandlingCallback(const turtle5k::BallHandlingMessage aMessage)
{
	if(aMessage.ballGrabbed) {
		pBallGrabbed = true;
	}
	else {
		pBallGrabbed = false;
	}
}

void worldCallback(const turtle5k::WorldMessage& aMessage)
{
	memset(&pTwistMessage, 0, sizeof(pTwistMessage));

	if(aMessage.objectType == OBJECT_GOAL && pBallGrabbed) {
		if(aMessage.angleBetween < (8 / (180 / M_PI))) {
			turtle5k::ShootMessage pMessage;
			pMessage.shootPower = 10;
			pMessage.shootAngle = 1;
			pShootPub.publish(pMessage);
		}
	}
	else {
		pTwistMessage = GetTwistMessage(aMessage);
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "t5k_tactics");
	ros::NodeHandle pHandle;
	ros::Rate pRate(100);
	
	pWorldSub = pHandle.subscribe("/world", 1000, worldCallback);
	pTwistPub = pHandle.advertise<geometry_msgs::Twist>("/motorspeed_set", 1000);
	pShootPub = pHandle.advertise<turtle5k::ShootMessage>("/shoot", 1000);
	pBallHandlingSub = pHandle.subscribe("/ballhandling", 1000, ballhandlingCallback);
	pBallHandlingPub = pHandle.advertise<turtle5k::BallHandlingOnOffMessage>("/handle", 1000);
	
	memset(&pTwistMessage, 0 , sizeof(pTwistMessage));
	
	turtle5k::BallHandlingOnOffMessage ballhandlingMsg;
	ballhandlingMsg.on = true;
	
	turtle5k::ShootMessage shootMsg;
	shootMsg.shootPower = 2;
	shootMsg.shootAngle = 9;

	pWorldSub = pHandle.subscribe("/world", 1000, worldCallback);
	pTwistPub = pHandle.advertise<geometry_msgs::Twist>("/motorspeed_set", 1000);
	pShootPub = pHandle.advertise<turtle5k::ShootMessage>("/shooting_shoot", 1000);
	pBallHandlingSub = pHandle.subscribe("/ballhandling", 1000, ballhandlingCallback);
	
	while(ros::ok()) {
		pTwistPub.publish(pTwistMessage);
		pBallHandlingPub.publish(ballhandlingMsg);
		if(pBallGrabbed) {
			ROS_INFO("We have the ball");
			//std::cout << "Got the ball" << std::endl;
			pShootPub.publish(shootMsg);
		}
		else {
			ROS_INFO("We don't have the ball");
		}
		ros::spinOnce();
		pRate.sleep();
	}
	
	return 0;
}
