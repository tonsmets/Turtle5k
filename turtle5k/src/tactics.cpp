#include "ros/ros.h"
#include "ObjectTypes.h"
#include "geometry_msgs/Twist.h"
#include "turtle5k/WorldMessage.h"
#include "turtle5k/ShootMessage.h"
#include "turtle5k/BallHandlingMessage.h"
#include <angles/angles.h>

ros::Publisher pTacticsPub;
ros::Publisher	pTwistPub;
ros::Publisher pShootPub;
ros::Subscriber pPathSub;
ros::Subscriber pWorldSub;
ros::Subscriber pBallHandlingSub;
bool pBallGrabbed = false;

geometry_msgs::Twist GetTwistMessage(turtle5k::WorldMessage aMessage)
{
	geometry_msgs::Twist pReturn;
	memset(&pReturn, 0, sizeof(geometry_msgs::Twist));
	if(aMessage.objectType == OBJECT_BALL) {
		pReturn.linear.x = abs(sqrt((pow(aMessage.objectPosition.x, 2) + pow(aMessage.objectPosition.y, 2)))) / 60;
		pReturn.angular.z = (angles::normalize_angle((aMessage.angleBetween / (180/M_PI))) * - 3.5f);
	}
	else if(aMessage.objectType == OBJECT_GOAL) {
		pReturn.angular.z = (angles::normalize_angle((aMessage.angleBetween / (180/M_PI))) * - 3.5f);
	}
	else if(aMessage.objectType == OBJECT_LINE) {
		
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
	if(aMessage.objectType == OBJECT_GOAL && pBallGrabbed) {
		if(aMessage.angleBetween < (8 / (180 / M_PI))) {
			turtle5k::ShootMessage pMessage;
			pMessage.shootPower = 10;
			pMessage.shootAngle = 1;
			pShootPub.publish(pMessage);
		}
	}
	else {
		geometry_msgs::Twist pMessage = GetTwistMessage(aMessage);
		pTwistPub.publish(pMessage);
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "t5k_tactics");
	ros::NodeHandle pHandle;
	
	pWorldSub = pHandle.subscribe("/world", 1000, worldCallback);
	pTwistPub = pHandle.advertise<geometry_msgs::Twist>("/motorspeed_set", 1000);
	pShootPub = pHandle.advertise<turtle5k::ShootMessage>("/shooting_shoot", 1000);
	pBallHandlingSub = pHandle.subscribe("/ballhandling", 1000, ballhandlingCallback);
	
	ros::spin();
	
	return 0;
}
