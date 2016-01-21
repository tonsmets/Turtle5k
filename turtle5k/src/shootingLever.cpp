#include "ros/ros.h"
#include "std_msgs/String.h"
#include "turtle5k/ShootMessage.h"
#include "turtle5k/BallHandlingMessage.h"
#include "turtle5k/BallHandlingOnOffMessage.h"

#include 		<stdio.h>
#include 		<string.h>
#include 		<unistd.h>
#include 		<fcntl.h>
#include 		<errno.h>
#include 		<termios.h>
#include 		<time.h>
#include 		<stdint.h>
#include 		<iostream>
#include 		<cmath>
#include 		<stdlib.h>
#include		<ctime>
#include 		<time.h>	

using namespace std;

#define SENSOR_LEFT 	0x04
#define SENSOR_RIGHT	0x03
#define LEFT		0x0008
#define RIGHT		0x0009
#define CLK_SCALE		0x1388
#define HALL_DIFF		0.7f

ros::Publisher		pBallHandlingPub;
ros::Subscriber		pShoot;
ros::Subscriber		pHandle;


static int	pLevel_Left 	= 0,
			pLevel_Right 	= 0,
			pDegree_Left	= 0,
			pDegree_Right	= 0,
			pScale_Left		= 0,
			pScale_Right	= 0,
			pClkCompare		= 0,
			clock_count		= 0,
			read_count		= 0;
			
static clock_t	pClkHandle,
				pClkHandlePart;
				
static int16_t	inRange		 = false,
				pHasBall	 = false,
				toShoot		 = false,
				toCount		 = false;
			
static int32_t  power 		 = 0,
				angle		 = 0;
				
static uint8_t	lever_level=0x00;
		
void timerCallback(const ros::TimerEvent&)
{
	if(toCount)
	clock_count++;
	
	read_count++;
}	
void shootCallback(const turtle5k::ShootMessage& aMessage)
{
	power=aMessage.shootPower;
	angle=aMessage.shootAngle;
	toShoot=true;
}
void handleCallback(const turtle5k::BallHandlingOnOffMessage&  aMessage)
{
	inRange=aMessage.on;
}
void shoot(int port,char power)
{
	uint8_t bSend[] = {0x5A, 0x02, 0x02, 0x0F, 0x00, 0x0D};
	for(int i=0;i<power*5;i++)
	{
		write(port, bSend, 6);
	}
}
void level(int port)
{
	uint8_t bSend[6] = {0x5a,0x01,0x00,0x00,0x09,0x0d};	
	write(port, bSend, 6);
} 
void ReadSensor(int port, uint8_t aArm, int side, int& aScale)
{
	uint8_t	pSend[] = { 0x5A, aArm, 0x00, 0x00, lever_level, 0x0d },
			pBuffer[6];
	write(port, pSend, sizeof(pSend));
	read(port, &pBuffer, sizeof(pBuffer));
	
	aScale = ((((int)pBuffer[4] - side) / 255) + ((int)pBuffer[3] - side));
}

void SetSpeed(int port, int aSign)
{
	int16_t	pWheelVel = 130*aSign;
	uint8_t	pSend[] = { 0x5A, 0xAA, 0x03, (uint8_t)(pWheelVel & 0xFF), (uint8_t)((pWheelVel >> 8) & 0xFF), 0x00, 0x00, 0x00 };
	write(port, pSend, sizeof(pSend));
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "t5k_ballhandling");
	ros::NodeHandle pNode;
	pBallHandlingPub = pNode.advertise<turtle5k::BallHandlingMessage>("/ballhandling", 1000);
	pShoot			 = pNode.subscribe("/shoot", 1000, shootCallback);
	pHandle			 = pNode.subscribe("/handle", 1000, handleCallback);
	ros::Timer timer = pNode.createTimer(ros::Duration(0.2), timerCallback);
	
	while(ros::ok())
	{
		int port1 = open("/dev/ttyS4", O_RDWR | O_NONBLOCK);
		int port2 = open("/dev/ttyS6", O_RDWR | O_NONBLOCK);
		int port3 = open("/dev/ttyS9", O_RDWR | O_NONBLOCK);
		
		if(inRange)
		{
			if(read_count==3) 
			{
				pClkCompare++;
				
				ReadSensor(port1, SENSOR_RIGHT, RIGHT, pScale_Right);
				ReadSensor(port1, SENSOR_LEFT, LEFT, pScale_Left);
				
				pScale_Left+=HALL_DIFF;
				
				ROS_INFO("%i       %i",pScale_Left,pScale_Right);
				if(pScale_Left < 5 || pScale_Right <5)
				{
					pHasBall=true;
				}
				else
				{
					pHasBall=false;
				}
				
				turtle5k::BallHandlingMessage pMessage;
				pMessage.ballGrabbed = pHasBall;
				pBallHandlingPub.publish(pMessage);
				read_count=0;
			}
			
			SetSpeed(port2, -1);
			SetSpeed(port3,  1);

			if(toShoot)
			{
				toCount=true;
				lever_level=angle;
				if(clock_count==5)
				{
					shoot(port1,power);
				}
				if(clock_count==10)
				{
					lever_level=0;
					toCount=false;
					toShoot=false;
				}
			}
		}
		ros::spinOnce();
		close(port1);
		close(port2);
		close(port3);
	}
	
	return 0;
}
