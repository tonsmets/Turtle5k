#include "ros/ros.h"
#include "std_msgs/String.h"
#include "turtle5k/BallHandlingMessage.h"

#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>

using namespace std;

#define SENSOR_LEFT 	0x0004
#define SENSOR_RIGHT	0x0003
#define SCALE_LEFT		0x0008
#define SCALE_RIGHT		0x0009
#define CLK_SCALE		0x1388
#define	NORM_MIN_SPD	0x001E
#define NORM_MAX_SPD	0x003C
#define HALL_DIFF		0.7f

ros::Publisher pBallHandlingPub;

const char* pDevices[] = 
{
	"/dev/ttyS6",
	"/dev/ttyS9"
};

static int	pPorts[sizeof(pDevices)];
static int	pLevel_Left 	= 0,
			pLevel_Right 	= 0,
			pDegree_Left	= 0,
			pDegree_Right	= 0,
			pClkCompare		= 0;
			
static clock_t	pClkNow,
				pClkPart;
				
static int16_t	pWheelVel_Left,
				pWheelVel_Right,
				pHasBall = false;
void OpenPorts(const char* aDevices[], int aPorts[]) {
	for(int i = 0; i < sizeof(pDevices); ++i) {
		aPorts[i] = open(pDevices[i], O_RDWR | O_NONBLOCK);
	}
}

void ClosePorts(int aPorts[]) {
	for(int i = 0; i < sizeof(aPorts); ++i) {
		close(aPorts[i]);
	}
}

void ReadSensor(int& aLevel, int& aDegree, int aScale, int aPort, uint8_t aArm)
{
	uint8_t	pSend[] = { 0x5A, aArm, 0x00, 0x00, 0x00, 0x0D },
			pBuffer[6];
			
	write(aPort, pSend, sizeof(pSend));
	read(aPort, pBuffer, sizeof(pBuffer));
	
	aLevel = (int)pBuffer[3] - aScale;
	aDegree = (int)pBuffer[4] - aScale;
}

int16_t CalculateSpeed(int aLevel, int aDegree, int aSign)
{
	double  pScale = aSign ? ((aDegree / 255) + aLevel) : ((aDegree / 255) + aLevel + HALL_DIFF);
	pHasBall = (pScale < 4);
	return aSign * 110;
}

void SetSpeed(int aLevel, int aDegree, int aPort, int aSign)
{
	OpenPorts(pDevices, pPorts);
	pClkNow = clock();
	
	if(pClkPart == pClkCompare) {
			pClkCompare++;
			ReadSensor(pLevel_Right, pDegree_Right, SCALE_RIGHT, pPorts[0], SENSOR_RIGHT);
			ReadSensor(pLevel_Left, pDegree_Left, SCALE_LEFT, pPorts[0], SENSOR_LEFT);
	}
	else 
	{
			pClkPart = pClkNow / CLK_SCALE;
	}

	int16_t	pWheelVel = CalculateSpeed(aLevel, aDegree, aSign);
	uint8_t	pSend[] = { 0x5A, 0xAA, 0x07, (uint8_t)(pWheelVel & 0xFF), (uint8_t)((pWheelVel >> 8) & 0xFF), 0x00, 0x00, 0x00 };
	write(aPort, pSend, sizeof(pSend));
	ClosePorts(pPorts);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "t5k_ballhandling");
	ros::NodeHandle pHandle;
	pBallHandlingPub = pHandle.advertise<turtle5k::BallHandlingMessage>("/ballhandling", 1000);
	
	while(ros::ok()) 
	{
		SetSpeed(pLevel_Left, pDegree_Left, pPorts[0], 1);
		SetSpeed(pLevel_Right, pDegree_Right, pPorts[1], -1);
		
		turtle5k::BallHandlingMessage pMessage;
		pMessage.ballGrabbed = pHasBall;
		pBallHandlingPub.publish(pMessage);
		
		ros::spinOnce();
	}
	return 0;
}
