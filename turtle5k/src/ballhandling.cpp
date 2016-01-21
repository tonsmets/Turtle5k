#include "ros/ros.h"
#include "std_msgs/String.h"
#include "turtle5k/BallHandlingMessage.h"
#include "turtle5k/BallHandlingOnOffMessage.h"

#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>

using namespace std;

#define SENSOR_LEFT 	0x04
#define SENSOR_RIGHT	0x03
#define SCALE_LEFT		8
#define SCALE_RIGHT		9
#define CLK_SCALE		5000
#define	NORM_MIN_SPD	30
#define NORM_MAX_SPD	60
#define HALL_DIFF		0.7

ros::Publisher pBallHandlingPub;
ros::Subscriber pBallHandlingSub;

enum StateEnum
{
	OFF,
	ON
};

StateEnum pState = StateEnum::OFF;

const char* pDevices[] = 
{
	"/dev/ttyS4",
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
				pWheelVel_Right;
				
int leftArm,
	rightArm;
				
bool 			pHasBall = false;
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
	uint8_t	pSend[] = { 0x5A, 0x00, 0x00, 0x00, 0x00, 0x0D },
			pBuffer[6];
			pSend[1] = aArm;
			
	write(aPort, pSend, sizeof(pSend));
	read(aPort, pBuffer, sizeof(pBuffer));
	
	if(aArm == SENSOR_LEFT) {
		leftArm = (int)pBuffer[4];
	}
	if(aArm == SENSOR_RIGHT) {
		rightArm = (int)pBuffer[4];
	}
	
	std::cout << "pBuffer3: " <<(int)pBuffer[3] << std::endl;
	std::cout << "pBuffer4: " << (int)pBuffer[4] << std::endl;

	aLevel = (int)pBuffer[3] - aScale;
	aDegree = (int)pBuffer[4] - aScale;
}

int16_t CalculateSpeed(int aLevel, int aDegree, int aSign)
{
	double  pScale;
	if(aSign > 0) {
		pScale = (aDegree / 255) + aLevel;
	} else {
		pScale = (aDegree / 255) + aLevel + HALL_DIFF;
	}
	
	pHasBall = !(rightArm > 180 && leftArm < 190);
	std::cout << "left: " << leftArm << std::endl;
	std::cout << "right: " << rightArm << std::endl;
	
	//std::cout << "pScale: " << pScale << std::endl;
	
	return aSign * 110;
}

void SetSpeed(int aLevel, int aDegree, int aPort, int aSign)
{
	OpenPorts(pDevices, pPorts);
	pClkNow = clock();
	
	if(pClkPart > pClkCompare) {
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

void BallHandlingOnOffMessageCallback(const turtle5k::BallHandlingOnOffMessage::ConstPtr msg)
{
	if(msg->on)
	{
		pState = StateEnum::ON;
	}
	else
	{
		pState = StateEnum::OFF;
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "t5k_ballhandling");
	ros::NodeHandle pHandle;
	pBallHandlingPub = pHandle.advertise<turtle5k::BallHandlingMessage>("/ballhandling", 1);
	pBallHandlingSub = pHandle.subscribe("/ballhandling/on_off", 1000, BallHandlingOnOffMessageCallback);
	while(ros::ok()) 
	{
		turtle5k::BallHandlingMessage pMessage;
		//ReadSensor(pLevel_Right,pDegree_Right,SCALE_RIGHT,pPorts[0],SENSOR_RIGHT);						
		//ReadSensor(pLevel_Left,pDegree_Left,SCALE_LEFT,pPorts[0],SENSOR_LEFT);
		//if(pState == StateEnum::ON)
		if(true)
		{
			SetSpeed(pLevel_Left, pDegree_Left, pPorts[1], -1);
			SetSpeed(pLevel_Right, pDegree_Right, pPorts[2], 1);
			pMessage.ballGrabbed = pHasBall;
		}
		else
		{
			pMessage.ballGrabbed = false;
		}
		pBallHandlingPub.publish(pMessage);
		
		ros::spinOnce();
	}

	ClosePorts(pPorts);

	return 0;
}
