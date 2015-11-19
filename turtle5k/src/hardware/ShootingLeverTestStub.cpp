#include "headers/ShootingLeverTestStub.h"

ShootingLeverTestStub::ShootingLeverTestStub()
{
	this->angle = 0;
	this->lastAngleDisplayed = 0;
}

void ShootingLeverTestStub::setAngle(int angle)
{
	this->angle = angle;
}

int ShootingLeverTestStub::getAngle()
{
	this->lastAngleDisplayed += (this->lastAngleDisplayed > this->angle ? -(this->angle * 0.01) : (this->angle * 0.01));
	return this->lastAngleDisplayed;
}

void ShootingLeverTestStub::shoot(int meters)
{

}
